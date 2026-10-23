#include "OutlineRenderer.h"

#include <mujoco/mujoco.h>

#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QVector2D>
#include <QtGlobal>

#include <algorithm>
#include <cmath>

// glClipControl 不在 QOpenGLExtraFunctions 中（非 GLES），需自行解析。
using PfnGlClipControl = void (QOPENGLF_APIENTRYP)(GLenum origin, GLenum depth);

// 桌面 GL 常量（Qt 的 GLES 头里可能没有定义）。
#ifndef GL_ZERO_TO_ONE
#  define GL_ZERO_TO_ONE 0x935F
#endif
#ifndef GL_NEGATIVE_ONE_TO_ONE
#  define GL_NEGATIVE_ONE_TO_ONE 0x935E
#endif
#ifndef GL_LOWER_LEFT
#  define GL_LOWER_LEFT 0x8CA1
#endif
#ifndef GL_MULTISAMPLE
#  define GL_MULTISAMPLE 0x809D
#endif
#ifndef GL_RGBA8
#  define GL_RGBA8 0x8058
#endif
#ifndef GL_RGBA16F
#  define GL_RGBA16F 0x881A
#endif
#ifndef GL_HALF_FLOAT
#  define GL_HALF_FLOAT 0x140B
#endif
#ifndef GL_DEPTH_COMPONENT24
#  define GL_DEPTH_COMPONENT24 0x81A6
#endif
#ifndef GL_DEPTH_COMPONENT
#  define GL_DEPTH_COMPONENT 0x1902
#endif
#ifndef GL_DEPTH_ATTACHMENT
#  define GL_DEPTH_ATTACHMENT 0x8D00
#endif
#ifndef GL_TEXTURE_COMPARE_MODE
#  define GL_TEXTURE_COMPARE_MODE 0x884C
#endif
#ifndef GL_CLAMP_TO_EDGE
#  define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_SCISSOR_TEST
#  define GL_SCISSOR_TEST 0x0C11
#endif
#ifndef GL_SCISSOR_BOX
#  define GL_SCISSOR_BOX 0x0C10
#endif
#ifndef GL_COLOR_BUFFER_BIT
#  define GL_COLOR_BUFFER_BIT 0x00004000
#endif
#ifndef GL_DEPTH_BUFFER_BIT
#  define GL_DEPTH_BUFFER_BIT 0x00000100
#endif

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;

// ---------------------------------------------------------------------------
// 法线 pass：只写视空间法线（RGB=0.5+0.5*n，A=1 表示有几何）。
// 双面处理用 gl_FrontFacing：MuJoCo 的 mesh 法线由绕序算出，法线与绕序始终一致，
// 因此绕序反的面（STL 历史问题）看到的是背面 → 取反即恢复朝外，不会凭空产生折痕。
// ---------------------------------------------------------------------------
const char* kNormalVertexShader = R"GLSL(
#version 330
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
out vec3 vNormalView;
void main() {
    vNormalView = uNormalMat * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)GLSL";

const char* kNormalFragmentShader = R"GLSL(
#version 330
in vec3 vNormalView;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormalView);
    if (!gl_FrontFacing) n = -n;
    fragColor = vec4(n * 0.5 + 0.5, 1.0);
}
)GLSL";

// ---------------------------------------------------------------------------
// 边缘检测 pass：全屏三角形；按 8 邻域比较法线夹角与相对深度差。
// 深度是本 pass 自建 G-buffer 的标准 GL 深度（near→0, far→1），线性化后比较。
// ---------------------------------------------------------------------------
const char* kEdgeVertexShader = R"GLSL(
#version 330
out vec2 vUv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = p;                                   // 0..2，覆盖整屏
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

const char* kEdgeFragmentShader = R"GLSL(
#version 330
in vec2 vUv;
uniform sampler2D uNormalTex;
uniform sampler2D uDepthTex;
uniform vec2  uTexel;             // 1 / 场景视口尺寸
uniform float uWidthPx;           // 线宽（像素）
uniform float uDepthThreshold;    // 二阶差分的相对阈值（真正的深度断裂）
uniform float uNormalCos;         // cos(折痕角)
uniform int   uCreases;           // 1 = 画折痕线
uniform vec4  uColor;
uniform float uNear;
uniform float uFar;
uniform int   uOrtho;
out vec4 fragColor;

float linDepth(vec2 uv) {
    float d = texture(uDepthTex, uv).r;
    if (uOrtho == 1) return uNear + d * (uFar - uNear);   // 正交：窗口深度线性
    return uNear * uFar / max(uFar - d * (uFar - uNear), 1e-6);
}
bool hasGeom(vec2 uv) { return texture(uNormalTex, uv).a > 0.5; }
vec3 normalAt(vec2 uv) { return normalize(texture(uNormalTex, uv).rgb * 2.0 - 1.0); }

// 4 个轴（水平/垂直/两条对角），每轴取中心两侧 ±r 的样本。
vec2 axisOffsets[4] = vec2[4](vec2(1.0, 0.0), vec2(0.0, 1.0),
                             vec2(1.0, 1.0), vec2(1.0, -1.0));

void main() {
    if (!hasGeom(vUv)) discard;                 // 只画在物体像素上
    float z0 = linDepth(vUv);
    float r = max(uWidthPx * 0.5, 1.0);
    float edge = 0.0;
    for (int i = 0; i < 4; ++i) {
        vec2 d = axisOffsets[i] * r * uTexel;
        vec2 uvA = vUv - d;
        vec2 uvB = vUv + d;
        bool vA = hasGeom(uvA);
        bool vB = hasGeom(uvB);
        if (vA != vB) { edge = 1.0; continue; }   // 剪影：一侧是背景
        if (!vA) continue;                        // 两侧都是背景

        float zA = linDepth(uvA);
        float zB = linDepth(uvB);
        // 二阶差分：斜坡面（掠射面）左右梯度对称 → 接近 0；真正的深度断裂才非 0。
        // 原来用一阶差分会把掠射的平板/地面整片当成“边”，是噪点的主因之一。
        float second = abs((zB - z0) - (z0 - zA)) / max(z0, 1e-6);
        edge = max(edge, second / max(uDepthThreshold, 1e-6));

        if (uCreases == 1) {
            // 折痕：比较两侧法线（跨过棱线时夹角大；光滑曲面逐像素变化小）。
            float dp = dot(normalAt(uvA), normalAt(uvB));
            edge = max(edge, (uNormalCos - dp) / max(1.0 - uNormalCos, 1e-4));
        }
    }
    float alpha = clamp(edge, 0.0, 1.0) * uColor.a;
    if (alpha <= 0.002) discard;
    fragColor = vec4(uColor.rgb, alpha);
}
)GLSL";

// ---------------------------------------------------------------------------
// 单位基元网格（法线朝外、绕序 CCW）。缩放由 model matrix 负责：
//   BOX / ELLIPSOID        → scale = geom_size（半轴）
//   SPHERE                 → scale = (size0, size0, size0)
//   CYLINDER               → scale = (size0, size0, size1)   单位圆柱 z∈[-1,1]
//   CAPSULE                → 按 size1/size0 生成（半径归一为 1），再均匀乘 size0
// ---------------------------------------------------------------------------

void appendVertex(std::vector<float>& v, float px, float py, float pz,
                  float nx, float ny, float nz)
{
    v.push_back(px); v.push_back(py); v.push_back(pz);
    v.push_back(nx); v.push_back(ny); v.push_back(nz);
}

void buildBox(std::vector<float>& v, std::vector<unsigned int>& idx)
{
    struct Face { float n[3]; float p[4][3]; };
    static const Face faces[6] = {
        // +X
        {{ 1, 0, 0}, {{ 1,-1,-1}, { 1, 1,-1}, { 1, 1, 1}, { 1,-1, 1}}},
        // -X
        {{-1, 0, 0}, {{-1,-1, 1}, {-1, 1, 1}, {-1, 1,-1}, {-1,-1,-1}}},
        // +Y
        {{ 0, 1, 0}, {{ 1, 1,-1}, {-1, 1,-1}, {-1, 1, 1}, { 1, 1, 1}}},
        // -Y
        {{ 0,-1, 0}, {{ 1,-1, 1}, {-1,-1, 1}, {-1,-1,-1}, { 1,-1,-1}}},
        // +Z
        {{ 0, 0, 1}, {{-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}}},
        // -Z
        {{ 0, 0,-1}, {{-1, 1,-1}, { 1, 1,-1}, { 1,-1,-1}, {-1,-1,-1}}},
    };
    for (const Face& f : faces) {
        const unsigned int base = static_cast<unsigned int>(v.size() / 6);
        for (int k = 0; k < 4; ++k)
            appendVertex(v, f.p[k][0], f.p[k][1], f.p[k][2], f.n[0], f.n[1], f.n[2]);
        idx.push_back(base + 0); idx.push_back(base + 1); idx.push_back(base + 2);
        idx.push_back(base + 0); idx.push_back(base + 2); idx.push_back(base + 3);
    }
}

// 经纬球：法线 = 位置（在单位球上）。
void buildSphere(std::vector<float>& v, std::vector<unsigned int>& idx,
                 int segU = 32, int segV = 16)
{
    for (int i = 0; i <= segV; ++i) {
        const float th = kPi * float(i) / float(segV);
        const float st = std::sin(th);
        const float ct = std::cos(th);
        for (int j = 0; j <= segU; ++j) {
            const float ph = 2.0f * kPi * float(j) / float(segU);
            const float x = st * std::cos(ph);
            const float y = st * std::sin(ph);
            appendVertex(v, x, y, ct, x, y, ct);
        }
    }
    const unsigned int stride = static_cast<unsigned int>(segU + 1);
    for (int i = 0; i < segV; ++i) {
        for (int j = 0; j < segU; ++j) {
            const unsigned int a = static_cast<unsigned int>(i) * stride + static_cast<unsigned int>(j);
            const unsigned int b = a + stride;
            idx.push_back(a);     idx.push_back(b);     idx.push_back(a + 1);
            idx.push_back(a + 1); idx.push_back(b);     idx.push_back(b + 1);
        }
    }
}

// 圆柱：半径 1、z∈[-1,1]（侧面 + 两个端盖）。
void buildCylinder(std::vector<float>& v, std::vector<unsigned int>& idx, int segU = 32)
{
    // 侧面：布置 top_j = j、bottom_j = segU+1+j。
    for (int j = 0; j <= segU; ++j) {
        const float ph = 2.0f * kPi * float(j) / float(segU);
        const float cx = std::cos(ph);
        const float cy = std::sin(ph);
        appendVertex(v, cx, cy,  1.0f, cx, cy, 0.0f);
        appendVertex(v, cx, cy, -1.0f, cx, cy, 0.0f);
    }
    const unsigned int sides = static_cast<unsigned int>(segU + 1);
    for (int j = 0; j < segU; ++j) {
        const unsigned int a = static_cast<unsigned int>(j);          // top_j
        const unsigned int b = a + sides;                             // bottom_j
        idx.push_back(a);     idx.push_back(b);     idx.push_back(a + 1);
        idx.push_back(a + 1); idx.push_back(b);     idx.push_back(b + 1);
    }
    // 顶盖（+z）：中心 + 环，CCW 从 +z 看。
    const unsigned int topCenter = static_cast<unsigned int>(v.size() / 6);
    appendVertex(v, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    const unsigned int topRing = static_cast<unsigned int>(v.size() / 6);
    for (int j = 0; j < segU; ++j) {
        const float ph = 2.0f * kPi * float(j) / float(segU);
        appendVertex(v, std::cos(ph), std::sin(ph), 1.0f, 0.0f, 0.0f, 1.0f);
    }
    for (int j = 0; j < segU; ++j) {
        const unsigned int j0 = topRing + static_cast<unsigned int>(j);
        const unsigned int j1 = topRing + static_cast<unsigned int>((j + 1) % segU);
        idx.push_back(topCenter); idx.push_back(j0); idx.push_back(j1);
    }
    // 底盖（-z）：方向反过来。
    const unsigned int botCenter = static_cast<unsigned int>(v.size() / 6);
    appendVertex(v, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f);
    const unsigned int botRing = static_cast<unsigned int>(v.size() / 6);
    for (int j = 0; j < segU; ++j) {
        const float ph = 2.0f * kPi * float(j) / float(segU);
        appendVertex(v, std::cos(ph), std::sin(ph), -1.0f, 0.0f, 0.0f, -1.0f);
    }
    for (int j = 0; j < segU; ++j) {
        const unsigned int j0 = botRing + static_cast<unsigned int>(j);
        const unsigned int j1 = botRing + static_cast<unsigned int>((j + 1) % segU);
        idx.push_back(botCenter); idx.push_back(j1); idx.push_back(j0);
    }
}

// 胶囊：半径 1，圆柱段半长 h（h=0 退化为球）。z = ±h ± 半球。
void buildCapsule(std::vector<float>& v, std::vector<unsigned int>& idx, float h,
                  int segU = 32, int segV = 16)
{
    for (int i = 0; i <= segV; ++i) {
        const float th = kPi * float(i) / float(segV);
        const float st = std::sin(th);
        const float ct = std::cos(th);
        const float z = (ct >= 0.0f) ? (h + ct) : (-h + ct);
        for (int j = 0; j <= segU; ++j) {
            const float ph = 2.0f * kPi * float(j) / float(segU);
            const float x = st * std::cos(ph);
            const float y = st * std::sin(ph);
            appendVertex(v, x, y, z, x, y, ct);
        }
    }
    const unsigned int stride = static_cast<unsigned int>(segU + 1);
    for (int i = 0; i < segV; ++i) {
        for (int j = 0; j < segU; ++j) {
            const unsigned int a = static_cast<unsigned int>(i) * stride + static_cast<unsigned int>(j);
            const unsigned int b = a + stride;
            idx.push_back(a);     idx.push_back(b);     idx.push_back(a + 1);
            idx.push_back(a + 1); idx.push_back(b);     idx.push_back(b + 1);
        }
    }
}

} // namespace

OutlineRenderer::OutlineRenderer() = default;

OutlineRenderer::~OutlineRenderer() {
    // GL 资源应已通过 releaseGL() 在渲染线程释放；此处不再触碰 GL。
}

bool OutlineRenderer::ensurePrograms() {
    if (m_programsFailed) return false;
    if (!m_normalProg) {
        auto* prog = new QOpenGLShaderProgram();
        if (prog->addShaderFromSourceCode(QOpenGLShader::Vertex, kNormalVertexShader) &&
            prog->addShaderFromSourceCode(QOpenGLShader::Fragment, kNormalFragmentShader) &&
            prog->link()) {
            m_normalProg = prog;
            m_locNormalMVP    = prog->uniformLocation("uMVP");
            m_locNormalMat    = prog->uniformLocation("uNormalMat");
        } else {
            delete prog;
            m_programsFailed = true;      // 不再每帧重试，避免刷日志
            return false;
        }
    }
    if (!m_edgeProg) {
        auto* prog = new QOpenGLShaderProgram();
        if (prog->addShaderFromSourceCode(QOpenGLShader::Vertex, kEdgeVertexShader) &&
            prog->addShaderFromSourceCode(QOpenGLShader::Fragment, kEdgeFragmentShader) &&
            prog->link()) {
            m_edgeProg = prog;
            m_locEdgeNormalTex   = prog->uniformLocation("uNormalTex");
            m_locEdgeDepthTex    = prog->uniformLocation("uDepthTex");
            m_locEdgeTexel       = prog->uniformLocation("uTexel");
            m_locEdgeWidthPx     = prog->uniformLocation("uWidthPx");
            m_locEdgeDepthThresh = prog->uniformLocation("uDepthThreshold");
            m_locEdgeNormalCos   = prog->uniformLocation("uNormalCos");
            m_locEdgeCreases     = prog->uniformLocation("uCreases");
            m_locEdgeColor       = prog->uniformLocation("uColor");
            m_locEdgeNear        = prog->uniformLocation("uNear");
            m_locEdgeFar         = prog->uniformLocation("uFar");
            m_locEdgeOrtho       = prog->uniformLocation("uOrtho");
        } else {
            delete prog;
            m_programsFailed = true;      // 不再每帧重试，避免刷日志
            return false;
        }
    }
    if (!m_dummyVao) {
        auto* gl = QOpenGLContext::currentContext()->extraFunctions();
        gl->glGenVertexArrays(1, &m_dummyVao);
    }
    return true;
}

bool OutlineRenderer::ensureTargets(int w, int h) {
    if (w <= 0 || h <= 0) return false;
    if (m_gbufFbo && m_gbufW == w && m_gbufH == h) return true;

    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    if (!ctx) return false;
    auto* gl = ctx->extraFunctions();
    auto* f  = ctx->functions();

    if (m_gbufFbo)      { gl->glDeleteFramebuffers(1, &m_gbufFbo); m_gbufFbo = 0; }
    if (m_gbufColorTex) { f->glDeleteTextures(1, &m_gbufColorTex); m_gbufColorTex = 0; }
    if (m_gbufDepthTex) { f->glDeleteTextures(1, &m_gbufDepthTex); m_gbufDepthTex = 0; }

    // 法线（RGBA16F：rgb = 0.5+0.5*n，a = 是否有几何）。
    // 用 16F 而不是 RGBA8：8 位量化会在近于平行的大平面上引入台阶，
    // 让法线在相邻像素间“跳档”，进而被折痕判据误判成细节线。
    f->glGenTextures(1, &m_gbufColorTex);
    f->glBindTexture(GL_TEXTURE_2D, m_gbufColorTex);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0,
                    GL_RGBA, GL_HALF_FLOAT, nullptr);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // 标准 GL 深度（near→0, far→1），可采样。
    f->glGenTextures(1, &m_gbufDepthTex);
    f->glBindTexture(GL_TEXTURE_2D, m_gbufDepthTex);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0,
                    GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    f->glBindTexture(GL_TEXTURE_2D, 0);

    gl->glGenFramebuffers(1, &m_gbufFbo);
    gl->glBindFramebuffer(GL_FRAMEBUFFER, m_gbufFbo);
    gl->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, m_gbufColorTex, 0);
    gl->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, m_gbufDepthTex, 0);
    const GLenum status = gl->glCheckFramebufferStatus(GL_FRAMEBUFFER);
    gl->glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gl->glDeleteFramebuffers(1, &m_gbufFbo);
        m_gbufFbo = 0;
        f->glDeleteTextures(1, &m_gbufColorTex);
        m_gbufColorTex = 0;
        f->glDeleteTextures(1, &m_gbufDepthTex);
        m_gbufDepthTex = 0;
        return false;
    }
    m_gbufW = w;
    m_gbufH = h;
    return true;
}

OutlineRenderer::MeshGpu& OutlineRenderer::ensureMeshSlot(int key) {
    auto it = m_meshes.find(key);
    if (it != m_meshes.end()) return it->second;
    return m_meshes.emplace(key, MeshGpu()).first->second;
}

bool OutlineRenderer::uploadMesh(MeshGpu& gpu, const std::vector<float>& interleaved,
                                 const std::vector<unsigned int>& indices) {
    if (interleaved.empty() || indices.empty()) return false;
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    if (!gpu.vao) gl->glGenVertexArrays(1, &gpu.vao);
    if (!gpu.vbo) gl->glGenBuffers(1, &gpu.vbo);
    if (!gpu.ibo) gl->glGenBuffers(1, &gpu.ibo);

    gl->glBindVertexArray(gpu.vao);
    gl->glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    gl->glBufferData(GL_ARRAY_BUFFER,
                     static_cast<qopengl_GLsizeiptr>(interleaved.size() * sizeof(float)),
                     interleaved.data(), GL_STATIC_DRAW);
    const GLsizei stride = static_cast<GLsizei>(6 * sizeof(float));
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                              reinterpret_cast<const void*>(3 * sizeof(float)));
    gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    gl->glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<qopengl_GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                     indices.data(), GL_STATIC_DRAW);
    gl->glBindVertexArray(0);
    gl->glBindBuffer(GL_ARRAY_BUFFER, 0);

    gpu.indexCount = static_cast<int>(indices.size());
    return true;
}

void OutlineRenderer::clearModelMeshes(const mjModel* model) {
    const int nmesh = model ? model->nmesh : 0;
    const float* vert = model ? model->mesh_vert : nullptr;
    if (model == m_cachedModel && nmesh == m_cachedNmesh && vert == m_cachedMeshVert)
        return;

    auto* gl = QOpenGLContext::currentContext()
                   ? QOpenGLContext::currentContext()->extraFunctions()
                   : nullptr;
    for (auto it = m_meshes.begin(); it != m_meshes.end(); ) {
        if (it->first >= kPrimitiveKeyBase) { ++it; continue; }
        if (gl) {
            MeshGpu& g = it->second;
            if (g.vao) gl->glDeleteVertexArrays(1, &g.vao);
            if (g.vbo) gl->glDeleteBuffers(1, &g.vbo);
            if (g.ibo) gl->glDeleteBuffers(1, &g.ibo);
        }
        it = m_meshes.erase(it);
    }
    m_cachedModel = model;
    m_cachedNmesh = nmesh;
    m_cachedMeshVert = vert;
}

OutlineRenderer::MeshGpu* OutlineRenderer::ensureModelMesh(const mjModel* model, int meshId) {
    if (!model || meshId < 0 || meshId >= model->nmesh) return nullptr;
    auto it = m_meshes.find(meshId);
    if (it != m_meshes.end())
        return it->second.indexCount > 0 ? &it->second : nullptr;

    const int vertAdr = model->mesh_vertadr[meshId];
    const int vertNum = model->mesh_vertnum[meshId];
    const int faceAdr = model->mesh_faceadr[meshId];
    const int faceNum = model->mesh_facenum[meshId];
    const int nrmAdr  = model->mesh_normaladr[meshId];
    const int nrmNum  = model->mesh_normalnum[meshId];
    if (vertNum <= 0 || faceNum <= 0 || !model->mesh_vert || !model->mesh_face)
        return nullptr;

    MeshGpu& gpu = ensureMeshSlot(meshId);
    std::vector<float> interleaved;
    std::vector<unsigned int> indices;

    // mesh_face 的索引相对本 mesh 的顶点段（mjModel 里与 mesh_vert 分段存放）。
    const bool perVertexNormals = (nrmNum == vertNum) && model->mesh_normal;
    if (perVertexNormals) {
        interleaved.resize(static_cast<size_t>(vertNum) * 6);
        for (int v = 0; v < vertNum; ++v) {
            const float* p = model->mesh_vert + 3 * (vertAdr + v);
            const float* n = model->mesh_normal + 3 * (nrmAdr + v);
            float* dst = interleaved.data() + 6 * v;
            dst[0] = p[0]; dst[1] = p[1]; dst[2] = p[2];
            dst[3] = n[0]; dst[4] = n[1]; dst[5] = n[2];
        }
        indices.resize(static_cast<size_t>(faceNum) * 3);
        for (int f = 0; f < faceNum; ++f) {
            const int* face = model->mesh_face + 3 * (faceAdr + f);
            for (int k = 0; k < 3; ++k) {
                int vi = face[k];
                if (vi < 0 || vi >= vertNum) vi = 0;
                indices[static_cast<size_t>(3 * f + k)] = static_cast<unsigned int>(vi);
            }
        }
    } else {
        // 法线与顶点不一一对应（OBJ 平滑组等）→ 展开成逐面角点。
        const size_t corners = static_cast<size_t>(faceNum) * 3;
        interleaved.resize(corners * 6);
        indices.resize(corners);
        for (int f = 0; f < faceNum; ++f) {
            for (int k = 0; k < 3; ++k) {
                const size_t corner = static_cast<size_t>(3 * f + k);
                int vi = model->mesh_face[3 * (faceAdr + f) + k];
                if (vi < 0 || vi >= vertNum) vi = 0;
                int ni = model->mesh_facenormal ? model->mesh_facenormal[3 * (faceAdr + f) + k] : vi;
                if (ni < 0 || ni >= nrmNum) ni = 0;
                const float* p = model->mesh_vert + 3 * (vertAdr + vi);
                float* dst = interleaved.data() + 6 * corner;
                dst[0] = p[0]; dst[1] = p[1]; dst[2] = p[2];
                if (model->mesh_normal && nrmNum > 0) {
                    const float* n = model->mesh_normal + 3 * (nrmAdr + ni);
                    dst[3] = n[0]; dst[4] = n[1]; dst[5] = n[2];
                } else {
                    dst[3] = 0.0f; dst[4] = 0.0f; dst[5] = 1.0f;
                }
                indices[corner] = static_cast<unsigned int>(corner);
            }
        }
    }

    if (!uploadMesh(gpu, interleaved, indices)) return nullptr;
    return &gpu;
}

OutlineRenderer::MeshGpu* OutlineRenderer::ensurePrimitive(int geomType,
                                                          float capsuleHalfLengthRatio) {
    int key = -1;
    switch (geomType) {
    case mjGEOM_BOX:
    case mjGEOM_SPHERE:
    case mjGEOM_ELLIPSOID:
    case mjGEOM_CYLINDER:
        key = kPrimitiveKeyBase + geomType * 100;
        break;
    case mjGEOM_CAPSULE: {
        const float ratio = qBound(0.0f, capsuleHalfLengthRatio, 99.0f);
        // 胶囊是按 size1/size0 生成的实际比例网格（半径归一为 1），所以缓存键
        // 带上量化后的比例；单独占一段键空间，避免与 CYLINDER(5*100) 撞车。
        key = kPrimitiveKeyBase + 10000 + static_cast<int>(ratio * 100.0f + 0.5f);
        break;
    }
    default:
        return nullptr;
    }

    auto it = m_meshes.find(key);
    if (it != m_meshes.end())
        return it->second.indexCount > 0 ? &it->second : nullptr;

    MeshGpu& gpu = ensureMeshSlot(key);
    std::vector<float> v;
    std::vector<unsigned int> idx;
    switch (geomType) {
    case mjGEOM_BOX:                     buildBox(v, idx); break;
    case mjGEOM_SPHERE:
    case mjGEOM_ELLIPSOID:               buildSphere(v, idx); break;
    case mjGEOM_CYLINDER:                buildCylinder(v, idx); break;
    case mjGEOM_CAPSULE:                 buildCapsule(v, idx, qBound(0.0f, capsuleHalfLengthRatio, 99.0f)); break;
    default:                             return nullptr;
    }
    if (!uploadMesh(gpu, v, idx)) return nullptr;
    return &gpu;
}

void OutlineRenderer::drawObject(const MeshGpu& gpu, const QMatrix4x4& modelM,
                                 const OverlayCamera& cam) {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    const QMatrix4x4 modelView = cam.view * modelM;
    m_normalProg->setUniformValue(m_locNormalMVP, cam.gbufProj * modelView);
    m_normalProg->setUniformValue(m_locNormalMat, modelView.normalMatrix());
    gl->glBindVertexArray(gpu.vao);
    gl->glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr);
}

void OutlineRenderer::buildGeometry(const OverlayCamera& cam, const mjModel* model,
                                    const std::vector<OutlineGeomSample>& samples,
                                    const std::vector<unsigned char>& visibleMask) {
    if (cam.sceneW <= 0 || cam.sceneH <= 0 || samples.empty()) return;
    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    if (!ctx) return;
    auto* gl = ctx->extraFunctions();
    if (!ensurePrograms()) return;
    if (!ensureTargets(cam.sceneW, cam.sceneH)) return;

    clearModelMeshes(model);

    // MuJoCo 的渲染把 clip control 留在 ZERO_TO_ONE（beginFrame 里设的）。
    // 我方 G-buffer 用标准 frustum 矩阵（NDC z∈[-1,1]），必须显式切回
    // NEGATIVE_ONE_TO_ONE，否则窗口深度会被当成 z_ndc 直接写入 → 深度失真。
    auto* clipControlFn = reinterpret_cast<PfnGlClipControl>(
        ctx->getProcAddress(QByteArrayLiteral("glClipControl")));
    if (clipControlFn) clipControlFn(GL_LOWER_LEFT, GL_NEGATIVE_ONE_TO_ONE);

    const GLboolean scissorWasOn = gl->glIsEnabled(GL_SCISSOR_TEST);
    GLint scissorBox[4] = {0, 0, 0, 0};
    if (scissorWasOn) gl->glGetIntegerv(GL_SCISSOR_BOX, scissorBox);
    gl->glDisable(GL_SCISSOR_TEST);

    gl->glBindFramebuffer(GL_FRAMEBUFFER, m_gbufFbo);
    gl->glViewport(0, 0, cam.sceneW, cam.sceneH);
    gl->glDisable(GL_BLEND);
    gl->glDisable(GL_CULL_FACE);
    gl->glDisable(GL_MULTISAMPLE);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_LESS);
    gl->glDepthMask(GL_TRUE);
    gl->glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    gl->glClearDepthf(1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_normalProg->bind();
    for (const OutlineGeomSample& s : samples) {
        if (s.geomId < 0 || static_cast<size_t>(s.geomId) >= visibleMask.size()) continue;
        if (!visibleMask[static_cast<size_t>(s.geomId)]) continue;

        MeshGpu* gpu = nullptr;
        float sx = 1.0f, sy = 1.0f, sz = 1.0f;
        switch (s.geomType) {
        case mjGEOM_MESH:
            gpu = ensureModelMesh(model, s.meshId);
            break;
        case mjGEOM_BOX:
            gpu = ensurePrimitive(mjGEOM_BOX, 0.0f);
            sx = s.size[0]; sy = s.size[1]; sz = s.size[2];
            break;
        case mjGEOM_SPHERE:
            gpu = ensurePrimitive(mjGEOM_SPHERE, 0.0f);
            sx = sy = sz = s.size[0];
            break;
        case mjGEOM_ELLIPSOID:
            gpu = ensurePrimitive(mjGEOM_ELLIPSOID, 0.0f);
            sx = s.size[0]; sy = s.size[1]; sz = s.size[2];
            break;
        case mjGEOM_CYLINDER:
            gpu = ensurePrimitive(mjGEOM_CYLINDER, 0.0f);
            sx = s.size[0]; sy = s.size[0]; sz = s.size[1];
            break;
        case mjGEOM_CAPSULE: {
            const float r = std::max(s.size[0], 1e-6f);
            const float ratio = s.size[1] / r;
            gpu = ensurePrimitive(mjGEOM_CAPSULE, ratio);
            sx = sy = sz = r;
            break;
        }
        default:
            continue;   // PLANE / HFIELD / SDF 等：不产生描边
        }
        if (!gpu || gpu->indexCount <= 0) continue;

        // world = T(xpos) * R(xmat) * S(size)
        QMatrix4x4 modelM;
        modelM(0, 0) = s.xmat[0]; modelM(0, 1) = s.xmat[1]; modelM(0, 2) = s.xmat[2];
        modelM(1, 0) = s.xmat[3]; modelM(1, 1) = s.xmat[4]; modelM(1, 2) = s.xmat[5];
        modelM(2, 0) = s.xmat[6]; modelM(2, 1) = s.xmat[7]; modelM(2, 2) = s.xmat[8];
        modelM.setColumn(3, QVector4D(s.xpos[0], s.xpos[1], s.xpos[2], 1.0f));
        modelM.scale(sx, sy, sz);
        drawObject(*gpu, modelM, cam);
    }
    m_normalProg->release();
    gl->glBindVertexArray(0);

    // 恢复 MuJoCo 叠加层需要的 ZERO_TO_ONE。
    if (clipControlFn) clipControlFn(GL_LOWER_LEFT, GL_ZERO_TO_ONE);

    if (scissorWasOn) {
        gl->glScissor(scissorBox[0], scissorBox[1], scissorBox[2], scissorBox[3]);
        gl->glEnable(GL_SCISSOR_TEST);
    }
}

void OutlineRenderer::composite(unsigned int targetFbo, const OverlayCamera& cam,
                                const OutlineStyle& style) {
    if (!m_edgeProg || !m_gbufColorTex || !m_gbufDepthTex) return;
    if (cam.sceneW <= 0 || cam.sceneH <= 0) return;
    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    if (!ctx) return;
    auto* gl = ctx->extraFunctions();
    auto* f  = ctx->functions();

    gl->glBindFramebuffer(GL_FRAMEBUFFER, targetFbo);
    gl->glViewport(cam.sceneX, cam.sceneY, cam.sceneW, cam.sceneH);
    gl->glDisable(GL_DEPTH_TEST);
    gl->glDepthMask(GL_FALSE);
    gl->glDisable(GL_CULL_FACE);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_edgeProg->bind();
    m_edgeProg->setUniformValue(m_locEdgeNormalTex, 0);
    m_edgeProg->setUniformValue(m_locEdgeDepthTex, 1);
    m_edgeProg->setUniformValue(m_locEdgeTexel,
                                QVector2D(1.0f / float(cam.sceneW),
                                          1.0f / float(cam.sceneH)));
    m_edgeProg->setUniformValue(m_locEdgeWidthPx, style.widthPx);
    m_edgeProg->setUniformValue(m_locEdgeDepthThresh, style.depthThreshold);
    m_edgeProg->setUniformValue(m_locEdgeNormalCos,
                                std::cos(style.creaseAngleDeg * kDeg2Rad));
    m_edgeProg->setUniformValue(m_locEdgeCreases, style.creases ? 1 : 0);
    m_edgeProg->setUniformValue(m_locEdgeColor, style.color);
    m_edgeProg->setUniformValue(m_locEdgeNear, cam.nearZ);
    m_edgeProg->setUniformValue(m_locEdgeFar, cam.farZ);
    m_edgeProg->setUniformValue(m_locEdgeOrtho, cam.orthographic ? 1 : 0);

    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, m_gbufColorTex);
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, m_gbufDepthTex);
    gl->glBindVertexArray(m_dummyVao);
    gl->glDrawArrays(GL_TRIANGLES, 0, 3);
    gl->glBindVertexArray(0);
    m_edgeProg->release();
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, 0);
    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, 0);

    // 恢复后续叠加层（接触点标记 / gizmo）依赖的状态与整幅视口。
    gl->glViewport(0, 0, cam.fullW, cam.fullH);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_GEQUAL);
    gl->glDepthMask(GL_TRUE);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void OutlineRenderer::releaseGL() {
    if (QOpenGLContext::currentContext()) {
        auto* gl = QOpenGLContext::currentContext()->extraFunctions();
        auto* f  = QOpenGLContext::currentContext()->functions();
        for (auto& kv : m_meshes) {
            MeshGpu& g = kv.second;
            if (g.vao) gl->glDeleteVertexArrays(1, &g.vao);
            if (g.vbo) gl->glDeleteBuffers(1, &g.vbo);
            if (g.ibo) gl->glDeleteBuffers(1, &g.ibo);
        }
        if (m_gbufFbo)      { gl->glDeleteFramebuffers(1, &m_gbufFbo); m_gbufFbo = 0; }
        if (m_gbufColorTex) { f->glDeleteTextures(1, &m_gbufColorTex); m_gbufColorTex = 0; }
        if (m_gbufDepthTex) { f->glDeleteTextures(1, &m_gbufDepthTex); m_gbufDepthTex = 0; }
        if (m_dummyVao)     { gl->glDeleteVertexArrays(1, &m_dummyVao); m_dummyVao = 0; }
    }
    m_meshes.clear();
    m_gbufW = m_gbufH = 0;
    m_cachedModel = nullptr;
    m_cachedNmesh = -1;
    m_cachedMeshVert = nullptr;
    delete m_normalProg;
    m_normalProg = nullptr;
    delete m_edgeProg;
    m_edgeProg = nullptr;
    m_programsFailed = false;
}
