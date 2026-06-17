#include "PointCloudRenderer.h"

#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QQuaternion>
#include <QVector3D>

#include <algorithm>

// 桌面 GL 常量（Qt 的 GLES 头里可能没有定义）。
#ifndef GL_PROGRAM_POINT_SIZE
#  define GL_PROGRAM_POINT_SIZE 0x8642
#endif
#ifndef GL_MULTISAMPLE
#  define GL_MULTISAMPLE 0x809D
#endif
#ifndef GL_LOWER_LEFT
#  define GL_LOWER_LEFT 0x8CA1
#endif
#ifndef GL_ZERO_TO_ONE
#  define GL_ZERO_TO_ONE 0x935F
#endif
#ifndef GL_POINTS
#  define GL_POINTS 0x0000
#endif

namespace {

// glClipControl 不在 QOpenGLExtraFunctions 中（非 GLES），需自行解析。
using PfnGlClipControl = void (QOPENGLF_APIENTRYP)(GLenum origin, GLenum depth);

const char* kVertexShader = R"GLSL(
#version 330
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4  uMVP;
uniform mat4  uMV;
uniform int   uStyle;          // 0 pixel,1 square,2 circle,3 sphere
uniform int   uOrtho;          // 1 正交
uniform float uPointSize;      // pixel: 像素；其余: 世界半径(米)
uniform float uViewportH;
uniform float uProjYY;         // 透视: 2n/(t-b)；正交: 2/(t-b)
uniform bool  uUseUniformColor;
uniform vec4  uUniformColor;
out vec4 vColor;
void main() {
    vColor = uUseUniformColor ? uUniformColor : aColor;
    vec4 viewPos = uMV * vec4(aPos, 1.0);
    gl_Position  = uMVP * vec4(aPos, 1.0);
    if (uStyle == 0) {
        gl_PointSize = uPointSize;                       // 固定像素
    } else {
        float px;
        if (uOrtho == 1) {
            px = uPointSize * uViewportH * uProjYY;      // 正交: 与距离无关
        } else {
            float dist = max(-viewPos.z, 1e-4);
            px = uPointSize * uViewportH * uProjYY / dist;
        }
        gl_PointSize = clamp(px, 1.0, 4096.0);
    }
}
)GLSL";

const char* kFragmentShader = R"GLSL(
#version 330
uniform int uStyle;
in vec4 vColor;
out vec4 fragColor;
void main() {
    if (uStyle == 0 || uStyle == 1) {
        fragColor = vColor;                              // pixel / square: 整片
    } else {
        vec2 d = gl_PointCoord - vec2(0.5);
        float r2 = dot(d, d);
        if (r2 > 0.25) discard;                          // 圆外丢弃
        if (uStyle == 3) {                               // sphere: 球面着色
            float z = sqrt(max(0.0, 0.25 - r2)) * 2.0;
            vec3 n = normalize(vec3(d * 2.0, z));
            vec3 L = normalize(vec3(0.35, 0.35, 1.0));
            float diff = max(dot(n, L), 0.0) * 0.8 + 0.2;
            fragColor = vec4(vColor.rgb * diff, vColor.a);
        } else {                                         // circle: 纯色圆片
            fragColor = vColor;
        }
    }
}
)GLSL";

} // namespace

PointCloudRenderer::PointCloudRenderer() = default;

PointCloudRenderer::~PointCloudRenderer() {
    // GL 资源应已通过 releaseGL() 在 GL 线程释放；此处不再触碰 GL。
}

bool PointCloudRenderer::ensureProgram() {
    if (m_program) return true;
    auto* prog = new QOpenGLShaderProgram();
    if (!prog->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) ||
        !prog->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) ||
        !prog->link()) {
        delete prog;
        return false;
    }
    m_program = prog;
    m_locMVP             = prog->uniformLocation("uMVP");
    m_locMV              = prog->uniformLocation("uMV");
    m_locStyle           = prog->uniformLocation("uStyle");
    m_locPointSize       = prog->uniformLocation("uPointSize");
    m_locViewportH       = prog->uniformLocation("uViewportH");
    m_locProjYY          = prog->uniformLocation("uProjYY");
    m_locUseUniformColor = prog->uniformLocation("uUseUniformColor");
    m_locUniformColor    = prog->uniformLocation("uUniformColor");
    return true;
}

PointCloudRenderer::GpuCloud& PointCloudRenderer::ensureCloud(int cloudId) {
    auto it = m_clouds.find(cloudId);
    if (it != m_clouds.end()) return it->second;

    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    GpuCloud c;
    gl->glGenVertexArrays(1, &c.vao);
    gl->glGenBuffers(1, &c.posVbo);
    gl->glGenBuffers(1, &c.colVbo);
    return m_clouds.emplace(cloudId, c).first->second;
}

void PointCloudRenderer::uploadPositions(int cloudId, const float* xyz, int count) {
    if (count < 0) count = 0;
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    GpuCloud& c = ensureCloud(cloudId);

    gl->glBindVertexArray(c.vao);
    gl->glBindBuffer(GL_ARRAY_BUFFER, c.posVbo);
    gl->glBufferData(GL_ARRAY_BUFFER,
                     static_cast<qopengl_GLsizeiptr>(sizeof(float) * 3 * count),
                     count > 0 ? xyz : nullptr, GL_DYNAMIC_DRAW);
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gl->glBindVertexArray(0);
    gl->glBindBuffer(GL_ARRAY_BUFFER, 0);
    c.count = count;
}

void PointCloudRenderer::uploadColors(int cloudId, const float* rgba, int count) {
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    GpuCloud& c = ensureCloud(cloudId);

    gl->glBindVertexArray(c.vao);
    if (count > 0 && rgba) {
        gl->glBindBuffer(GL_ARRAY_BUFFER, c.colVbo);
        gl->glBufferData(GL_ARRAY_BUFFER,
                         static_cast<qopengl_GLsizeiptr>(sizeof(float) * 4 * count),
                         rgba, GL_DYNAMIC_DRAW);
        gl->glEnableVertexAttribArray(1);
        gl->glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        c.hasColors = true;
    } else {
        gl->glDisableVertexAttribArray(1);
        c.hasColors = false;
    }
    gl->glBindVertexArray(0);
    gl->glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloudRenderer::removeCloud(int cloudId) {
    auto it = m_clouds.find(cloudId);
    if (it == m_clouds.end()) return;
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    gl->glDeleteVertexArrays(1, &it->second.vao);
    gl->glDeleteBuffers(1, &it->second.posVbo);
    gl->glDeleteBuffers(1, &it->second.colVbo);
    m_clouds.erase(it);
}

void PointCloudRenderer::retainOnly(const std::vector<int>& keepIds) {
    for (auto it = m_clouds.begin(); it != m_clouds.end(); ) {
        const bool keep = std::find(keepIds.begin(), keepIds.end(), it->first) != keepIds.end();
        if (keep) {
            ++it;
        } else {
            auto* gl = QOpenGLContext::currentContext()->extraFunctions();
            gl->glDeleteVertexArrays(1, &it->second.vao);
            gl->glDeleteBuffers(1, &it->second.posVbo);
            gl->glDeleteBuffers(1, &it->second.colVbo);
            it = m_clouds.erase(it);
        }
    }
}

void PointCloudRenderer::beginFrame(unsigned int targetFbo, int viewportW, int viewportH,
                                    const float camPos[3], const float camForward[3],
                                    const float camUp[3],
                                    float frustumCenter, float frustumWidth,
                                    float frustumBottom, float frustumTop,
                                    float frustumNear, float frustumFar,
                                    bool orthographic,
                                    bool sceneTransform, const float translate[3],
                                    const float rotateQuat[4], float scale) {
    if (!ensureProgram()) return;

    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    auto* gl = ctx->extraFunctions();

    m_viewportH = viewportH;

    // --- 复制 MuJoCo render_gl3.c setView() 的投影/视图/深度约定 ---------
    // 视图矩阵：lookAt(pos, pos+forward, up)，再叠加 scene transform。
    const QVector3D pos(camPos[0], camPos[1], camPos[2]);
    const QVector3D fwd(camForward[0], camForward[1], camForward[2]);
    const QVector3D up(camUp[0], camUp[1], camUp[2]);
    m_view.setToIdentity();
    m_view.lookAt(pos, pos + fwd, up);
    if (sceneTransform) {
        m_view.translate(translate[0], translate[1], translate[2]);
        m_view.rotate(QQuaternion(rotateQuat[0], rotateQuat[1],
                                  rotateQuat[2], rotateQuat[3]));
        m_view.scale(scale);
    }

    // 投影：halfwidth 匹配视口宽高比（与 MuJoCo 完全一致）。
    const float halfwidth = frustumWidth != 0.0f
        ? frustumWidth
        : 0.5f * static_cast<float>(viewportW) / static_cast<float>(viewportH) *
              (frustumTop - frustumBottom);
    const float left  = frustumCenter - halfwidth;
    const float right = frustumCenter + halfwidth;

    QMatrix4x4 frustum;
    if (orthographic)
        frustum.ortho(left, right, frustumBottom, frustumTop, frustumNear, frustumFar);
    else
        frustum.frustum(left, right, frustumBottom, frustumTop, frustumNear, frustumFar);

    // MuJoCo 使用 reverse-Z。是否启用 ARB_clip_control 决定 Z 映射方式。
    const bool clipControl =
        ctx->format().version() >= qMakePair(4, 5) ||
        ctx->hasExtension(QByteArrayLiteral("GL_ARB_clip_control"));
    auto clipControlFn = reinterpret_cast<PfnGlClipControl>(
        ctx->getProcAddress(QByteArrayLiteral("glClipControl")));

    QMatrix4x4 zflip;
    if (clipControl && clipControlFn) {
        clipControlFn(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
        zflip.translate(0.0f, 0.0f, 0.5f);
        zflip.scale(1.0f, 1.0f, -0.5f);
    } else {
        zflip.scale(1.0f, 1.0f, -1.0f);
    }
    m_proj = zflip * frustum;

    // 世界尺寸 → 像素换算系数。
    m_projYY = orthographic
        ? 2.0f / (frustumTop - frustumBottom)
        : 2.0f * frustumNear / (frustumTop - frustumBottom);

    // --- GL 状态 -------------------------------------------------------
    gl->glBindFramebuffer(GL_FRAMEBUFFER, targetFbo);
    gl->glViewport(0, 0, viewportW, viewportH);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_GEQUAL);          // reverse-Z：与 MuJoCo 一致
    gl->glDepthMask(GL_TRUE);
    gl->glDepthRangef(0.0f, 1.0f);
    gl->glEnable(GL_PROGRAM_POINT_SIZE);
    gl->glEnable(GL_MULTISAMPLE);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_program->bind();
    m_program->setUniformValue(m_locMVP, m_proj * m_view);
    m_program->setUniformValue(m_locMV, m_view);
    m_program->setUniformValue(m_locViewportH, static_cast<float>(viewportH));
    m_program->setUniformValue(m_locProjYY, m_projYY);
    m_program->setUniformValue("uOrtho", orthographic ? 1 : 0);
}

void PointCloudRenderer::drawCloud(int cloudId, int style, float pointSize,
                                   const QVector4D& uniformColor) {
    if (!m_program) return;
    auto it = m_clouds.find(cloudId);
    if (it == m_clouds.end() || it->second.count <= 0) return;
    const GpuCloud& c = it->second;

    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    m_program->setUniformValue(m_locStyle, style);
    m_program->setUniformValue(m_locPointSize, pointSize > 0.0f ? pointSize : 1.0f);
    m_program->setUniformValue(m_locUseUniformColor, !c.hasColors);
    m_program->setUniformValue(m_locUniformColor, uniformColor);

    gl->glBindVertexArray(c.vao);
    gl->glDrawArrays(GL_POINTS, 0, c.count);
    gl->glBindVertexArray(0);
}

void PointCloudRenderer::endFrame() {
    if (!m_program) return;
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    m_program->release();
    gl->glDisable(GL_PROGRAM_POINT_SIZE);
    // 深度/混合等状态由下一帧 mjr_render 的 initGL3 重置，无需在此恢复。
}

void PointCloudRenderer::releaseGL() {
    if (QOpenGLContext::currentContext()) {
        auto* gl = QOpenGLContext::currentContext()->extraFunctions();
        for (auto& kv : m_clouds) {
            gl->glDeleteVertexArrays(1, &kv.second.vao);
            gl->glDeleteBuffers(1, &kv.second.posVbo);
            gl->glDeleteBuffers(1, &kv.second.colVbo);
        }
    }
    m_clouds.clear();
    delete m_program;
    m_program = nullptr;
}
