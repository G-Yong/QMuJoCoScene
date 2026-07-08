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
// 兼容性 profile 下，gl_PointCoord 仅在启用点精灵后才会被填充；
// 否则它恒为 (0,0)，圆/球样式着色器会把所有片元 discard（看不到任何东西）。
#ifndef GL_POINT_SPRITE
#  define GL_POINT_SPRITE 0x8861
#endif

namespace {

// glClipControl 不在 QOpenGLExtraFunctions 中（非 GLES），需自行解析。
using PfnGlClipControl = void (QOPENGLF_APIENTRYP)(GLenum origin, GLenum depth);

const char* kVertexShader = R"GLSL(
#version 330
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4  uMVP;            // proj * view（不含模型变换）
uniform mat4  uMV;            // view
uniform float uAlphaScale;     // 透明度缩放（倒影时 <1 用于淡化，否则 1）
uniform int   uStyle;          // 0 pixel,1 square,2 circle,3 sphere
uniform int   uOrtho;          // 1 正交
uniform float uPointSize;      // pixel: 像素；其余: 世界半径(米)
uniform float uViewportH;
uniform float uProjYY;         // 透视: 2n/(t-b)；正交: 2/(t-b)
uniform bool  uUseUniformColor;
uniform vec4  uUniformColor;
// --- 倒影（屏幕空间平面反射）---
uniform int   uReflect;        // 1 = 渲染地面倒影
uniform vec3  uCamPos;         // 相机世界坐标（倒影投影用）
uniform float uPlaneZ;         // 反射平面高度 z
out vec4 vColor;
out float vCull;               // >0 表示该点应被裁剪（倒影无效）
void main() {
    vColor = uUseUniformColor ? uUniformColor : aColor;
    vColor.a *= uAlphaScale;
    vCull = 0.0;

    vec3 worldPos;      // 屏幕位置 + 深度用（倒影时是地面交点 F）
    vec3 sizeRefPos;    // 点大小的透视距离参考
    if (uReflect == 1) {
        // 经典平面反射：点 P 在地面下的镜像 P'，相机看向 P' 的视线与地面
        // z=uPlaneZ 的交点 F 就是该反射在地面上出现的位置。把带颜色的点画在
        // F 处（地面深度），即可被前景物体的深度正确遮挡、且贴合不透明地面。
        vec3 mirrored = vec3(aPos.xy, 2.0 * uPlaneZ - aPos.z);
        vec3 dir = mirrored - uCamPos;
        // 相机须在平面上方且视线确实向下穿过平面，否则倒影无效。
        if (uCamPos.z <= uPlaneZ || dir.z >= -1e-6) {
            vCull = 1.0;
            worldPos = mirrored;            // 占位，后面会被裁剪
        } else {
            float t = (uPlaneZ - uCamPos.z) / dir.z;
            worldPos = uCamPos + t * dir;   // 地面交点 F
            // 轻微抬高，避免与地面 z-fighting（深度写关闭，仅做测试）。
            worldPos.z += 1e-3;
        }
        // 关键：倒影点的“视觉大小”必须按镜像点 P' 的真实距离算。镜面反射的
        // 视觉距离 = 相机→地面交点 F→原点 的路径长 = 相机→P'。若错用 F 的距离，
        // 掠射角下 F 会趋近相机 → dist→0 → 点被放大到爆炸，造成巨量 overdraw 卡顿。
        sizeRefPos = mirrored;
    } else {
        worldPos = aPos;
        sizeRefPos = aPos;
    }

    gl_Position  = uMVP * vec4(worldPos, 1.0);
    if (uStyle == 0) {
        gl_PointSize = uPointSize;                       // 固定像素
    } else {
        float px;
        if (uOrtho == 1) {
            px = uPointSize * uViewportH * uProjYY;      // 正交: 与距离无关
        } else {
            vec4 sizeViewPos = uMV * vec4(sizeRefPos, 1.0);
            float dist = max(-sizeViewPos.z, 1e-4);
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
in float vCull;
out vec4 fragColor;
void main() {
    if (vCull > 0.5) discard;                            // 无效倒影点
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
    m_locAlphaScale      = prog->uniformLocation("uAlphaScale");
    m_locReflect         = prog->uniformLocation("uReflect");
    m_locCamPos          = prog->uniformLocation("uCamPos");
    m_locPlaneZ          = prog->uniformLocation("uPlaneZ");
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
    m_camPos = QVector3D(camPos[0], camPos[1], camPos[2]);

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
    // 兼容 profile 下必须显式打开点精灵，gl_PointCoord 才有效（圆/球样式依赖它）。
    gl->glEnable(GL_POINT_SPRITE);
    gl->glEnable(GL_MULTISAMPLE);
    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_program->bind();
    m_program->setUniformValue(m_locMVP, m_proj * m_view);
    m_program->setUniformValue(m_locMV, m_view);
    m_program->setUniformValue(m_locViewportH, static_cast<float>(viewportH));
    m_program->setUniformValue(m_locProjYY, m_projYY);
    m_program->setUniformValue("uOrtho", orthographic ? 1 : 0);
    // 默认不淡化、不做倒影；倒影绘制时由 drawCloudReflected 临时覆盖。
    m_program->setUniformValue(m_locAlphaScale, 1.0f);
    m_program->setUniformValue(m_locReflect, 0);
    m_program->setUniformValue(m_locCamPos, m_camPos);
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

void PointCloudRenderer::drawCloudReflected(int cloudId, int style, float pointSize,
                                            const QVector4D& uniformColor,
                                            float planeZ, float alphaScale) {
    if (!m_program) return;

    // 相机在反射平面下方或恰在平面上时，所有点的镜像都产生向上/水平的射线，
    // 不会穿过平面，顶点着色器会对全部点设 vCull=1.0 → 片元着色器全部 discard。
    // 对百万级点云这等于每帧白跑 vertex shader + 光栅化 + fragment shader，
    // 造成明显的越过地面时的卡顿。直接在 CPU 侧跳过整个 draw call。
    if (m_camPos.z() <= planeZ) return;

    auto it = m_clouds.find(cloudId);
    if (it == m_clouds.end() || it->second.count <= 0) return;
    const GpuCloud& c = it->second;

    auto* gl = QOpenGLContext::currentContext()->extraFunctions();

    // 屏幕空间平面反射：顶点着色器把每个点的镜像沿视线投影到地面 z=planeZ 上，
    // 颜色画在地面交点处、深度等于该处地面深度。
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_GEQUAL);   // reverse-Z
    gl->glDepthMask(GL_FALSE);    // 倒影不写深度（非真实几何）

    // ---------------------------------------------------------------
    // 透明度累积问题：密集点云（35000+点）中多个点投影到同一地面像素时，
    // 标准 GL_SRC_ALPHA 混合会反复叠加，导致该像素趋向不透明（0.3^N → 1）。
    // 解决方法：用模板缓冲令每个像素最多接受一次反射绘制。
    //
    // mjr_render 结束后模板缓冲已不再被 MuJoCo 使用，可安全清空并借用。
    // 清空为 0 → 仅在模板≠1（即初始 0）处绘制 → 绘制后 REPLACE 写入 ref=1
    // （后续点在同像素看到模板=1 被挡掉）。结果：每个地面像素恰好混入一个反射点，
    // 透明度严格等于 alphaScale，避免多点叠加导致的发白。
    //
    // 注意：GL_REPLACE 写入的是 glStencilFunc 的 ref 值，故必须让“通过条件”用的
    // ref 恰好等于要写入的值。用 GL_EQUAL+ref=0 会把 0 写回模板（等于没写），
    // 去重失效；改用 GL_NOTEQUAL+ref=1：初始 0≠1 通过、写 1，之后 1≠1 失败被挡。
    // ---------------------------------------------------------------
    gl->glClear(GL_STENCIL_BUFFER_BIT);
    gl->glEnable(GL_STENCIL_TEST);
    gl->glStencilFunc(GL_NOTEQUAL, 1, 0xFF);       // 仅在模板≠1（初始 0）处通过
    gl->glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE); // 通过后写入 ref=1，禁止重复绘制
    gl->glStencilMask(0xFF);

    m_program->setUniformValue(m_locReflect, 1);
    m_program->setUniformValue(m_locPlaneZ, planeZ);
    m_program->setUniformValue(m_locCamPos, m_camPos);
    m_program->setUniformValue(m_locAlphaScale, alphaScale);
    m_program->setUniformValue(m_locStyle, style);
    m_program->setUniformValue(m_locPointSize, pointSize > 0.0f ? pointSize : 1.0f);
    m_program->setUniformValue(m_locUseUniformColor, !c.hasColors);
    m_program->setUniformValue(m_locUniformColor, uniformColor);

    gl->glBindVertexArray(c.vao);
    gl->glDrawArrays(GL_POINTS, 0, c.count);
    gl->glBindVertexArray(0);

    // 还原：关闭倒影模式与模板测试、恢复深度写入与不淡化，供后续实点云正常绘制。
    m_program->setUniformValue(m_locReflect, 0);
    m_program->setUniformValue(m_locAlphaScale, 1.0f);
    gl->glDisable(GL_STENCIL_TEST);
    gl->glDepthMask(GL_TRUE);
}

void PointCloudRenderer::endFrame() {
    if (!m_program) return;
    auto* gl = QOpenGLContext::currentContext()->extraFunctions();
    m_program->release();
    gl->glDisable(GL_PROGRAM_POINT_SIZE);
    gl->glDisable(GL_POINT_SPRITE);
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
