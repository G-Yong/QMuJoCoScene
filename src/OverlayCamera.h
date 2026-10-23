#pragma once
// ---------------------------------------------------------------------------
// OverlayCamera —— 叠加层共用的相机 / 投影构造。
//
// 所有"画在 mjr_render 结果之上"的代码（点云、gizmo 命中测试、描边 G-buffer）
// 都必须与 MuJoCo render_gl3.c::setView() 逐像素一致，否则会重演"叠加层相对
// 场景整体偏移"的事故（UI 侧栏 inset 那次）。这里把那段纯计算抽成共享函数，
// 由 PointCloudRenderer::beginFrame() 与 OutlineRenderer 共同调用。
//
// 两种投影的区别：
//   proj     —— MuJoCo 约定：reverse-Z（可选 ARB_clip_control 的 ZERO_TO_ONE）
//               + "场景视口 → 整幅视口"的裁剪空间变换 S（UI 侧栏 inset）。
//               点云 / 线段 / 标记都用它，且 glViewport 用整幅。
//   gbufProj —— 标准 GL 深度（near→0, far→1，clearDepth 1.0 + GL_LESS）、
//               不带 S。描边的 G-buffer FBO 尺寸就是场景矩形，合成时把
//               glViewport 设成场景矩形，因此两者 1:1 对应。
// ---------------------------------------------------------------------------

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

struct OverlayCamera {
    QMatrix4x4 view;            // lookAt + scn transform
    QMatrix4x4 proj;            // MuJoCo 约定（reverse-Z + inset S）
    QMatrix4x4 viewProj;        // proj * view
    QMatrix4x4 invViewProj;     // (proj * view)^-1
    QMatrix4x4 gbufProj;        // 标准 GL 深度、无 inset（描边 G-buffer 用）
    QMatrix4x4 gbufViewProj;    // gbufProj * view
    QVector3D  camPos;
    float      projYY = 1.0f;   // proj[1][1]，世界尺寸 → 像素换算
    bool       clipZeroToOne = false;
    int        fullW = 0, fullH = 0;                 // 整幅 FBO（设备像素）
    int        sceneX = 0, sceneY = 0;               // 3D 场景视口（设备像素）
    int        sceneW = 0, sceneH = 0;
    float      nearZ = 0.01f, farZ = 100.0f;
    bool       orthographic = false;
};

// 与 render_gl3.c::setView() 一致地求投影半宽：相机自带 frustum_width 非 0 时
// 用它，否则按"场景视口"宽高比展开（MuJoCo 自己渲染时忽略 frustum_width，
// 该分支只为兼容历史行为；正常情况 frustum_width == 0）。
inline float overlayProjectionHalfWidth(float frustumWidth, float frustumBottom,
                                        float frustumTop, int sceneW, int sceneH)
{
    if (frustumWidth != 0.0f) return frustumWidth;
    const float span = frustumTop - frustumBottom;
    if (sceneH > 0) return 0.5f * span * (float(sceneW) / float(sceneH));
    return 0.5f * span;
}

// 把"场景视口"(sceneX/Y/W/H) 映射到"整幅视口"(fullW/H) 的裁剪空间变换 S：
// s' = S * s，使 s 在整幅 glViewport 下渲染出与 inset 视口完全相同的位置。
// scene 等于整幅时返回单位阵。
inline QMatrix4x4 overlayViewportInsetMatrix(float sceneX, float sceneY,
                                             float sceneW, float sceneH,
                                             float fullW, float fullH)
{
    QMatrix4x4 m;   // 单位阵 = 无 inset
    if (fullW <= 0.0f || fullH <= 0.0f || sceneW <= 0.0f || sceneH <= 0.0f)
        return m;
    // 期望像素 px = sceneX + (s+1)/2*sceneW，整幅下 px = (s'+1)/2*fullW
    //   ⇒ s' = (sceneW/fullW)*s + (2*sceneX + sceneW)/fullW - 1
    const float ax = sceneW / fullW;
    const float ay = sceneH / fullH;
    const float bx = (2.0f * sceneX + sceneW) / fullW - 1.0f;
    const float by = (2.0f * sceneY + sceneH) / fullH - 1.0f;
    m.setToIdentity();
    m(0, 0) = ax;  m(0, 3) = bx;
    m(1, 1) = ay;  m(1, 3) = by;
    return m;
}

// 构造一帧的相机。参数与 PointCloudRenderer::beginFrame() 完全一致。
// haveClipControl：当前 GL 上下文是否支持 ARB_clip_control（由调用方用
// PointCloudRenderer::clipControlSupported() 查询后传入；这里不碰 GL 状态）。
inline OverlayCamera buildOverlayCamera(int fullW, int fullH,
                                        int sceneX, int sceneY, int sceneW, int sceneH,
                                        const float camPos[3], const float camForward[3],
                                        const float camUp[3],
                                        float frustumCenter, float frustumWidth,
                                        float frustumBottom, float frustumTop,
                                        float frustumNear, float frustumFar,
                                        bool orthographic,
                                        bool sceneTransform, const float translate[3],
                                        const float rotateQuat[4], float scale,
                                        bool haveClipControl)
{
    OverlayCamera cam;
    cam.fullW = fullW;
    cam.fullH = fullH;
    cam.sceneX = sceneX;
    cam.sceneY = sceneY;
    cam.sceneW = sceneW;
    cam.sceneH = sceneH;
    cam.nearZ = frustumNear;
    cam.farZ = frustumFar;
    cam.orthographic = orthographic;
    cam.camPos = QVector3D(camPos[0], camPos[1], camPos[2]);

    // 视图矩阵：lookAt(pos, pos+forward, up)，再叠加 scene transform。
    const QVector3D pos = cam.camPos;
    const QVector3D fwd(camForward[0], camForward[1], camForward[2]);
    const QVector3D up(camUp[0], camUp[1], camUp[2]);
    cam.view.setToIdentity();
    cam.view.lookAt(pos, pos + fwd, up);
    if (sceneTransform) {
        cam.view.translate(translate[0], translate[1], translate[2]);
        cam.view.rotate(QQuaternion(rotateQuat[0], rotateQuat[1],
                                    rotateQuat[2], rotateQuat[3]));
        cam.view.scale(scale);
    }

    // 投影：halfwidth 匹配"场景视口"的宽高比（与 mjr_render 的 rect[3] 一致）。
    const float halfwidth = overlayProjectionHalfWidth(
        frustumWidth, frustumBottom, frustumTop, sceneW, sceneH);
    const float left  = frustumCenter - halfwidth;
    const float right = frustumCenter + halfwidth;

    QMatrix4x4 frustum;
    if (orthographic)
        frustum.ortho(left, right, frustumBottom, frustumTop, frustumNear, frustumFar);
    else
        frustum.frustum(left, right, frustumBottom, frustumTop, frustumNear, frustumFar);

    // MuJoCo 使用 reverse-Z。是否启用 ARB_clip_control 决定 Z 映射方式。
    QMatrix4x4 zflip;
    if (haveClipControl) {
        zflip.translate(0.0f, 0.0f, 0.5f);
        zflip.scale(1.0f, 1.0f, -0.5f);
        cam.clipZeroToOne = true;
    } else {
        zflip.scale(1.0f, 1.0f, -1.0f);
        cam.clipZeroToOne = false;
    }
    cam.proj = zflip * frustum;
    // 场景在 FBO 里 inset（MuJoCo UI 侧栏）时，把"场景视口 → 整幅"的裁剪空间
    // 变换叠加到投影上：glViewport 仍用整幅，但内容只落在 rect[3] 内。
    cam.proj = overlayViewportInsetMatrix(float(sceneX), float(sceneY),
                                          float(sceneW), float(sceneH),
                                          float(fullW), float(fullH)) * cam.proj;
    cam.viewProj = cam.proj * cam.view;
    cam.invViewProj = cam.viewProj.inverted();
    cam.projYY = cam.proj(1, 1);

    // 描边 G-buffer：标准 GL 深度、无 inset（视口就是场景矩形本身）。
    cam.gbufProj = frustum;
    cam.gbufViewProj = cam.gbufProj * cam.view;
    return cam;
}
