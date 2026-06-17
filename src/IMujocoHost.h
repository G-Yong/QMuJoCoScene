#pragma once

#include <QString>

namespace mjqt {

class IMujocoHost {
public:
    virtual ~IMujocoHost() = default;
    virtual void onFrameRendered() = 0;
    virtual void onSetTitle(const QString& title) = 0;
    virtual void onToggleFullscreen() = 0;

    // 渲染线程回调：在 mjr_render 已把场景写入离屏 FBO、但尚未把它 blit 到
    // 共享纹理之前触发。宿主可借此把自定义内容（如 GL_POINTS 点云叠加层）
    // 画进同一个 FBO（共享其深度缓冲，从而与场景正确互遮挡）。
    // targetFbo：离屏帧缓冲句柄；viewWidth/viewHeight：其像素尺寸。
    // GL context 已在当前（渲染）线程 current。默认空实现。
    virtual void onRenderOverlay(unsigned int /*targetFbo*/,
                                 int /*viewWidth*/, int /*viewHeight*/) {}
};

} // namespace mjqt