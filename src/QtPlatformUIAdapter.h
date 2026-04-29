#pragma once
// ---------------------------------------------------------------------------
// QtPlatformUIAdapter
//
// 实现 mujoco::PlatformUIAdapter，给官方 mujoco::Simulate 提供 Qt 后端。
//
// 线程模型：
//   - Qt 主线程  : 接收 QWindow 事件，调用 Post*() 把事件入队
//   - 渲染线程   : 调用 PollEvents()→出队回放，再调 OnXxx 触发基类逻辑
//
// 与 GlfwAdapter 不同点：
//   - GL 上下文 / 缓冲区交换 / 窗口尺寸 都委托给 QtSimulateWindow
//   - "ShouldCloseWindow" 由 QtSimulateWindow 在用户关闭时置位
// ---------------------------------------------------------------------------

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <utility>

#include "platform_ui_adapter.h"

class QtSimulateWindow;

namespace mjqt {

class QtPlatformUIAdapter : public mujoco::PlatformUIAdapter {
public:
    explicit QtPlatformUIAdapter(QtSimulateWindow *win);
    ~QtPlatformUIAdapter() override;

    // ---- PlatformUIAdapter ----
    std::pair<double, double> GetCursorPosition() const override;
    double GetDisplayPixelsPerInch() const override;
    std::pair<int, int> GetFramebufferSize() const override;
    std::pair<int, int> GetWindowSize() const override;
    bool IsGPUAccelerated() const override;
    void PollEvents() override;
    void SetClipboardString(const char *text) override;
    void SetVSync(bool enabled) override;
    void SetWindowTitle(const char *title) override;
    bool ShouldCloseWindow() const override;
    void SwapBuffers() override;
    void ToggleFullscreen() override;

    bool IsLeftMouseButtonPressed()   const override { return m_btnLeft.load(); }
    bool IsMiddleMouseButtonPressed() const override { return m_btnMiddle.load(); }
    bool IsRightMouseButtonPressed()  const override { return m_btnRight.load(); }

    bool IsAltKeyPressed()   const override { return m_modAlt.load(); }
    bool IsCtrlKeyPressed()  const override { return m_modCtrl.load(); }
    bool IsShiftKeyPressed() const override { return m_modShift.load(); }

    bool IsMouseButtonDownEvent(int act) const override { return act == 1; }
    bool IsKeyDownEvent(int act)         const override { return act == 1; }

    int        TranslateKeyCode(int key)        const override { return key; }
    mjtButton  TranslateMouseButton(int btn)    const override;

    // ---- Qt 主线程 → 渲染线程 ----（线程安全）
    // act: 1 = press/down, 0 = release/up
    void PostMouseButton(int qtButton, int act, double x, double y);
    void PostMouseMove(double x, double y);
    void PostScroll(double dx, double dy);
    void PostKey(int mjKey, int act);
    void PostResize(int w, int h);
    void PostClose();

    // 修饰键状态（Qt 主线程在事件前先更新）
    void SetModifiers(bool ctrl, bool shift, bool alt);

    // 窗口尺寸 / 像素比（Qt 主线程在 resize 时更新）
    void SetWindowGeometry(int win_w, int win_h, int fb_w, int fb_h, double dpi);

private:
    QtSimulateWindow *m_win;

    // 鼠标状态
    std::atomic<bool>   m_btnLeft   {false};
    std::atomic<bool>   m_btnMiddle {false};
    std::atomic<bool>   m_btnRight  {false};
    std::atomic<bool>   m_modCtrl   {false};
    std::atomic<bool>   m_modShift  {false};
    std::atomic<bool>   m_modAlt    {false};

    // 光标
    mutable std::mutex      m_cursorMtx;
    double                  m_cursorX = 0;
    double                  m_cursorY = 0;

    // 窗口尺寸
    std::atomic<int>    m_winW {1280};
    std::atomic<int>    m_winH {800};
    std::atomic<int>    m_fbW  {1280};
    std::atomic<int>    m_fbH  {800};
    std::atomic<double> m_dpi  {96.0};

    std::atomic<bool>   m_shouldClose {false};

    // 事件队列
    std::mutex                          m_qMtx;
    std::deque<std::function<void()>>   m_queue;
};

} // namespace mjqt
