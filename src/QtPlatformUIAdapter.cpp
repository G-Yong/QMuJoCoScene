#include "QtPlatformUIAdapter.h"
#include "QtSimulateWindow.h"

#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>
#include <QGuiApplication>
#include <QClipboard>

namespace mjqt {

QtPlatformUIAdapter::QtPlatformUIAdapter(QtSimulateWindow *win) : m_win(win) {}
QtPlatformUIAdapter::~QtPlatformUIAdapter() { FreeMjrContext(); }

// ---------------- 基本 getter ----------------
std::pair<double,double> QtPlatformUIAdapter::GetCursorPosition() const {
    std::lock_guard<std::mutex> lk(m_cursorMtx);
    return {m_cursorX, m_cursorY};
}
double QtPlatformUIAdapter::GetDisplayPixelsPerInch() const   { return m_dpi.load(); }
std::pair<int,int> QtPlatformUIAdapter::GetFramebufferSize() const { return {m_fbW.load(),  m_fbH.load()};  }
std::pair<int,int> QtPlatformUIAdapter::GetWindowSize()      const { return {m_winW.load(), m_winH.load()}; }
bool QtPlatformUIAdapter::IsGPUAccelerated() const { return true; }
bool QtPlatformUIAdapter::ShouldCloseWindow() const { return m_shouldClose.load(); }

// ---------------- 通过 QtSimulateWindow 间接做的 ----------------
void QtPlatformUIAdapter::SwapBuffers()             { m_win->swapBuffersFromRenderThread(); }
void QtPlatformUIAdapter::SetVSync(bool enabled)    { m_win->setVSyncFromRenderThread(enabled); }
void QtPlatformUIAdapter::SetWindowTitle(const char *t) { m_win->postSetTitle(QString::fromUtf8(t)); }
void QtPlatformUIAdapter::ToggleFullscreen()        { m_win->postToggleFullscreen(); }
void QtPlatformUIAdapter::SetClipboardString(const char *text) {
    QGuiApplication::clipboard()->setText(QString::fromUtf8(text));
}

mjtButton QtPlatformUIAdapter::TranslateMouseButton(int btn) const {
    switch (btn) {
        case 1: return mjBUTTON_LEFT;
        case 2: return mjBUTTON_RIGHT;
        case 3: return mjBUTTON_MIDDLE;
        default: return mjBUTTON_NONE;
    }
}

// ---------------- 事件队列 ----------------
void QtPlatformUIAdapter::PollEvents() {
    std::deque<std::function<void()>> drained;
    {
        std::lock_guard<std::mutex> lk(m_qMtx);
        drained.swap(m_queue);
    }
    for (auto &fn : drained) fn();
}

void QtPlatformUIAdapter::PostMouseButton(int qtBtn, int act, double x, double y) {
    {
        std::lock_guard<std::mutex> lk(m_cursorMtx);
        m_cursorX = x; m_cursorY = y;
    }
    bool down = (act == 1);
    if      (qtBtn == 1) m_btnLeft.store(down);
    else if (qtBtn == 2) m_btnRight.store(down);
    else if (qtBtn == 3) m_btnMiddle.store(down);

    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, qtBtn, act] { OnMouseButton(qtBtn, act); });
}

void QtPlatformUIAdapter::PostMouseMove(double x, double y) {
    {
        std::lock_guard<std::mutex> lk(m_cursorMtx);
        m_cursorX = x; m_cursorY = y;
    }
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, x, y] { OnMouseMove(x, y); });
}

void QtPlatformUIAdapter::PostScroll(double dx, double dy) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, dx, dy] { OnScroll(dx, dy); });
}

void QtPlatformUIAdapter::PostKey(int mjKey, int act) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    // 我们的 TranslateKeyCode 是恒等映射，所以传过去的就是 mjKey
    m_queue.emplace_back([this, mjKey, act] { OnKey(mjKey, /*scancode*/0, act); });
}

void QtPlatformUIAdapter::PostResize(int w, int h) {
    std::lock_guard<std::mutex> lk(m_qMtx);
    m_queue.emplace_back([this, w, h] { OnWindowResize(w, h); });
}

void QtPlatformUIAdapter::PostClose() { m_shouldClose.store(true); }

void QtPlatformUIAdapter::SetModifiers(bool ctrl, bool shift, bool alt) {
    m_modCtrl.store(ctrl);
    m_modShift.store(shift);
    m_modAlt.store(alt);
}

void QtPlatformUIAdapter::SetWindowGeometry(int win_w, int win_h, int fb_w, int fb_h, double dpi) {
    m_winW.store(win_w);  m_winH.store(win_h);
    m_fbW.store(fb_w);    m_fbH.store(fb_h);
    m_dpi.store(dpi);
}

} // namespace mjqt
