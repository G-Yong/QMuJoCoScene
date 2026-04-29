#pragma once
// ---------------------------------------------------------------------------
// QtSimulateWindow
//
// QWindow（OpenGL 表面）+ 渲染线程 + 物理线程，承载官方 mujoco::Simulate。
// 通过 QWidget::createWindowContainer 嵌入到普通 Qt 部件树。
//
// 用法：
//   auto *win    = new QtSimulateWindow();
//   auto *widget = QWidget::createWindowContainer(win, parent);
//   win->start("path/to/model.xml");
// ---------------------------------------------------------------------------

#include <QWindow>
#include <QString>
#include <QPointer>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

class QOpenGLContext;

namespace mujoco { class Simulate; }
namespace mjqt   { class QtPlatformUIAdapter; }

class QtSimulateWindow : public QWindow {
    Q_OBJECT
public:
    explicit QtSimulateWindow(QWindow *parent = nullptr);
    ~QtSimulateWindow() override;

    // 启动两条工作线程并加载模型；filename 可为空，之后再 loadModel
    void start(const QString &filename = QString());
    void stop();

    // 请求加载新模型（异步）
    void loadModel(const QString &filename);

    // ---- 仅渲染线程调用 ----
    void swapBuffersFromRenderThread();
    void setVSyncFromRenderThread(bool on);

    // ---- Qt 主线程：来自适配器的请求 ----
    void postSetTitle(const QString &t);
    void postToggleFullscreen();

protected:
    // QWindow 事件
    bool event(QEvent *e) override;
    void exposeEvent(QExposeEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void keyPressEvent(QKeyEvent *e) override;
    void keyReleaseEvent(QKeyEvent *e) override;

private:
    int  qtMouseButtonToInternal(int btn) const; // 1=L,2=R,3=M
    int  qtKeyToMjui(int key) const;
    void updateModifiersFrom(int qtMods);
    void updateGeometryToAdapter();

    void renderThreadMain();
    void physicsThreadMain();

    // 渲染 / 物理线程之间共享的资源
    std::unique_ptr<mujoco::Simulate>           m_sim;
    std::unique_ptr<mjqt::QtPlatformUIAdapter>  m_adapter;  // 由 m_sim 接管所有权后只持引用
    mjqt::QtPlatformUIAdapter*                  m_adapterRaw = nullptr;

    QOpenGLContext*  m_ctx = nullptr;     // 在渲染线程里 makeCurrent

    std::thread       m_renderThread;
    std::thread       m_physicsThread;
    std::atomic<bool> m_running {false};

    // 待加载的文件名（Qt 主线程写，物理线程读）
    std::mutex   m_pendingMtx;
    QString      m_pendingFile;
    std::atomic<bool> m_hasPendingLoad {false};
};
