#include "SimulateView.h"
#include <QApplication>
#include <QSurfaceFormat>
#include <QStringList>
#include <QMainWindow>

#ifndef ASSETS_DIR
#define ASSETS_DIR "."
#endif
int main(int argc, char *argv[])
{
    // ------------------------------------------------------------------
    // OpenGL 后端选择
    // ------------------------------------------------------------------
    // MuJoCo 的 mjr_makeContext 内部使用 GLAD 加载桌面 OpenGL，
    // 并要求 ARB_framebuffer_object（GL 3.0+ 核心功能）。
    // 在虚拟机 / 远程桌面 / 没有显卡驱动 的环境下，Windows 默认提供的
    // 是「Microsoft GDI Generic」OpenGL 1.1，没有 FBO，会直接报：
    //     ERROR: OpenGL ARB_framebuffer_object required
    //
    // 解决：把 Qt 自带的 Mesa3D 软件渲染器 opengl32sw.dll 重命名为
    // opengl32.dll 放到可执行文件同目录（.pro 里已配置自动拷贝）。
    // Windows DLL 搜索顺序会优先加载应用程序目录下的 opengl32.dll，
    // 因此 Qt 与 MuJoCo（通过 -lopengl32 链接到系统 opengl32）都会
    // 使用这份 Mesa 软件实现，提供 GL 3.3 + ARB_framebuffer_object。
    //
    // 配合本目的需使用 AA_UseDesktopOpenGL（不要用 AA_UseSoftwareOpenGL，
    // 后者会让 Qt 加载 opengl32sw.dll，但 MuJoCo 仍然走系统 opengl32.dll，
    // 二者用不同 GL 实现，会失败）。
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    // MuJoCo 渲染要求 OpenGL 3.3 Core 或更高的兼容上下文
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);  // mjr_* 用了部分固定管线兼容调用
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4);
    fmt.setSwapInterval(0);  // 禁用 vsync；物理时序由仿真循环控制，不需要帧同步锁速
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    auto *view = new SimulateView();
    view->setWindowTitle("MuJoCo Simulate in Qt");
    view->resize(1280, 768);
    view->show();
    view->start(QString("C:/Users/Administrator/Desktop/robotSim/qt-mujoco/mujoco-3.8.0-windows-x86_64/model/cards/cards.xml")); // 改成你自己的路径
    return app.exec();

}
