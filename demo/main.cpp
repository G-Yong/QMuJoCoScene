#include "SimulateView.h"
#include <QApplication>
#include <QSurfaceFormat>
#include <QStringList>
#include <QMainWindow>

#ifndef ASSETS_DIR
#define ASSETS_DIR "."
#endif

// ---------------------------------------------------------------------------
// 强制混合显卡（NVIDIA Optimus / AMD PowerXpress）选用独立 GPU
//
// 背景：在带集显 + 独显的 Windows 机器上，操作系统默认让进程跑在集显上。
//      若集显驱动缺失/异常，Windows 会回退到 "Microsoft GDI Generic" 软件
//      OpenGL 1.1（无 ARB_framebuffer_object），导致 MuJoCo 的
//      mjr_makeContext 报 "ERROR: OpenGL ARB_framebuffer_object required"。
//
// 修复：在 *主可执行文件* 中导出下面两个符号，NVIDIA / AMD 驱动会识别并
//      自动把本进程切到独显（这是业界标准做法，不需要用户手动改 NV 控制面板）。
//
// 注意：必须导出在 .exe 主模块。放在静态库 / DLL 里无效，所以写在 main.cpp。
// 参考：
//   - https://docs.nvidia.com/gameworks/content/technologies/desktop/optimus.htm
//   - AMD Driver Profile XML, "PowerXpressRequestHighPerformance"
// ---------------------------------------------------------------------------
#if defined(_WIN32)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

int main(int argc, char *argv[])
{
    // 使用系统硬件 OpenGL（独立 GPU 由文件头部的 NvOptimusEnablement 导出符号强制选定）。
    // MuJoCo 的 mjr_makeContext 要求 ARB_framebuffer_object（GL 3.0+ 核心功能），
    // 独立 GPU 驱动可正常满足此要求。
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    // MuJoCo 渲染要求 OpenGL 3.3 Core 或更高的兼容上下文
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);  // mjr_* 用了部分固定管线兼容调用
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(4);
    // fmt.setSwapInterval(0);  // 禁用 vsync；物理时序由仿真循环控制，不需要帧同步锁速
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    auto *view = new SimulateView();
    view->setWindowTitle("MuJoCo Simulate in Qt");
    view->resize(1280, 768);
    view->show();
    view->start(QString("C:/Users/Administrator/Desktop/robotSim/qt-mujoco/mujoco-3.8.0-windows-x86_64/model/cards/cards.xml")); // 改成你自己的路径
    return app.exec();

}
