HEADERS += \
    $$PWD/QtPlatformUIAdapter.h \
    $$PWD/QtSimulateWindow.h \
    $$PWD/SimulateView.h \
    $$PWD/lodepng.h

SOURCES += \
    $$PWD/QtPlatformUIAdapter.cpp \
    $$PWD/QtSimulateWindow.cpp \
    $$PWD/SimulateView.cpp


MUJOCO_DIR = $$PWD/../mujoco-3.8.0-windows-x86_64
message("Using MuJoCo at: $$MUJOCO_DIR")

INCLUDEPATH += $$MUJOCO_DIR/include
# 官方 simulate 源码所在目录（有 simulate.h / platform_ui_adapter.h）
INCLUDEPATH += $$MUJOCO_DIR/simulate
# 让 simulate.cc 找到我们提供的 lodepng 替身
INCLUDEPATH += $$PWD

SOURCES += \
    $$MUJOCO_DIR/simulate/simulate.cc \
    $$MUJOCO_DIR/simulate/platform_ui_adapter.cc

win32 {
    LIBS += -L$$MUJOCO_DIR/lib -lmujoco
    LIBS += -lopengl32

    LIBS += -L$$MUJOCO_DIR/bin

    # ------------------------------------------------------------------
    # Mesa 软件 OpenGL 回退（默认禁用）
    #
    # 说明：Qt 已设置 AA_UseDesktopOpenGL，两者都会走系统 opengl32.dll（硬件驱动），
    #       无需再覆盖。只在真实硬件 GL 不可用时（如无驱动的纯虚拟机）才启用。
    #
    # 启用方式：qmake "MUJOCO_USE_MESA_FALLBACK=1"
    # ------------------------------------------------------------------
    equals(MUJOCO_USE_MESA_FALLBACK, 1) {
        # ------------------------------------------------------------------
        #  opengl32sw.dll  —— Qt 自带的 Mesa3D 软件 OpenGL（GL 3.3 + FBO），
        #  用于在虚拟机 / 远程桌面 / 无显卡驱动环境下让 MuJoCo 渲染正常工作
        #  配合 main.cpp 的 Qt::AA_UseSoftwareOpenGL）
        # ------------------------------------------------------------------
        OPENGL_SW_DLL_SRC = $$shell_path($$[QT_INSTALL_BINS]/opengl32sw.dll)
        message("Mesa fallback enabled: copying opengl32sw.dll as opengl32.dll")
        DEST_GL_DLL = $$shell_path($$DEST_DLL_DIR/opengl32.dll)
        QMAKE_POST_LINK += $$QMAKE_COPY \"$$OPENGL_SW_DLL_SRC\" \"$$DEST_GL_DLL\" $$escape_expand(\\n\\t)
    } else {
        message("Using system hardware OpenGL (default). Pass MUJOCO_USE_MESA_FALLBACK=1 if GL errors occur.")
    }
}
unix:!macx {
    LIBS += -L$$MUJOCO_DIR/lib -lmujoco -lGL
    QMAKE_RPATHDIR += $$MUJOCO_DIR/lib
}