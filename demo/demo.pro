#-------------------------------------------------
#  demo.pro
#  Qt 5.15.2 + MuJoCo 3.8.0 集成演示 (qmake)
#-------------------------------------------------

QT       += core gui qml quick quickwidgets
CONFIG   += c++17
TARGET    = demo
TEMPLATE  = app

# MSVC 默认以 GBK 读取源文件，遇到 UTF-8 中文会报 C2001（常量中有换行符）
# /utf-8 = /source-charset:utf-8 /execution-charset:utf-8，一次性解决
win32-msvc* {
    QMAKE_CXXFLAGS += /utf-8
}

include(../src/qmujocoscene.pri)

# ----------------------------------------------------------------------------
# 源文件 / 资源
# ----------------------------------------------------------------------------
SOURCES += \
    coalcollision.cpp \
    pointcloudcollision.cpp \
    main.cpp

RESOURCES += qml.qrc

HEADERS += \
    coalcollision.h \
    pointcloudcollision.h \
    plyloader.h \
    assimploader.h


INCLUDEPATH += $$PWD/../thirdparty/coal/include
INCLUDEPATH += $$PWD/../thirdparty/eigen3_x64-windows/include/eigen3
INCLUDEPATH += $$PWD/../thirdparty/boost-math_x64-windows/include
# assimp 已通过 coal 内部链接，无需在本项目中单独引入头文件或库。
# 运行时仍需 assimp-vc142-mt.dll 部署在可执行文件同目录（coal 依赖）。
LIBS += -L$$PWD/../thirdparty/coal/lib -lcoal
LIBS += -L$$PWD/../thirdparty/bin
