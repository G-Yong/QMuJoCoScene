#include "MujocoQuickItem.h"
#include "pointcloudcollision.h"
#include "plyloader.h"

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QSurfaceFormat>
#include <QQuickWidget>
#include <QApplication>
#include <QTimer>
#include <QLabel>
#include <algorithm>

namespace {
QString vec3Text(const QVector3D& v)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(static_cast<double>(v.x()), 0, 'f', 3)
        .arg(static_cast<double>(v.y()), 0, 'f', 3)
        .arg(static_cast<double>(v.z()), 0, 'f', 3);
}

QString collisionSummary(MujocoQuickItem* mujoco)
{
    if (!mujoco) return QStringLiteral("未找到 MujocoView");

    const int total = mujoco->contactCount();
    if (total <= 0)
        return QStringLiteral("碰撞: 无\ncontacts: 0");

    int activeCount = 0, penetratingCount = 0;
    for (int i = 0; i < total; ++i) {
        const ContactInfo c = mujoco->contact(i);
        if (c.active)      ++activeCount;
        if (c.penetrating) ++penetratingCount;
    }

    QString summary = QStringLiteral("碰撞/接触: 有\ncontacts: %1  active: %2  penetrating: %3")
        .arg(total).arg(activeCount).arg(penetratingCount);

    const int shown = std::min(total, 4);
    for (int i = 0; i < shown; ++i) {
        const ContactInfo c = mujoco->contact(i);
        summary += QStringLiteral(
            "\n#%1 %2 (body %3) <-> %4 (body %5)"
            "\n  dist=%6  normalForce=%7  active=%8"
            "\n  pos=%9")
            .arg(i)
            .arg(c.geom0Name, c.body0Name, c.geom1Name, c.body1Name)
            .arg(c.dist, 0, 'f', 6)
            .arg(c.normalForce, 0, 'f', 3)
            .arg(c.active ? QStringLiteral("yes") : QStringLiteral("no"))
            .arg(vec3Text(c.position));
    }
    if (total > shown)
        summary += QStringLiteral("\n... 还有 %1 个 contact").arg(total - shown);

    return summary;
}

// 简单高度热力色：t∈[0,1] → 蓝→青→黄→红，用于展示逐点颜色。
QVector4D heightColor(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    float r, g, b;
    if (t < 0.33f) {            // 蓝 → 青
        float u = t / 0.33f;  r = 0.0f; g = u; b = 1.0f;
    } else if (t < 0.66f) {     // 青 → 黄
        float u = (t - 0.33f) / 0.33f; r = u; g = 1.0f; b = 1.0f - u;
    } else {                    // 黄 → 红
        float u = (t - 0.66f) / 0.34f; r = 1.0f; g = 1.0f - u; b = 0.0f;
    }
    return QVector4D(r, g, b, 1.0f);
}

// bunny 点云：加载结果（世界坐标点 + 逐点高度色 + 包围信息）。
struct BunnyCloud {
    int                cloudId = -1;     // MujocoView 里的渲染点云 id
    QVector<QVector3D> points;           // 世界坐标点（供碰撞使用）
    QVector<QVector4D> colors;           // 逐点高度色（底色）
    QVector3D          center;           // XY 摆放中心
    float              topZ = 0.0f;      // 点云顶部 z（用于在其上方投放小球）
};

// 加载 .ply 点云，转换到 MuJoCo 的 Z-up，放大并摆放：XY 居中到 centerXY，
// 底部贴地（zmin=0）。创建一个 Circle 样式的彩色渲染点云并开启地面倒影。
// bunny 无颜色，按高度生成逐点色。成功返回 true 并填充 out。
bool setupBunnyPointCloud(MujocoQuickItem* mujoco, const QString& plyPath,
                          float scale, const QVector3D& centerXY,
                          float renderPointSize, BunnyCloud* out)
{
    PlyCloud ply;
    QString err;
    if (!loadPly(plyPath, &ply, &err)) {
        qWarning() << "[ply] load failed:" << plyPath << err;
        return false;
    }
    qDebug() << "[ply] loaded" << ply.count << "points from" << plyPath
             << "hasColor=" << ply.hasColor;

    QVector<QVector3D> pts;
    pts.reserve(ply.count);
    QVector3D vmin( 1e9f,  1e9f,  1e9f);
    QVector3D vmax(-1e9f, -1e9f, -1e9f);
    for (int i = 0; i < ply.count; ++i) {
        const float x = ply.positions[3 * i + 0];
        const float y = ply.positions[3 * i + 1];
        const float z = ply.positions[3 * i + 2];
        // Y-up → Z-up：绕 X 轴 +90°，(x, y, z) → (x, -z, y)
        QVector3D p(x * scale, -z * scale, y * scale);
        pts.append(p);
        vmin.setX(std::min(vmin.x(), p.x())); vmax.setX(std::max(vmax.x(), p.x()));
        vmin.setY(std::min(vmin.y(), p.y())); vmax.setY(std::max(vmax.y(), p.y()));
        vmin.setZ(std::min(vmin.z(), p.z())); vmax.setZ(std::max(vmax.z(), p.z()));
    }

    // 摆放：XY 中心移到 centerXY，底部贴地（zmin → 0）。
    const QVector3D mid((vmin.x() + vmax.x()) * 0.5f,
                        (vmin.y() + vmax.y()) * 0.5f,
                        0.0f);
    const QVector3D shift(centerXY.x() - mid.x(),
                          centerXY.y() - mid.y(),
                          -vmin.z());

    QVector<float> positions;
    QVector<float> colors;
    positions.reserve(ply.count * 3);
    colors.reserve(ply.count * 4);
    const float span = (vmax.z() > vmin.z()) ? (vmax.z() - vmin.z()) : 1.0f;
    out->points.clear();
    out->colors.clear();
    out->points.reserve(ply.count);
    out->colors.reserve(ply.count);
    for (int i = 0; i < pts.size(); ++i) {
        const QVector3D p = pts[i] + shift;
        positions << p.x() << p.y() << p.z();
        const QVector4D c = heightColor((pts[i].z() - vmin.z()) / span);
        colors << c.x() << c.y() << c.z() << c.w();
        out->points.append(p);
        out->colors.append(c);
    }

    const int cloudId = mujoco->addPointCloud(QVariantList(), renderPointSize,
                                              MujocoQuickItem::PointStyleSquare,
                                              QVector4D(1, 1, 1, 1));
    mujoco->setPointCloudData(cloudId, positions, colors);
    // 地面倒影：与地面材质 reflectance(.2) 接近的强度。
    mujoco->setPointCloudGroundReflection(cloudId, true, 0.0f, 0.2f);

    out->cloudId = cloudId;
    out->center  = QVector3D(centerXY.x(), centerXY.y(), 0.0f);
    out->topZ    = vmax.z() - vmin.z();   // 贴地后顶部高度
    return true;
}
} // namespace

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
// 修复：在主可执行文件中导出下面两个符号，NVIDIA / AMD 驱动会识别并
//      自动把本进程切到独显（业界标准做法）。
// ---------------------------------------------------------------------------
#if defined(_WIN32)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

int main(int argc, char *argv[])
{
    // 使用系统硬件 OpenGL（独立 GPU 由文件头部的导出符号强制选定）
    QGuiApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    // 关键：让 Mujoco 渲染线程的私有 GL context 与 Qt Quick 的 scenegraph
    //       context 共享，从而可以跨线程使用同一个 GL 纹理。
    QGuiApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QSurfaceFormat fmt;
    fmt.setVersion(4, 5);
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile); // mjr_* 用了部分固定管线兼容调用
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    // 注册 QML 类型
    qmlRegisterType<MujocoQuickItem>("Mujoco", 1, 0, "MujocoView");

    QQuickWidget *view = new QQuickWidget();
    view->setWindowTitle("MuJoCo in Qt Quick Demo");
    view->setResizeMode(QQuickWidget::SizeRootObjectToView);
    // 默认模型路径 —— 改成你自己的路径
    QString filePath;
    // filePath = "../../../../mujoco-3.8.0-windows-x86_64/model/humanoid/humanoid.xml";
    // filePath = "../../../../mujoco-3.8.0-windows-x86_64/model/cards/cards.xml";
    filePath = "../../../model/slide.xml";
    view->engine()->rootContext()->setContextProperty(
        "initialXmlPath",
        filePath);
    view->setSource(QUrl("qrc:/main.qml"));
    view->show();

    auto* mujoco = view->rootObject()
        ? view->rootObject()->findChild<MujocoQuickItem*>(QStringLiteral("mujocoView"))
        : nullptr;

    // 叠加一个纯文本标签，演示如何从 C++ 侧读取当前碰撞信息。
    QLabel *label = new QLabel(view);
    label->setStyleSheet("QLabel { color: white; background-color: rgba(0, 0, 0, 160); font-size: 13px; padding: 6px; border-radius: 4px; }");
    label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    label->setAttribute(Qt::WA_TransparentForMouseEvents); // 鼠标事件穿透
    label->setWordWrap(true);
    label->setMaximumWidth(620);
    label->setText(collisionSummary(mujoco));
    label->adjustSize();
    label->move(10, 10);
    label->show();

    // contactsChanged 仅在接触快照实际变化时发出，直接驱动标签刷新。
    if (mujoco) {
        QObject::connect(mujoco, &MujocoQuickItem::contactsChanged, view, [label, mujoco]() {
            label->setText(collisionSummary(mujoco));
            label->adjustSize();
        });
    }

    // ----------------------------------------------------------------------
    // 点云碰撞演示：小球(ball, mesh body) vs bunny 点云。
    // coal 计算 ball 网格 vs 各点球的穿透，惩罚力经 xfrc_applied 打回 MuJoCo；
    // 命中的点高亮成红色，其余保留高度色。bunny 点云带地面倒影。
    // ----------------------------------------------------------------------
    PointCloudCollision pcc;
    pcc.setMujocoItem(mujoco);

    QObject::connect(mujoco, &MujocoQuickItem::sceneLoaded, view, [=, &pcc](const QString& /*source*/) {
        // 1) 加载并渲染 bunny 点云（彩色 Circle + 地面倒影）。
        BunnyCloud bunny;
        if (!setupBunnyPointCloud(mujoco,
                                  QStringLiteral("../../../model/meshes/bunny.ply"),
                                  3.0f, QVector3D(0.4f, 0.0f, 0.0f),
                                  0.00035f, &bunny)) {
            return;
        }

        // 2) 碰撞参数：coal 点球半径略大于渲染点，保证 ball 不漏过点缝。
        pcc.setPointRadius(0.008);        // 此时 cloudId 尚未设置，不会改渲染点尺寸
        pcc.setStiffness(2500.0);
        pcc.setDamping(40.0);

        // 3) 复用彩色渲染点云做命中高亮（保留高度色），并接入碰撞。
        pcc.setRenderCloud(bunny.cloudId, bunny.colors);
        pcc.setPoints(bunny.points);
        pcc.addCollisionBody("ball");
        pcc.setupForLoadedScene();

        // 4) 把小球放到 bunny 上方，让它落到点云上演示碰撞。
        int ballId = -1;
        for (int i = 0; i < mujoco->objectCount(); ++i) {
            if (mujoco->objectInfo(i).name == QStringLiteral("ball")) { ballId = i; break; }
        }
        if (ballId >= 0) {
            mujoco->setObjectPosition(
                ballId, QVector3D(bunny.center.x(), bunny.center.y(),
                                  bunny.topZ + 0.2f));
        }
    });

    // 周期性驱动点云碰撞注入（~120Hz）。
    QTimer* pccTimer = new QTimer(view);
    QObject::connect(pccTimer, &QTimer::timeout, &pcc, &PointCloudCollision::update);
    pccTimer->start(8);

    return app.exec();
}

