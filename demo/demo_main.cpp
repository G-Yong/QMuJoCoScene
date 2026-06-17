#include "MujocoQuickItem.h"
#include "coalcollision.h"
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

// 加载 .ply 点云并作为点云图层加入 MuJoCo 视图。
// Stanford bunny 是 Y-up，这里旋转到 MuJoCo 的 Z-up 并放大、平移到可见位置；
// 该 ply 无颜色，按高度生成逐点颜色以演示彩色点云。返回 cloudId（失败 -1）。
int addPlyPointCloud(MujocoQuickItem* mujoco, const QString& plyPath,
                     float scale, const QVector3D& offset)
{
    PlyCloud ply;
    QString err;
    if (!loadPly(plyPath, &ply, &err)) {
        qWarning() << "[ply] load failed:" << plyPath << err;
        return -1;
    }
    qDebug() << "[ply] loaded" << ply.count << "points from" << plyPath
             << "hasColor=" << ply.hasColor;

    QVector<QVector3D> pts;
    pts.reserve(ply.count);
    float zmin = 1e9f, zmax = -1e9f;
    for (int i = 0; i < ply.count; ++i) {
        const float x = ply.positions[3 * i + 0];
        const float y = ply.positions[3 * i + 1];
        const float z = ply.positions[3 * i + 2];
        // Y-up → Z-up：绕 X 轴 +90°，(x, y, z) → (x, -z, y)
        QVector3D p(x * scale, -z * scale, y * scale);
        p += offset;
        pts.append(p);
        zmin = std::min(zmin, p.z());
        zmax = std::max(zmax, p.z());
    }

    QVector<float> positions;
    QVector<float> colors;
    positions.reserve(ply.count * 3);
    colors.reserve(ply.count * 4);
    const float span = (zmax > zmin) ? (zmax - zmin) : 1.0f;
    for (int i = 0; i < pts.size(); ++i) {
        const QVector3D& p = pts[i];
        positions << p.x() << p.y() + 0.2 << p.z();
        if (ply.hasColor) {
            colors << ply.colors[4 * i + 0] << ply.colors[4 * i + 1]
                   << ply.colors[4 * i + 2] << ply.colors[4 * i + 3];
        } else {
            const QVector4D c = heightColor((p.z() - zmin) / span);
            colors << c.x() << c.y() << c.z() << c.w();
        }
    }

    // 用 Pixel 样式（固定屏幕像素）保证清晰可见；可改为 PointStyleCircle/Sphere。
    const int cloudId = mujoco->addPointCloud(QVariantList(), 2.0f,
                                              MujocoQuickItem::PointStylePixel,
                                              QVector4D(1, 1, 1, 1));
    mujoco->setPointCloudData(cloudId, positions, colors);
    return cloudId;
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

    CoalCollision coalCollision;
    coalCollision.setMujocoItem(mujoco);

    QObject::connect(mujoco, &MujocoQuickItem::sceneLoaded, view, [=, &coalCollision](const QString& source) {
        // 绘制轨迹
        auto tjId = mujoco->addTrajectory(1024,
                              2.0,
                              QVector4D(1.0f, 0.85f, 0.2f, 1.0f),
                              true);
        int bodyId = 0;
        for (int i = 0; i < mujoco->objectCount(); ++i) {
            auto info = mujoco->objectInfo(i);
            qDebug() << "Body" << i << ":" << info.name;
            if (info.name == "head") {
                bodyId = i;
                break;
            }
        }
        mujoco->setTrajectoryTrackedBody(tjId, bodyId, 0);


        coalCollision.setupCoalForLoadedScene();
        coalCollision.setCoalSecurityMargin(0.00);
        coalCollision.setCoalNormalFlip(false);
        coalCollision.addCoalPair("ball", "slide");

        // 加载 bunny.ply 点云并渲染（GPU GL_POINTS 叠加层，按高度着色）。
        // 放大 3 倍、平移到 (0.4, 0, 0) 处，使其在场景中清晰可见。
        addPlyPointCloud(mujoco, QStringLiteral("../../../model/meshes/bunny.ply"),
                         3.0f, QVector3D(0.4f, 0.0f, 0.0f));
     });

    // ----------------------------------------------------------------------
    // 点云碰撞示例：coal(body 网格 vs 点云) → 惩罚力 xfrc_applied 打入 MuJoCo。
    // 实际项目里把 setPoints(...) 换成你自己的点云数据即可。
    // ----------------------------------------------------------------------
    PointCloudCollision pcc;
    pcc.setMujocoItem(mujoco);

    QObject::connect(mujoco, &MujocoQuickItem::sceneLoaded, view, [=, &pcc](const QString& /*source*/) {
        pcc.addCollisionBody("ball");   // ball 是 mesh body，可被 coal 提取网格
        pcc.setPointRadius(0.01);
        pcc.setStiffness(3000.0);
        pcc.setDamping(30.0);

        // 演示点云：一片水平点阵（实际中替换为你的传感器点云）。
        QVector<QVector3D> demoCloud;
        for (int ix = -8; ix <= 8; ++ix)
            for (int iy = -8; iy <= 8; ++iy)
                demoCloud.append(QVector3D(ix * 0.02f, iy * 0.02f, 0.12f));
        pcc.setPoints(demoCloud);

        pcc.setupForLoadedScene();
    });

    // 周期性驱动点云碰撞注入（~120Hz）。
    QTimer* pccTimer = new QTimer(view);
    QObject::connect(pccTimer, &QTimer::timeout, &pcc, &PointCloudCollision::update);
    pccTimer->start(8);

    return app.exec();
}

