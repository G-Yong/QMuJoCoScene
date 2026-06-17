#ifndef POINTCLOUDCOLLISION_H
#define POINTCLOUDCOLLISION_H

// ---------------------------------------------------------------------------
// PointCloudCollision —— 点云碰撞接入 MuJoCo 的完整示例
//
// 背景：MuJoCo 原生不认识点云，点云在 MujocoQuickItem 里只是 user_scn 里的
//       一堆渲染几何（addPointCloud），不是参与物理的 geom。因此：
//
//   · setExternalNarrowPhase 那条"硬约束"路径只拦截 MuJoCo *真实 mesh geom*
//     之间的窄相位，宽相位永远不会提议"点云对"，对点云不触发；
//   · 要把点云碰撞结果"打入 MuJoCo"，现实做法是走通用逃生口
//     withMutableSimulation：每帧锁内拿 mjModel*/mjData*，用第三方库（这里是
//     coal）算"机器人 body 网格 vs 点云"的穿透，再把结果写成 **外部笛卡尔力**
//     d->xfrc_applied（弹簧-阻尼惩罚力）。mj_step 每步都会把 xfrc_applied 积分
//     进动力学，这就是"打入系统"。
//
// 本类做的事：
//   1. setPoints(worldPoints)：保存点云（世界坐标），每点建一个 coal Sphere，
//      同时用 addPointCloud 渲染出来；
//   2. addCollisionBody(name)：把某个 MuJoCo *mesh* body 用 bodyCollisionMesh
//      提取三角网格，构建 coal BVH 碰撞模型；
//   3. update()：每帧（建议 ~120Hz）把 body 当前世界位姿同步给 coal，逐点
//      collide，得到穿透深度/法向，累加成 force/torque 写入 xfrc_applied，并把
//      发生穿透的点高亮成 hitColor。
//
// 线程：update() 在 GUI 线程调用，withMutableSimulation 内部持 sim.mtx 锁，与
//       物理线程串行化。xfrc_applied 在两次 update 之间保持不变并持续被施加，
//       因此 ~120Hz 周期驱动即可（生产环境若要逐物理步精确，可改用 MuJoCo 的
//       mjcb_control 全局回调，但那是另一套机制，不在本示例范围）。
// ---------------------------------------------------------------------------

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <memory>

#include "MujocoQuickItem.h"

class PointCloudCollision : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudCollision(QObject* parent = nullptr);
    ~PointCloudCollision() override;

    PointCloudCollision(const PointCloudCollision&) = delete;
    PointCloudCollision& operator=(const PointCloudCollision&) = delete;

    void setMujocoItem(MujocoQuickItem* item);

    // 复用一个已存在的渲染点云（由调用方用 addPointCloud/setPointCloudData 创建），
    // 而不是本类自建。baseColors 为逐点底色（与点数一致）：未穿透点显示底色，
    // 穿透点高亮 hitColor，从而既保留原始着色（如高度色）又能显示命中。
    // 传 cloudId<0 关闭复用、恢复自建点云行为。需在每次场景重载、点云重建后调用。
    void setRenderCloud(int cloudId, const QVector<QVector4D>& baseColors = {});

    // 设置点云（世界坐标）。重建 coal 点球与渲染点云；可随时调用更新数据。
    // 实际项目里把你自己的点云（相机/雷达）传进来即可。
    void setPoints(const QVector<QVector3D>& worldPoints);
    int  pointCount() const;

    // 每点半径（米）：既是 coal 碰撞球半径，也是渲染点尺寸。默认 0.01。
    void setPointRadius(double radius);

    // 注册一个参与碰撞的机器人 body（必须是 mesh geom body，否则
    // bodyCollisionMesh 取不到三角网格，会被跳过）。可多次调用。
    void addCollisionBody(const QString& bodyName);
    void clearCollisionBodies();

    // 惩罚力参数：沿"把 body 推离点"的方向 n，
    //   F = max(0, k*penetration - c*(v·n)) * n
    // k 越大越"硬"（但过大易抖/穿透），c 提供阻尼抑制反弹。
    void setStiffness(double k);   // 默认 3000 N/m
    void setDamping(double c);     // 默认 30 N·s/m

    // 法向取反兜底开关：若出现 body 被点云"吸住"而非弹开，置 true。默认 false。
    void setNormalFlip(bool flip);

    // 渲染颜色：未穿透点用 baseColor，穿透点高亮 hitColor。
    void setBaseColor(const QVector4D& rgba);
    void setHitColor(const QVector4D& rgba);

    // 场景加载完成后调用：注册各 body 的 coal 网格，并确保点云已渲染。
    void setupForLoadedScene();

public slots:
    // 执行一次"碰撞检测 + 注入惩罚力 + 高亮"。建议用 QTimer ~120Hz 周期调用。
    void update();

private:
    void rebuildCoalPoints();
    void refreshRenderColors(const QVector<unsigned char>& hitFlags);

    struct Impl;
    std::unique_ptr<Impl> d;
};

#endif // POINTCLOUDCOLLISION_H
