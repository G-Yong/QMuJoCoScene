#ifndef COALCOLLISION_H
#define COALCOLLISION_H

#include <QObject>
#include <QVector3D>
#include <QHash>
#include <QSet>
#include <QVector4D>
#include <QQuaternion>
#include <QPair>
#include <memory>

#include "MujocoQuickItem.h"

// 单个接触点信息（世界坐标系）。
struct CoalContactPoint {
    QVector3D position;          // 接触点世界坐标
    QVector3D normal;            // 接触法向（从 bodyB 指向 bodyA）
    double    penetrationDepth = 0.0;
};

// 一对 body 的碰撞检测结果。
struct CoalCollisionResult {
    QString bodyA;
    QString bodyB;
    bool    colliding = false;
    double  distanceLowerBound = 0.0;       // 未碰撞时两者最近距离下界
    QVector<CoalContactPoint> contacts;     // 接触点列表（colliding 时非空）
};

class CoalCollisionDetector {
public:
    CoalCollisionDetector();
    ~CoalCollisionDetector();

    CoalCollisionDetector(const CoalCollisionDetector&) = delete;
    CoalCollisionDetector& operator=(const CoalCollisionDetector&) = delete;

    // 注册一个 body 的三角网格。vertices 为 body 局部坐标系顶点，indices 为
    // 三角面索引（长度须为 3 的倍数）。同名重复注册会覆盖。
    // 返回是否构建出有效网格模型。
    bool registerBody(const QString& name,
                      const QVector<QVector3D>& vertices,
                      const QVector<int>& indices);
    bool hasBody(const QString& name) const;
    void removeBody(const QString& name);
    void clear();

    // 设置需要检测的 body 对。仅当两个 body 都已 registerBody 时该对才生效。
    void setPairs(const QVector<QPair<QString, QString>>& pairs);
    void addPair(const QString& a, const QString& b);
    void clearPairs();
    QVector<QPair<QString, QString>> pairs() const;

    // 更新某个 body 的世界位姿（位置 + 四元数，四元数标量在前）。
    void updatePose(const QString& name,
                    const QVector3D& position,
                    const QQuaternion& orientation);

    // 对所有已注册且双方都存在的 body 对执行碰撞检测，返回每对的结果。
    QVector<CoalCollisionResult> detect();

    // 仅对单个 body 对执行碰撞检测（两者须已 registerBody，并已通过 updatePose
    // 设置好世界位姿）。供把 coal 接入 MuJoCo 求解器的窄阶段回调按需调用，避免
    // 每次都遍历全部 pair。a/b 任一未注册时返回 colliding=false 的空结果。
    CoalCollisionResult detectPair(const QString& a, const QString& b);

    // 安全裕度（米）：两物体间距小于该值即视为碰撞。默认 0。
    void   setSecurityMargin(double margin);
    double securityMargin() const;

private:
    struct Impl;
    std::unique_ptr<Impl> d;
};

class CoalCollision : public QObject
{
    Q_OBJECT
public:
    explicit CoalCollision(QObject *parent = nullptr);
    void setMujocoItem(MujocoQuickItem* item);

    // 抽离一对 body 到 coal 做精确碰撞检测。bodyA/bodyB 为 MuJoCo 模型中的 body
    // 名称。可在场景加载前调用（延迟到加载后生效），也可加载后调用（立即生效）。
    Q_INVOKABLE void addCoalPair(const QString& bodyA, const QString& bodyB);
    // 清空所有 coal 碰撞对与已注册网格。
    Q_INVOKABLE void clearCoalPairs();
    // 安全裕度（米）：间距小于该值即视为碰撞，默认 0。
    Q_INVOKABLE void setCoalSecurityMargin(double margin);
    // 接触法向符号开关。coal 法向（o1→o2）与 MuJoCo（geom[0]→geom[1]）约定一致,
    // 默认 false（不取反）即正确推开物体。仅当出现异常（物体相互"吸住"而非弹开）
    // 时才需翻转为 true 兜底。
    Q_INVOKABLE void setCoalNormalFlip(bool flip);
    Q_INVOKABLE bool coalNormalFlip() const;


    void setupCoalForLoadedScene();
signals:

private:
    void registerCoalBody(const QString& name);
    void updateCollisionHighlight();
    int  findBodyId(const QString& name) const;

    // coal physics：注册 coal body；建立 id↔name 反查与接管 body 对集合；
    // 把 filter / provider 安装到 SimulationView。
    void setupCoalPhysics();
    void installCoalNarrowPhase();
    void uninstallCoalNarrowPhase();
    // 重置 coal 运行态。restoreModelState=true 时按记录恢复被改动的 body 颜色
    //（仅当 body id 仍有效，如同场景内切换模式时）；场景重载时传 false。
    void teardownCoalState(bool restoreModelState);
    // 按 m_coalPairNames 重新注册网格、加入检测对并安装窄阶段。需 m_coalSceneReady。
    void rebuildCoalAndActivate();

private:
    MujocoQuickItem *mMujocoItem = nullptr;   // 场景中的 MujocoQuickItem

    // coal 精确碰撞（接入 MuJoCo 求解器）
    QScopedPointer<CoalCollisionDetector> m_coal;
    QVector<QPair<QString, QString>> m_coalPairNames;   // 用户请求的碰撞对（按名字）
    QHash<QString, int>  m_coalBodyId;                  // 已注册 body 名 → bodyId
    bool m_coalSceneReady = false;
    bool m_coalPhysicsInstalled = false;    // 当前是否已安装窄阶段回调
    bool m_coalNormalFlip = false;          // 法向取反（见 setCoalNormalFlip）
    QHash<int, QString> m_coalBodyName;     // bodyId → name 反查（provider 用）
    QSet<QPair<int, int>> m_coalPhysicsPairs; // 接管的 body 对（归一化 min,max)
};

#endif // COALCOLLISION_H
