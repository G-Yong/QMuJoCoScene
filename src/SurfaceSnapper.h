#pragma once
// ---------------------------------------------------------------------------
// SurfaceSnapper
//
// 拖动示教「表面吸附」的几何求解器：给一个世界坐标查询点，返回最近的
// 物体表面点（及 geom id）。纯 C++ 类（仿 DragTeachGizmo），不碰 sim 锁、
// 不建相机、不发信号；只读 mjModel / mjData。
//
// 支持求最近点的 geom 类型：
//   plane / sphere / capsule / cylinder / box / ellipsoid / mesh
// 其余类型（hfield / flex / sdf）直接跳过，不会产生“看不见的吸附”。
//
// 候选排除：可设置一个 tracked site 所在的 body，本类会沿 parent 链找到
// 「世界下第一级祖先」，把它的整棵子树排除掉（= 当前示教的机器人手臂 +
// 工具），避免工具尖吸到自己的手腕上。
//
// 回差：吸附距离 absorb 内进入吸附，超过 release（默认 2× absorb）才脱开，
// 防止阈值边缘抖动。判断一律基于调用方传进来的“自由位姿”，与吸附后的
// 位置无关，因此长时间拖动不会漂移。
// ---------------------------------------------------------------------------

#include <QString>
#include <QVector3D>

#include <vector>

struct mjModel_;
typedef mjModel_ mjModel;
struct mjData_;
typedef mjData_ mjData;

class SurfaceSnapper {
public:
    struct Hit {
        QVector3D point;         // 世界系最近表面点
        int       geomId = -1;   // 命中的 geom
        float     distance = 0;  // 查询点到 point 的距离（米）
    };

    // ---- 配置 -------------------------------------------------------------
    void  setEnabled(bool on) { m_enabled = on; }

    // absorb = 进入吸附的距离；release = 脱开的距离（< absorb 时自动取 2× absorb）。
    void  setThresholdM(float absorb, float release);
    float absorbThresholdM() const { return m_absorbM; }

    // ---- 排除集合 ---------------------------------------------------------
    // siteBodyId = tracked site 所在 body。内部缓存，重复调用开销可忽略；
    // 模型指针 / body 数变化时自动重建。
    void setTrackedBody(const mjModel* m, int siteBodyId);
    void clearExclusions();

    // ---- 查询 -------------------------------------------------------------
    // 按回差状态机判断是否处于吸附；返回 true 时 out 为吸附点。
    bool query(const mjModel* m, const mjData* d, const QVector3D& p, Hit& out);
    // 只求最近表面点（不做阈值/回差判断，自检与调试用）。
    bool nearestPoint(const mjModel* m, const mjData* d, const QVector3D& p, Hit& out) const;

    void reset();                       // 松手 / 取消时清吸附状态
    bool snapped() const { return m_snapped; }   // 当前是否处于吸附状态

    // ---- 自检 -------------------------------------------------------------
    // 内联 MJCF 构造 plane/box/sphere/cylinder/capsule/ellipsoid/mesh，
    // 逐类型断言最近点（容差 1 µm），并验证排除子树与回差规则。
    // 返回空串 = 通过；否则是失败描述（ASCII）。
    static QString selfTest();

private:
    bool computeNearest(const mjModel* m, const mjData* d, const QVector3D& p,
                        float maxDist, Hit& out) const;

    bool  m_enabled  = true;
    float m_absorbM  = 0.010f;   // 10 mm
    float m_releaseM = 0.020f;   // 20 mm
    bool  m_snapped  = false;

    // 排除缓存（按 body 的位图）。
    const mjModel*             m_exclModel    = nullptr;
    int                        m_exclSiteBody = -1;
    int                        m_exclNbody    = 0;
    std::vector<unsigned char> m_excluded;
};
