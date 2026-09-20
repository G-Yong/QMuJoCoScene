#pragma once
// ---------------------------------------------------------------------------
// AxisGizmo
//
// 附加轴 / 变位机的拖动 gizmo：机器人本体之外的**单自由度关节**（旋转轴 hinge /
// 平移轴 slide）。与 TCP 的 6-DoF DragTeachGizmo 不同，这里每个手柄只驱动一个关节，
// **拖动直接改该关节的 qpos，不涉及逆解**。
//
// 与 DragTeachGizmo 同样是纯 C++ 类（非 QObject）：不碰 sim 锁、不建相机、不发信号。
// 宿主 MujocoQuickItem 持 sim.mtx 后驱动它，传入相机矩阵/视口，拖动结果通过 move()
// 的出参回给宿主去写 qpos。
//
// 关节类型自动按 mjModel::jnt_type 判定：hinge→旋转环，slide→平移箭头（预留）。
// ---------------------------------------------------------------------------

#include <QMatrix4x4>
#include <QPointF>
#include <QString>
#include <QVector3D>

#include <mutex>
#include <vector>

struct mjModel_;
typedef mjModel_ mjModel;
struct mjData_;
typedef mjData_ mjData;
struct mjvScene_;
typedef mjvScene_ mjvScene;

class AxisGizmo {
public:
    enum Kind { Rotary = 0, Prismatic = 1 };   // Prismatic = 平移轴（预留）

    // always-on-top 叠加层的一批同线宽线段（每段 2 个端点），与 DragTeachGizmo 一致。
    struct LineBatch {
        float              widthPx = 1.0f;
        std::vector<float> xyz;
        std::vector<float> rgba;
    };

    AxisGizmo() = default;

    // 配置要控制的附加轴关节名（hinge / slide）。会重置内部状态。
    void setJointNames(const std::vector<QString>& names);
    void clear() { setJointNames({}); }
    int  axisCount() const { return static_cast<int>(m_axes.size()); }

    bool visible() const { return m_visible; }
    bool setVisible(bool on);              // 返回是否变化；隐藏时清 dragging/hover
    bool dragging() const { return m_dragging; }
    bool alwaysOnTop() const { return m_alwaysOnTop; }
    bool setAlwaysOnTop(bool on);
    void setScreenSizePx(double px) { m_screenSizePx = px; }
    void setLastMouse(const QPointF& p) { m_lastMouse = p; }

    // 每帧（宿主持 sim.mtx）：按 mjData 刷新每个轴的世界锚点/轴向/当前值 + 关节类型。
    // 返回是否发生变化（需要重建 geom）。
    bool updateFromData(const mjModel* m, const mjData* d);
    // 恒定屏幕尺寸：按当前相机反解每个轴的显示尺寸（环半径 / 箭头长度）。返回是否变化。
    bool updateEffectiveSizes(const QMatrix4x4& vp, const QVector3D& camPos, float w, float h);
    // 在 [startIndex, ...) 写 geom；always-on-top 时改为收集到叠加层线缓冲。返回新 ngeom。
    int  rebuildGeoms(mjvScene* userScene, int startIndex);

    // 命中测试：返回命中的轴索引（-1 未命中）。
    int  hitTest(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) const;
    bool press(const QPointF& mouse, const QMatrix4x4& vp, float w, float h);
    // 拖动：返回是否处于拖动；edited=true 时 outJoint/outValue 有效（宿主写该关节 qpos）。
    bool move(const QPointF& mouse, const QMatrix4x4& vp, const QVector3D& camPos,
              float w, float h, QString& outJoint, double& outValue, bool& edited);
    bool release();
    bool updateHover(const QPointF& mouse, const QMatrix4x4& vp, float w, float h);

    std::vector<LineBatch> overlayLinesCopy() const;

private:
    struct Axis {
        QString   jointName;
        Kind      kind = Rotary;
        int       jointId = -1;
        bool      valid = false;
        QVector3D anchor;          // 世界锚点（hinge/slide 通用）
        QVector3D axis;            // 世界轴（单位）
        double    value = 0.0;     // 当前 qpos
        float     size = 0.12f;    // 生效世界尺寸（旋转环半径 / 平移箭头半长基准）
    };

    // 与 axis 正交的平面基（画旋转环 / 命中用）。
    static void axisPlaneBasis(const QVector3D& axis, QVector3D& e1, QVector3D& e2);
    // 某世界尺寸下该旋转环的屏幕投影半径（像素），量不了返回 -1。
    float ringRadiusPx(const QMatrix4x4& vp, const Axis& a, float size, float w, float h) const;

    std::vector<Axis> m_axes;
    bool    m_visible = false;
    bool    m_dragging = false;
    bool    m_alwaysOnTop = true;
    int     m_hovered = -1;         // 轴索引
    int     m_active = -1;
    QPointF m_lastMouse;
    double  m_screenSizePx = 160.0;

    mutable std::mutex     m_overlayMtx;
    std::vector<LineBatch> m_overlayLines;
};
