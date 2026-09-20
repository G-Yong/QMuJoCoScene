#pragma once
// ---------------------------------------------------------------------------
// DragTeachGizmo
//
// 从 MujocoQuickItem 里剥离出来的“拖动示教 gizmo”：一个 rviz 风格的 6-DoF 交互
// 手柄（3 个旋转环 + 6 根平移箭头 + 3 个平面拖动 pad），叠加在 TCP（site）上。
// 本类只负责 gizmo 自身的**状态 + 几何 + 命中 + 拖动数学**，是个纯 C++ 类
// （不是 QObject，仿照 PointCloudRenderer）。它**不碰** mjModel/mjData 的锁、
// 不建相机、不发信号——这些由宿主 MujocoQuickItem 负责：
//   - 宿主持 sim.mtx 后再调用本类的 updatePoseFromSite / rebuildGeoms /
//     updateEffectiveSize / hitTest / press / move / release / updateHover；
//   - 相机矩阵（viewProj + camPos + 视口宽高）由宿主用 scn 相机构造后传进来；
//   - 拖动产生的目标位姿通过 move() 的出参回给宿主去做 IK / 发 gizmoPoseEdited。
//
// 手柄索引编码（渲染 / 命中 / 拖动三处共用）：
//   0..5  平移箭头，每轴一对、先负后正：0=-X 1=+X 2=-Y 3=+Y 4=-Z 5=+Z
//   6..8  旋转环 X/Y/Z
//   9..11 平面拖动 XY/YZ/ZX（两轴联动）
// ---------------------------------------------------------------------------

#include <QMatrix4x4>
#include <QPointF>
#include <QQuaternion>
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

class DragTeachGizmo {
public:
    // always-on-top 叠加层的一批同线宽线段（每段 2 个端点）。
    struct LineBatch {
        float              widthPx = 1.0f;
        std::vector<float> xyz;   // 每段 6 个 float
        std::vector<float> rgba;  // 每段 8 个 float
    };

    DragTeachGizmo() = default;

    // 世界坐标 → 屏幕像素（item 逻辑像素，y 向下）。ok=false 表示点在相机背后。
    // TCP 坐标读数也用它，所以做成公有静态。
    static QPointF worldToScreen(const QMatrix4x4& viewProj, const QVector3D& world,
                                 float w, float h, bool* ok);

    // ---- 状态查询 ---------------------------------------------------------
    bool        visible() const { return m_visible; }
    bool        dragging() const { return m_dragging; }
    bool        poseValid() const { return m_poseValid; }
    bool        toolAligned() const { return m_toolAligned; }
    QVector3D   pos() const { return m_pos; }
    QQuaternion ori() const { return m_ori; }
    QString     trackedSite() const { return m_siteName; }
    double      worldSize() const { return m_worldSize; }
    bool        constantScreenSize() const { return m_constantScreenSize; }
    double      screenSizePx() const { return m_screenSizePx; }
    bool        alwaysOnTop() const { return m_alwaysOnTop; }

    // ---- 状态设置（只改字段；重建 geom / 发信号由宿主负责）----------------
    // setter 返回是否真的变了，宿主据此决定要不要重建 / 发信号。
    bool setVisible(bool on);            // 隐藏时顺带清 dragging/active/hovered
    bool setToolAligned(bool on);
    bool setTrackedSite(const QString& siteName);  // 清 siteId / poseValid
    void setWorldSize(float meters);     // clamp；非恒定屏幕尺寸时同步生效尺寸
    bool setConstantScreenSize(bool on);
    bool setScreenSizePx(double px);
    bool setAlwaysOnTop(bool on);
    void setLastMouse(const QPointF& p) { m_lastMouse = p; }

    // ---- 每帧（宿主持 sim.mtx 后调用）------------------------------------
    // 按 site 世界位姿刷新 gizmo 位姿（仅可见且非拖动时）。返回位姿是否变化。
    bool updatePoseFromSite(const mjModel* m, const mjData* d);
    // 在 [startIndex, ...) 处写 gizmo 的 user_scn geom；always-on-top 时改为收集到
    // 叠加层线缓冲（不写 user_scn）。返回写入后 user_scn 应有的 ngeom。
    int  rebuildGeoms(mjvScene* userScene, int startIndex);
    // 恒定屏幕尺寸模式：按当前相机反解生效世界尺寸。返回生效尺寸是否变化。
    bool updateEffectiveSize(const QMatrix4x4& vp, const QVector3D& camPos, float w, float h);

    // ---- 交互（宿主持 sim.mtx + 传入相机后调用）--------------------------
    // 命中测试：返回手柄索引，未命中 -1。
    int  hitTest(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) const;
    // 按下：命中手柄则进入拖动并返回 true。
    bool press(const QPointF& mouse, const QMatrix4x4& vp, float w, float h);
    // 拖动移动：内部按 dragMode 解算目标位姿并写回 m_pos/m_ori。返回是否处于拖动
    //（true = 已消费）；poseEdited=true 时 outPos/outOri 有效，宿主需要 emit + IK。
    bool move(const QPointF& mouse, const QMatrix4x4& vp, const QVector3D& camPos,
              float w, float h, QVector3D& outPos, QQuaternion& outOri, bool& poseEdited);
    // 松手：返回此前是否在拖动。IK 不可解时把位姿弹回上一次可解值。
    bool release();
    // 悬停：更新高亮手柄，返回是否变化。
    bool updateHover(const QPointF& mouse, const QMatrix4x4& vp, float w, float h);
    // 宿主在 gizmoPoseEdited 槽里回告本次目标位姿 IK 是否可解。
    void reportSolvable(bool solvable);

    // ---- always-on-top 叠加层（渲染线程读）------------------------------
    // 线程安全地拷走当前叠加层线段（内部 mutex，不走 sim.mtx）。
    std::vector<LineBatch> overlayLinesCopy() const;

private:
    // 按对齐模式返回世界轴向量 / 旋转环平面基 / 带符号的手柄方向。
    QVector3D axisVec(int axis) const;
    void      planeBasis(int axis, QVector3D& e1, QVector3D& e2) const;
    QVector3D handleDir(int handle) const;
    // 量某世界尺寸下三个旋转环里最大的屏幕投影半径（像素），量不了返回 -1。
    float     ringRadiusPx(const QMatrix4x4& vp, float size, float w, float h) const;

    // 状态（受宿主 sim.mtx 保护）。
    bool        m_visible = false;
    QString     m_siteName = QStringLiteral("tcp");
    int         m_siteId = -1;
    bool        m_poseValid = false;
    QVector3D   m_pos;                 // 世界位置（米）
    QQuaternion m_ori;                 // 世界姿态
    float       m_size = 0.12f;        // 生效世界尺寸（恒定屏幕尺寸模式下每帧换算）
    int         m_hovered = -1;
    int         m_active = -1;
    bool        m_dragging = false;
    int         m_dragMode = 0;        // 0=平移 1=旋转 2=平面
    int         m_dragAxis = 0;
    int         m_dragAxisB = 0;
    int         m_dragSign = 1;
    bool        m_toolAligned = false;
    QPointF     m_lastMouse;
    QVector3D   m_solvablePos;         // 最近一次 IK 可解的位姿
    QQuaternion m_solvableOri;
    bool        m_lastSolvable = true;

    // 尺寸参数：m_worldSize 是手动固定世界尺寸（关掉恒定屏幕尺寸时生效）。
    float       m_worldSize = 0.12f;
    bool        m_constantScreenSize = true;
    double      m_screenSizePx = 320.0;
    bool        m_alwaysOnTop = true;

    // always-on-top 叠加层线段：rebuildGeoms 在 sim.mtx 内填、overlayLinesCopy 在
    // m_overlayMtx 内读（与点云同一套跨线程约定，不走 sim.mtx）。
    mutable std::mutex     m_overlayMtx;
    std::vector<LineBatch> m_overlayLines;
};
