#include "DragTeachGizmo.h"

#include <mujoco/mujoco.h>

#include <QMatrix3x3>
#include <QVector2D>
#include <QVector4D>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

// ===========================================================================
// 纯几何 / 命中辅助 + 各部件尺寸常量（原 MujocoQuickItem.cpp 的 gizmo_detail）
// ===========================================================================
namespace {

// 点 p 到线段 [a,b] 的 2D 距离（像素）。
float pointSegmentDistance(const QPointF& p, const QPointF& a, const QPointF& b) {
    const QPointF ab = b - a;
    const float len2 = float(ab.x() * ab.x() + ab.y() * ab.y());
    float t = 0.0f;
    if (len2 > 1e-9f) {
        t = float(((p.x() - a.x()) * ab.x() + (p.y() - a.y()) * ab.y()) / len2);
        t = std::clamp(t, 0.0f, 1.0f);
    }
    const QPointF proj(a.x() + t * ab.x(), a.y() + t * ab.y());
    const QPointF d = p - proj;
    return std::sqrt(float(d.x() * d.x() + d.y() * d.y()));
}

// 点是否落在（凸）四边形内：屏幕空间叉积同号即在内。
bool pointInQuad(const QPointF& p, const QPointF quad[4]) {
    int pos = 0, neg = 0;
    for (int i = 0; i < 4; ++i) {
        const QPointF& a = quad[i];
        const QPointF& b = quad[(i + 1) % 4];
        const float cr = float((b.x() - a.x()) * (p.y() - a.y()) -
                               (b.y() - a.y()) * (p.x() - a.x()));
        if (cr > 0.0f)      ++pos;
        else if (cr < 0.0f) ++neg;
    }
    return pos == 0 || neg == 0;
}

// 四边形面积（像素²，鞋带公式）。用来剔除几乎侧对相机的退化投影。
float quadAreaPx(const QPointF quad[4]) {
    float a = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const QPointF& p = quad[i];
        const QPointF& q = quad[(i + 1) % 4];
        a += float(p.x() * q.y() - q.x() * p.y());
    }
    return std::abs(a) * 0.5f;
}

// 世界坐标轴向量。
QVector3D axisVector(int axis) {
    switch (axis) {
        case 0:  return QVector3D(1, 0, 0);
        case 1:  return QVector3D(0, 1, 0);
        default: return QVector3D(0, 0, 1);
    }
}

// 与 axis 正交的平面基（e1,e2），用于生成旋转环上的采样点。
void planeBasisOf(int axis, QVector3D& e1, QVector3D& e2) {
    switch (axis) {
        case 0:  e1 = QVector3D(0, 1, 0); e2 = QVector3D(0, 0, 1); break; // X
        case 1:  e1 = QVector3D(0, 0, 1); e2 = QVector3D(1, 0, 0); break; // Y
        default: e1 = QVector3D(1, 0, 0); e2 = QVector3D(0, 1, 0); break; // Z
    }
}

// ---- 手柄索引编码 ---------------------------------------------------------
constexpr int kTranslateHandleCount = 6;   // 平移箭头个数（3 轴 × 2 方向）
constexpr int kHandleRingBase       = 6;   // 第一个旋转环手柄的索引

inline int handleAxis(int handle) {
    return handle < kTranslateHandleCount ? handle / 2 : handle - kHandleRingBase;
}
// 平移箭头方向符号：同一轴内偶数索引 = 负方向，奇数索引 = 正方向。
inline float handleSign(int handle) {
    return (handle < kTranslateHandleCount && (handle % 2) == 0) ? -1.0f : 1.0f;
}

constexpr int kPlaneHandleBase  = 9;
constexpr int kPlaneHandleCount = 3;

inline int planeIndex(int handle) { return handle - kPlaneHandleBase; }
inline int planeAxisA(int handle)      { return planeIndex(handle); }
inline int planeAxisB(int handle)      { return (planeIndex(handle) + 1) % 3; }
inline int planeNormalAxis(int handle) { return (planeIndex(handle) + 2) % 3; }

// ---- 各部件尺寸（m_size 的倍率）-----------------------------------------
constexpr float kRingRadiusScale  = 0.90f;  // 旋转环半径
constexpr float kRingMinorScale   = 0.022f; // 旋转环管半径（仅实体模式用）
constexpr float kArrowStartScale  = 0.0f;   // 箭头根部：TCP 中心
constexpr float kArrowLengthScale = 0.55f;  // 箭头长度
constexpr float kArrowTipScale    = kArrowStartScale + kArrowLengthScale;
constexpr float kArrowShaftScale  = 0.035f; // 箭头杆半径（仅实体模式用）

// true = 圆环/箭头都用 mjGEOM_LINE 画（线框，像素级线宽、不受光照）。
constexpr bool  kGizmoWireframe       = true;
constexpr float kLineWidthPx          = 2.0f;   // 未选中（像素）
constexpr float kLineWidthSelPx       = 6.0f;   // 悬停 / 拖动中（像素）
constexpr float kArrowHeadScale       = 0.14f;  // 锥头长度 × size
constexpr float kArrowHeadRadiusScale = 0.056f; // 锥头底面半径 × size

constexpr float kPlaneOffsetScale = 0.28f;  // pad 中心沿两轴的偏移 × size
constexpr float kPlaneHalfScale   = 0.06f;  // pad 半边长 × size
constexpr float kPlaneMinAreaPx2  = 64.0f;  // 像素²：投影太小视为侧对相机
constexpr float kPlaneMinSinPx    = 0.05f;  // 平面拖动两屏幕方向近平行阈值（≈3°）

constexpr int   kRingSegments     = 40;     // 旋转环采样段数
constexpr int   kRingProbeSamples = 16;     // 量环投影半径的采样段数
constexpr float kHitTolPx         = 8.0f;   // 命中容差（像素）
constexpr float kTwoPi            = 6.28318530717958647692f;

} // namespace

// ===========================================================================
// DragTeachGizmo
// ===========================================================================
QPointF DragTeachGizmo::worldToScreen(const QMatrix4x4& viewProj, const QVector3D& world,
                                      float w, float h, bool* ok) {
    const QVector4D clip = viewProj * QVector4D(world, 1.0f);
    if (clip.w() <= 1e-6f) { if (ok) *ok = false; return QPointF(); }
    const float nx = clip.x() / clip.w();
    const float ny = clip.y() / clip.w();
    if (ok) *ok = true;
    return QPointF((nx * 0.5f + 0.5f) * w, (1.0f - (ny * 0.5f + 0.5f)) * h);
}

QVector3D DragTeachGizmo::axisVec(int axis) const {
    const QVector3D a = axisVector(axis);
    return m_toolAligned ? m_ori.rotatedVector(a) : a;
}

void DragTeachGizmo::planeBasis(int axis, QVector3D& e1, QVector3D& e2) const {
    planeBasisOf(axis, e1, e2);
    if (m_toolAligned) {
        e1 = m_ori.rotatedVector(e1);
        e2 = m_ori.rotatedVector(e2);
    }
}

QVector3D DragTeachGizmo::handleDir(int handle) const {
    return axisVec(handleAxis(handle)) * handleSign(handle);
}

// ------------------------------------------------------------- 状态设置 ----
bool DragTeachGizmo::setVisible(bool on) {
    if (m_visible == on) return false;
    m_visible = on;
    if (!on) {
        m_dragging = false;
        m_active = -1;
        m_hovered = -1;
    }
    return true;
}

bool DragTeachGizmo::setToolAligned(bool on) {
    if (m_toolAligned == on) return false;
    m_toolAligned = on;
    return true;
}

bool DragTeachGizmo::setTrackedSite(const QString& siteName) {
    const QString s = siteName.trimmed();
    if (s.isEmpty() || s == m_siteName) return false;
    m_siteName = s;
    m_siteId = -1;
    m_poseValid = false;
    return true;
}

void DragTeachGizmo::setWorldSize(float meters) {
    m_worldSize = std::clamp(meters, 0.01f, 2.0f);
    if (!m_constantScreenSize) m_size = m_worldSize;
}

bool DragTeachGizmo::setConstantScreenSize(bool on) {
    if (m_constantScreenSize == on) return false;
    m_constantScreenSize = on;
    return true;
}

bool DragTeachGizmo::setScreenSizePx(double px) {
    const double v = std::clamp(px, 8.0, 2000.0);
    if (qFuzzyCompare(m_screenSizePx, v)) return false;
    m_screenSizePx = v;
    return true;
}

bool DragTeachGizmo::setAlwaysOnTop(bool on) {
    if (m_alwaysOnTop == on) return false;
    m_alwaysOnTop = on;
    return true;
}

// --------------------------------------------------------------- 每帧 ----
bool DragTeachGizmo::updatePoseFromSite(const mjModel* m, const mjData* d) {
    if (!m || !d || !m_visible || m_dragging) return false;
    m_siteId = mj_name2id(m, mjOBJ_SITE, m_siteName.toUtf8().constData());
    if (m_siteId < 0 || m_siteId >= m->nsite) {
        if (m_poseValid) { m_poseValid = false; return true; }
        return false;
    }
    const mjtNum* xp = d->site_xpos + 3 * m_siteId;
    const mjtNum* xm = d->site_xmat + 9 * m_siteId;
    const QVector3D pos(static_cast<float>(xp[0]), static_cast<float>(xp[1]),
                        static_cast<float>(xp[2]));
    float rot[9];
    for (int i = 0; i < 9; ++i) rot[i] = float(xm[i]);   // site_xmat 行主序
    const QQuaternion ori = QQuaternion::fromRotationMatrix(QMatrix3x3(rot));

    const bool moved = !m_poseValid ||
                       (pos - m_pos).lengthSquared() > 1e-10f ||
                       !qFuzzyCompare(ori, m_ori);
    m_pos = pos;
    m_ori = ori;
    m_solvablePos = pos;
    m_solvableOri = ori;
    m_poseValid = true;
    return moved;
}

int DragTeachGizmo::rebuildGeoms(mjvScene* userScene, int startIndex) {
    if (!userScene || !userScene->geoms) return startIndex;
    const int maxg = userScene->maxgeom;

    if (!m_visible || !m_poseValid) {
        userScene->ngeom = std::min(startIndex, maxg);
        std::lock_guard<std::mutex> lk(m_overlayMtx);
        m_overlayLines.clear();
        return userScene->ngeom;
    }

    // 三轴基础色（X 红 / Y 绿 / Z 蓝）；hover/active 提亮。
    static const float kBase[3][3] = {
        {0.90f, 0.22f, 0.22f}, {0.25f, 0.80f, 0.28f}, {0.30f, 0.52f, 0.95f}};

    auto handleColor = [&](int handle, float out[4]) {
        const int axis = handle < kPlaneHandleBase ? handleAxis(handle) : planeNormalAxis(handle);
        float k = 1.0f;
        if (handle == m_active)       k = 1.6f;
        else if (handle == m_hovered) k = 1.28f;
        out[0] = std::min(1.0f, kBase[axis][0] * k);
        out[1] = std::min(1.0f, kBase[axis][1] * k);
        out[2] = std::min(1.0f, kBase[axis][2] * k);
        out[3] = 1.0f;
    };

    const QVector3D c = m_pos;
    const float size = m_size;
    int g = startIndex;

    auto handleLineWidth = [&](int handle) {
        return (handle == m_active || handle == m_hovered) ? kLineWidthSelPx : kLineWidthPx;
    };

    // always-on-top：线段不进 user_scn，收集到 overlay（按线宽分批）。
    std::vector<LineBatch> overlay;
    auto pushOverlay = [&](const QVector3D& p0, const QVector3D& p1,
                           const float color[4], float widthPx) {
        LineBatch* b = nullptr;
        for (auto& e : overlay)
            if (qFuzzyCompare(e.widthPx, widthPx)) { b = &e; break; }
        if (!b) { overlay.push_back(LineBatch{widthPx, {}, {}}); b = &overlay.back(); }
        b->xyz.insert(b->xyz.end(), {p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z()});
        for (int k = 0; k < 2; ++k)
            b->rgba.insert(b->rgba.end(), {color[0], color[1], color[2], color[3]});
    };

    auto addLine = [&](const QVector3D& p0, const QVector3D& p1,
                       const float color[4], float widthPx) {
        if (m_alwaysOnTop) { pushOverlay(p0, p1, color, widthPx); return; }
        if (g >= maxg) return;
        mjvGeom* geom = &userScene->geoms[g];
        mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, color);
        const mjtNum from[3] = { p0.x(), p0.y(), p0.z() };
        const mjtNum to[3]   = { p1.x(), p1.y(), p1.z() };
        mjv_connector(geom, mjGEOM_LINE, widthPx, from, to);
        ++g;
    };

    // 平移箭头（handle 0..5）：根部在 TCP 中心，沿轴向外伸。
    for (int handle = 0; handle < kTranslateHandleCount && g < maxg; ++handle) {
        const int axis = handleAxis(handle);
        const QVector3D a = handleDir(handle);
        float color[4];
        handleColor(handle, color);
        const QVector3D p0 = c + a * (size * kArrowStartScale);
        const QVector3D p1 = c + a * (size * kArrowTipScale);

        if (!kGizmoWireframe) {
            mjvGeom* geom = &userScene->geoms[g];
            mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, color);
            const mjtNum from[3] = { p0.x(), p0.y(), p0.z() };
            const mjtNum to[3]   = { p1.x(), p1.y(), p1.z() };
            mjv_connector(geom, mjGEOM_ARROW, size * kArrowShaftScale, from, to);
            ++g;
            continue;
        }

        const float lw = handleLineWidth(handle);
        const QVector3D headBase = c + a * (size * (kArrowTipScale - kArrowHeadScale));
        addLine(p0, headBase, color, lw);
        QVector3D e1, e2;
        planeBasis(axis, e1, e2);
        const float headR = size * kArrowHeadRadiusScale;
        for (int k = 0; k < 4; ++k) {
            const float t = kTwoPi * (0.125f + 0.25f * float(k));   // 45/135/225/315°
            addLine(headBase + headR * (std::cos(t) * e1 + std::sin(t) * e2), p1, color, lw);
        }
    }

    // 旋转环（handle 6..8）。
    for (int axis = 0; axis < 3; ++axis) {
        const int handle = kHandleRingBase + axis;
        float color[4];
        handleColor(handle, color);
        QVector3D e1, e2;
        planeBasis(axis, e1, e2);
        const float ringRadius = size * kRingRadiusScale;
        QVector3D prev;
        for (int i = 0; i <= kRingSegments; ++i) {
            if (g >= maxg) break;
            const float t = float(i) / float(kRingSegments) * kTwoPi;
            const QVector3D p = c + ringRadius * (std::cos(t) * e1 + std::sin(t) * e2);
            if (i > 0) {
                if (kGizmoWireframe) {
                    addLine(prev, p, color, handleLineWidth(handle));
                } else {
                    mjvGeom* geom = &userScene->geoms[g];
                    mjv_initGeom(geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, color);
                    const mjtNum from[3] = { prev.x(), prev.y(), prev.z() };
                    const mjtNum to[3]   = { p.x(),    p.y(),    p.z()    };
                    mjv_connector(geom, mjGEOM_CAPSULE, size * kRingMinorScale, from, to);
                    ++g;
                }
            }
            prev = p;
        }
    }

    // 平面拖动手柄（handle 9..11）：每块一个正方形外框。
    for (int p = 0; p < kPlaneHandleCount; ++p) {
        const int handle = kPlaneHandleBase + p;
        float color[4];
        handleColor(handle, color);
        const float lw = handleLineWidth(handle);

        const QVector3D u = axisVec(planeAxisA(handle));
        const QVector3D v = axisVec(planeAxisB(handle));
        const QVector3D padC = c + (u + v) * (size * kPlaneOffsetScale);
        const float half = size * kPlaneHalfScale;
        const QVector3D sq[4] = { padC + (u + v) * half, padC + (u - v) * half,
                                  padC - (u + v) * half, padC + (v - u) * half };
        for (int i = 0; i < 4; ++i)
            addLine(sq[i], sq[(i + 1) % 4], color, lw);
    }

    userScene->ngeom = std::min(g, maxg);

    std::lock_guard<std::mutex> lk(m_overlayMtx);
    m_overlayLines.swap(overlay);
    return userScene->ngeom;
}

float DragTeachGizmo::ringRadiusPx(const QMatrix4x4& vp, float size, float w, float h) const {
    bool okCenter = false;
    const QPointF c = worldToScreen(vp, m_pos, w, h, &okCenter);
    if (!okCenter) return -1.0f;

    const float r = size * kRingRadiusScale;
    float best = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        QVector3D e1, e2;
        planeBasis(axis, e1, e2);
        for (int i = 0; i < kRingProbeSamples; ++i) {
            const float t = float(i) / float(kRingProbeSamples) * kTwoPi;
            bool ok = false;
            const QPointF p = worldToScreen(
                vp, m_pos + r * (std::cos(t) * e1 + std::sin(t) * e2), w, h, &ok);
            if (!ok) continue;
            best = std::max(best, float(std::hypot(p.x() - c.x(), p.y() - c.y())));
        }
    }
    return best;
}

bool DragTeachGizmo::updateEffectiveSize(const QMatrix4x4& vp, const QVector3D& camPos,
                                         float w, float h) {
    if (!m_poseValid) return false;

    if (!m_constantScreenSize) {
        if (qFuzzyCompare(m_size, m_worldSize)) return false;
        m_size = m_worldSize;
        return true;
    }
    if ((m_pos - camPos).lengthSquared() < 1e-8f) return false;   // 相机就在 TCP 上

    const float targetR = float(0.5 * m_screenSizePx);
    float size = m_size;
    if (!(size > 1e-6f) || !std::isfinite(size)) size = m_worldSize;
    bool measured = false;
    for (int iter = 0; iter < 3; ++iter) {
        const float rPx = ringRadiusPx(vp, size, w, h);
        if (rPx <= 1e-4f) break;
        measured = true;
        const float next = size * (targetR / rPx);
        if (!std::isfinite(next) || next <= 0.0f) break;
        const float rel = std::abs(next - size) / std::max(next, size);
        size = next;
        if (rel < 0.002f) break;
    }
    if (!measured) return false;
    if (std::abs(size - m_size) <= 0.005f * m_size) return false;
    m_size = size;
    return true;
}

// -------------------------------------------------------------- 交互处理 ----
int DragTeachGizmo::hitTest(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) const {
    if (!m_poseValid) return -1;
    const QVector3D c = m_pos;
    const float arrowRoot = m_size * kArrowStartScale;
    const float arrowTip  = m_size * kArrowTipScale;
    const float ringR = m_size * kRingRadiusScale;

    float best = kHitTolPx;
    int bestHandle = -1;

    // 平面拖动手柄优先。
    for (int p = 0; p < kPlaneHandleCount; ++p) {
        const int handle = kPlaneHandleBase + p;
        const QVector3D u = axisVec(planeAxisA(handle));
        const QVector3D v = axisVec(planeAxisB(handle));
        const QVector3D padC = c + (u + v) * (m_size * kPlaneOffsetScale);
        const float half = m_size * kPlaneHalfScale;
        const QVector3D corner[4] = { padC + (u + v) * half, padC + (u - v) * half,
                                      padC - (u + v) * half, padC + (v - u) * half };
        QPointF quad[4];
        bool ok = true;
        for (int i = 0; i < 4 && ok; ++i)
            quad[i] = worldToScreen(vp, corner[i], w, h, &ok);
        if (ok && quadAreaPx(quad) > kPlaneMinAreaPx2 && pointInQuad(mouse, quad))
            return handle;
    }

    // 平移箭头。
    for (int handle = 0; handle < kTranslateHandleCount; ++handle) {
        bool okR = false, okT = false;
        const QVector3D dir = handleDir(handle);
        const QPointF a = worldToScreen(vp, c + dir * arrowRoot, w, h, &okR);
        const QPointF b = worldToScreen(vp, c + dir * arrowTip, w, h, &okT);
        if (!okR || !okT) continue;
        const float d = pointSegmentDistance(mouse, a, b);
        if (d < best) { best = d; bestHandle = handle; }
    }

    // 旋转环。
    for (int axis = 0; axis < 3; ++axis) {
        QVector3D e1, e2;
        planeBasis(axis, e1, e2);
        QPointF prev;
        bool havePrev = false;
        float dmin = std::numeric_limits<float>::max();
        for (int i = 0; i <= kRingSegments; ++i) {
            const float t = float(i) / float(kRingSegments) * kTwoPi;
            bool okp = false;
            const QPointF sp = worldToScreen(vp, c + ringR * (std::cos(t) * e1 + std::sin(t) * e2),
                                             w, h, &okp);
            if (okp && havePrev) dmin = std::min(dmin, pointSegmentDistance(mouse, prev, sp));
            prev = sp;
            havePrev = okp;
        }
        if (dmin < best) { best = dmin; bestHandle = kHandleRingBase + axis; }
    }
    return bestHandle;
}

bool DragTeachGizmo::press(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) {
    if (!m_visible || !m_poseValid) return false;
    const int handle = hitTest(mouse, vp, w, h);
    if (handle < 0) return false;
    m_dragging = true;
    if (handle < kTranslateHandleCount) {          // 平移箭头
        m_dragMode  = 0;
        m_dragAxis  = handleAxis(handle);
        m_dragAxisB = 0;
    } else if (handle < kPlaneHandleBase) {         // 旋转环
        m_dragMode  = 1;
        m_dragAxis  = handleAxis(handle);
        m_dragAxisB = 0;
    } else {                                         // 平面拖动
        m_dragMode  = 2;
        m_dragAxis  = planeAxisA(handle);
        m_dragAxisB = planeAxisB(handle);
    }
    m_dragSign = handleSign(handle);
    m_active = handle;
    m_hovered = handle;
    m_lastMouse = mouse;
    m_solvablePos = m_pos;
    m_solvableOri = m_ori;
    m_lastSolvable = true;
    return true;
}

bool DragTeachGizmo::move(const QPointF& mouse, const QMatrix4x4& vp, const QVector3D& camPos,
                          float w, float h, QVector3D& outPos, QQuaternion& outOri,
                          bool& poseEdited) {
    poseEdited = false;
    if (!m_dragging) return false;

    const QVector3D axisVecDrag = axisVec(m_dragAxis);
    QVector3D candPos = m_pos;
    QQuaternion candOri = m_ori;

    if (m_dragMode == 0) {
        // 平移：鼠标位移投影到屏幕上的手柄方向（含正负号），换算成世界位移。
        const QVector3D a = axisVecDrag * float(m_dragSign);
        const float r0 = m_size * kArrowStartScale;
        const float r1 = m_size * kArrowTipScale;
        const float L = r1 - r0;
        bool ok1 = false, ok2 = false;
        const QPointF s0 = worldToScreen(vp, m_pos + a * r0, w, h, &ok1);
        const QPointF s1 = worldToScreen(vp, m_pos + a * r1, w, h, &ok2);
        if (ok1 && ok2) {
            QVector2D dir(float(s1.x() - s0.x()), float(s1.y() - s0.y()));
            const float len = dir.length();
            if (len > 1e-3f) {
                dir /= len;
                const QVector2D md(float(mouse.x() - m_lastMouse.x()),
                                   float(mouse.y() - m_lastMouse.y()));
                const float screenMove = QVector2D::dotProduct(md, dir);
                candPos = m_pos + a * (screenMove * (L / len));
            }
        }
    } else if (m_dragMode == 2) {
        // 平面拖动：鼠标位移分解到平面内两个轴的屏幕方向（解 2×2 线性方程）。
        const QVector3D u = axisVec(m_dragAxis);
        const QVector3D v = axisVec(m_dragAxisB);
        bool ok0 = false, ok1 = false, ok2 = false;
        const QPointF s0 = worldToScreen(vp, m_pos, w, h, &ok0);
        const QPointF su = worldToScreen(vp, m_pos + u, w, h, &ok1);   // 每米对应多少像素
        const QPointF sv = worldToScreen(vp, m_pos + v, w, h, &ok2);
        if (ok0 && ok1 && ok2) {
            const QVector2D pu(float(su.x() - s0.x()), float(su.y() - s0.y()));
            const QVector2D pv(float(sv.x() - s0.x()), float(sv.y() - s0.y()));
            const float det   = pu.x() * pv.y() - pu.y() * pv.x();
            const float scale = pu.length() * pv.length();
            if (scale > 1e-6f && std::abs(det) / scale > kPlaneMinSinPx) {
                const float dx = float(mouse.x() - m_lastMouse.x());
                const float dy = float(mouse.y() - m_lastMouse.y());
                const float dA = (dx * pv.y() - dy * pv.x()) / det;   // 单位：米
                const float dB = (pu.x() * dy - pu.y() * dx) / det;
                candPos = m_pos + u * dA + v * dB;
            }
        }
    } else {
        // 旋转：鼠标相对屏幕投影中心的转角，方向按轴是否朝向相机翻转。
        bool ok = false;
        const QPointF cc = worldToScreen(vp, m_pos, w, h, &ok);
        if (ok) {
            const QVector2D v0(float(m_lastMouse.x() - cc.x()),
                               float(m_lastMouse.y() - cc.y()));
            const QVector2D v1(float(mouse.x() - cc.x()), float(mouse.y() - cc.y()));
            if (v0.length() > 1e-3f && v1.length() > 1e-3f) {
                const float a0 = std::atan2(v0.y(), v0.x());
                const float a1 = std::atan2(v1.y(), v1.x());
                const float facing = QVector3D::dotProduct(axisVecDrag, (m_pos - camPos));
                const float sign = (facing > 0.0f) ? 1.0f : -1.0f;
                const float deg = float(qRadiansToDegrees(double(a1 - a0))) * sign;
                candOri = QQuaternion::fromAxisAndAngle(axisVecDrag, deg) * m_ori;
            }
        }
    }

    m_pos = candPos;
    m_ori = candOri;
    m_lastMouse = mouse;
    m_lastSolvable = true;
    outPos = candPos;
    outOri = candOri;
    poseEdited = true;
    return true;
}

bool DragTeachGizmo::release() {
    if (!m_dragging) return false;
    m_dragging = false;
    m_active = -1;
    m_hovered = -1;
    if (!m_lastSolvable) {   // IK 跟不到位：弹回上一次可解位姿
        m_pos = m_solvablePos;
        m_ori = m_solvableOri;
    }
    m_lastSolvable = true;
    return true;
}

bool DragTeachGizmo::updateHover(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) {
    int want = m_hovered;
    if (!m_visible || m_dragging || !m_poseValid)
        want = -1;
    else
        want = hitTest(mouse, vp, w, h);
    if (want == m_hovered) return false;
    m_hovered = want;
    return true;
}

void DragTeachGizmo::reportSolvable(bool solvable) {
    if (!m_dragging) return;
    m_lastSolvable = solvable;
    if (solvable) {
        m_solvablePos = m_pos;
        m_solvableOri = m_ori;
    }
}

std::vector<DragTeachGizmo::LineBatch> DragTeachGizmo::overlayLinesCopy() const {
    std::lock_guard<std::mutex> lk(m_overlayMtx);
    return m_overlayLines;
}
