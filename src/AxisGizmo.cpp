#include "AxisGizmo.h"
#include "DragTeachGizmo.h"   // 复用 worldToScreen

#include <mujoco/mujoco.h>

#include <QVector2D>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

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

// 附加轴 gizmo 的外观常量（与 TCP gizmo 同口径，但用不同色系区分）。
constexpr float kRotaryRingScale   = 0.90f;  // 旋转环半径 × size
constexpr float kPrismHalfScale    = 0.90f;  // 平移箭头半长 × size
constexpr float kArrowHeadScale    = 0.16f;  // 箭头锥头长度 × size
constexpr float kArrowHeadRScale   = 0.06f;  // 箭头锥头底半径 × size
constexpr float kLineWidthPx       = 2.0f;
constexpr float kLineWidthSelPx    = 6.0f;
constexpr int   kRingSegments      = 40;
constexpr int   kRingProbeSamples  = 16;
constexpr float kHitTolPx          = 8.0f;
constexpr float kTwoPi             = 6.28318530717958647692f;

// 旋转轴：橙黄；平移轴：青。hover/active 提亮。
void axisColor(AxisGizmo::Kind kind, float k, float out[4]) {
    const float base[2][3] = {{1.00f, 0.62f, 0.10f}, {0.15f, 0.85f, 0.85f}};
    const int i = (kind == AxisGizmo::Prismatic) ? 1 : 0;
    out[0] = std::min(1.0f, base[i][0] * k);
    out[1] = std::min(1.0f, base[i][1] * k);
    out[2] = std::min(1.0f, base[i][2] * k);
    out[3] = 1.0f;
}

} // namespace

void AxisGizmo::axisPlaneBasis(const QVector3D& axis, QVector3D& e1, QVector3D& e2) {
    QVector3D n = axis;
    if (n.lengthSquared() < 1e-12f) n = QVector3D(0, 0, 1);
    n.normalize();
    // 取一个与 n 不平行的参考向量做叉积，得到平面基。
    QVector3D ref = std::abs(n.z()) < 0.9f ? QVector3D(0, 0, 1) : QVector3D(1, 0, 0);
    e1 = QVector3D::crossProduct(n, ref);
    if (e1.lengthSquared() < 1e-12f) e1 = QVector3D(1, 0, 0);
    e1.normalize();
    e2 = QVector3D::crossProduct(n, e1);
    e2.normalize();
}

void AxisGizmo::setJointNames(const std::vector<QString>& names) {
    m_axes.clear();
    m_axes.reserve(names.size());
    for (const QString& n : names) {
        if (n.trimmed().isEmpty()) continue;
        Axis a;
        a.jointName = n.trimmed();
        m_axes.push_back(a);
    }
    m_dragging = false;
    m_hovered = m_active = -1;
    std::lock_guard<std::mutex> lk(m_overlayMtx);
    m_overlayLines.clear();
}

bool AxisGizmo::setVisible(bool on) {
    if (m_visible == on) return false;
    m_visible = on;
    if (!on) { m_dragging = false; m_active = -1; m_hovered = -1; }
    return true;
}

bool AxisGizmo::setAlwaysOnTop(bool on) {
    if (m_alwaysOnTop == on) return false;
    m_alwaysOnTop = on;
    return true;
}

bool AxisGizmo::updateFromData(const mjModel* m, const mjData* d) {
    if (!m || !d) return false;
    bool changed = false;
    for (Axis& a : m_axes) {
        const int id = mj_name2id(m, mjOBJ_JOINT, a.jointName.toUtf8().constData());
        const bool wasValid = a.valid;
        a.jointId = id;
        if (id < 0 || id >= m->njnt) { a.valid = false; if (wasValid) changed = true; continue; }
        const int type = m->jnt_type[id];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
            a.valid = false; if (wasValid) changed = true; continue;
        }
        a.kind = (type == mjJNT_SLIDE) ? Prismatic : Rotary;
        const QVector3D anchor(float(d->xanchor[3 * id + 0]),
                               float(d->xanchor[3 * id + 1]),
                               float(d->xanchor[3 * id + 2]));
        QVector3D axis(float(d->xaxis[3 * id + 0]),
                       float(d->xaxis[3 * id + 1]),
                       float(d->xaxis[3 * id + 2]));
        if (axis.lengthSquared() > 1e-12f) axis.normalize();
        const double value = d->qpos[m->jnt_qposadr[id]];
        if (!wasValid ||
            (anchor - a.anchor).lengthSquared() > 1e-10f ||
            (axis - a.axis).lengthSquared() > 1e-10f ||
            std::abs(value - a.value) > 1e-9) {
            changed = true;
        }
        a.anchor = anchor;
        a.axis = axis;
        a.value = value;
        a.valid = true;
    }
    return changed;
}

float AxisGizmo::ringRadiusPx(const QMatrix4x4& vp, const Axis& a, float size,
                              float w, float h) const {
    bool okC = false;
    const QPointF c = DragTeachGizmo::worldToScreen(vp, a.anchor, w, h, &okC);
    if (!okC) return -1.0f;
    QVector3D e1, e2;
    axisPlaneBasis(a.axis, e1, e2);
    const float r = size * kRotaryRingScale;
    float best = 0.0f;
    for (int i = 0; i < kRingProbeSamples; ++i) {
        const float t = float(i) / float(kRingProbeSamples) * kTwoPi;
        bool ok = false;
        const QPointF p = DragTeachGizmo::worldToScreen(
            vp, a.anchor + r * (std::cos(t) * e1 + std::sin(t) * e2), w, h, &ok);
        if (!ok) continue;
        best = std::max(best, float(std::hypot(p.x() - c.x(), p.y() - c.y())));
    }
    return best;
}

bool AxisGizmo::updateEffectiveSizes(const QMatrix4x4& vp, const QVector3D& camPos,
                                     float w, float h) {
    bool changed = false;
    const float targetR = float(0.5 * m_screenSizePx);
    for (Axis& a : m_axes) {
        if (!a.valid) continue;
        if ((a.anchor - camPos).lengthSquared() < 1e-8f) continue;
        float size = a.size;
        if (!(size > 1e-6f) || !std::isfinite(size)) size = 0.12f;
        bool measured = false;
        for (int iter = 0; iter < 3; ++iter) {
            const float rPx = ringRadiusPx(vp, a, size, w, h);
            if (rPx <= 1e-4f) break;
            measured = true;
            const float next = size * (targetR / rPx);
            if (!std::isfinite(next) || next <= 0.0f) break;
            const float rel = std::abs(next - size) / std::max(next, size);
            size = next;
            if (rel < 0.002f) break;
        }
        if (measured && std::abs(size - a.size) > 0.005f * a.size) {
            a.size = size;
            changed = true;
        }
    }
    return changed;
}

int AxisGizmo::rebuildGeoms(mjvScene* userScene, int startIndex) {
    if (!userScene || !userScene->geoms) return startIndex;
    const int maxg = userScene->maxgeom;
    int g = startIndex;

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

    if (m_visible) {
        for (int idx = 0; idx < int(m_axes.size()); ++idx) {
            const Axis& a = m_axes[idx];
            if (!a.valid) continue;
            const float k = (idx == m_active) ? 1.6f : (idx == m_hovered ? 1.28f : 1.0f);
            const float lw = (idx == m_active || idx == m_hovered) ? kLineWidthSelPx : kLineWidthPx;
            float color[4];
            axisColor(a.kind, k, color);

            if (a.kind == Prismatic) {
                // 平移轴：沿轴的双向箭头（预留，拖动改 qpos 距离）。
                const float half = a.size * kPrismHalfScale;
                const QVector3D p0 = a.anchor - a.axis * half;
                const QVector3D p1 = a.anchor + a.axis * half;
                addLine(p0, p1, color, lw);
                QVector3D e1, e2;
                axisPlaneBasis(a.axis, e1, e2);
                const float hs = a.size * kArrowHeadScale;
                const float hr = a.size * kArrowHeadRScale;
                for (int s = -1; s <= 1; s += 2) {
                    const QVector3D tip = a.anchor + a.axis * (half * float(s));
                    const QVector3D base = tip - a.axis * (hs * float(s));
                    for (int q = 0; q < 4; ++q) {
                        const float t = kTwoPi * (0.125f + 0.25f * float(q));
                        addLine(base + hr * (std::cos(t) * e1 + std::sin(t) * e2), tip, color, lw);
                    }
                }
            } else {
                // 旋转轴：轴平面内的环。
                QVector3D e1, e2;
                axisPlaneBasis(a.axis, e1, e2);
                const float r = a.size * kRotaryRingScale;
                QVector3D prev;
                for (int i = 0; i <= kRingSegments; ++i) {
                    const float t = float(i) / float(kRingSegments) * kTwoPi;
                    const QVector3D p = a.anchor + r * (std::cos(t) * e1 + std::sin(t) * e2);
                    if (i > 0) addLine(prev, p, color, lw);
                    prev = p;
                }
            }
        }
    }

    if (!m_alwaysOnTop) userScene->ngeom = std::min(g, maxg);
    std::lock_guard<std::mutex> lk(m_overlayMtx);
    m_overlayLines.swap(overlay);
    return m_alwaysOnTop ? startIndex : std::min(g, maxg);
}

int AxisGizmo::hitTest(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) const {
    if (!m_visible) return -1;
    float best = kHitTolPx;
    int bestAxis = -1;
    for (int idx = 0; idx < int(m_axes.size()); ++idx) {
        const Axis& a = m_axes[idx];
        if (!a.valid) continue;

        if (a.kind == Prismatic) {
            const float half = a.size * kPrismHalfScale;
            bool ok0 = false, ok1 = false;
            const QPointF s0 = DragTeachGizmo::worldToScreen(vp, a.anchor - a.axis * half, w, h, &ok0);
            const QPointF s1 = DragTeachGizmo::worldToScreen(vp, a.anchor + a.axis * half, w, h, &ok1);
            if (ok0 && ok1) {
                const float d = pointSegmentDistance(mouse, s0, s1);
                if (d < best) { best = d; bestAxis = idx; }
            }
        } else {
            QVector3D e1, e2;
            axisPlaneBasis(a.axis, e1, e2);
            const float r = a.size * kRotaryRingScale;
            QPointF prev;
            bool havePrev = false;
            float dmin = std::numeric_limits<float>::max();
            for (int i = 0; i <= kRingSegments; ++i) {
                const float t = float(i) / float(kRingSegments) * kTwoPi;
                bool okp = false;
                const QPointF sp = DragTeachGizmo::worldToScreen(
                    vp, a.anchor + r * (std::cos(t) * e1 + std::sin(t) * e2), w, h, &okp);
                if (okp && havePrev) dmin = std::min(dmin, pointSegmentDistance(mouse, prev, sp));
                prev = sp;
                havePrev = okp;
            }
            if (dmin < best) { best = dmin; bestAxis = idx; }
        }
    }
    return bestAxis;
}

bool AxisGizmo::press(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) {
    if (!m_visible) return false;
    const int idx = hitTest(mouse, vp, w, h);
    if (idx < 0) return false;
    m_dragging = true;
    m_active = idx;
    m_hovered = idx;
    m_lastMouse = mouse;
    return true;
}

bool AxisGizmo::move(const QPointF& mouse, const QMatrix4x4& vp, const QVector3D& camPos,
                     float w, float h, QString& outJoint, double& outValue, bool& edited) {
    edited = false;
    if (!m_dragging || m_active < 0 || m_active >= int(m_axes.size())) return m_dragging;
    Axis& a = m_axes[m_active];
    if (!a.valid) { m_lastMouse = mouse; return true; }

    if (a.kind == Prismatic) {
        // 平移：鼠标位移投影到屏幕上的轴方向 → 每米像素 → 世界位移量。
        bool ok0 = false, ok1 = false;
        const QPointF s0 = DragTeachGizmo::worldToScreen(vp, a.anchor, w, h, &ok0);
        const QPointF s1 = DragTeachGizmo::worldToScreen(vp, a.anchor + a.axis, w, h, &ok1);
        if (ok0 && ok1) {
            QVector2D dir(float(s1.x() - s0.x()), float(s1.y() - s0.y()));
            const float len = dir.length();
            if (len > 1e-3f) {
                dir /= len;
                const QVector2D md(float(mouse.x() - m_lastMouse.x()),
                                   float(mouse.y() - m_lastMouse.y()));
                const float screenMove = QVector2D::dotProduct(md, dir);
                a.value += double(screenMove / len);   // len = 每米像素
                edited = true;
            }
        }
    } else {
        // 旋转：鼠标绕投影锚点的转角，方向按轴是否朝向相机翻转 → 增量弧度。
        bool ok = false;
        const QPointF cc = DragTeachGizmo::worldToScreen(vp, a.anchor, w, h, &ok);
        if (ok) {
            const QVector2D v0(float(m_lastMouse.x() - cc.x()), float(m_lastMouse.y() - cc.y()));
            const QVector2D v1(float(mouse.x() - cc.x()), float(mouse.y() - cc.y()));
            if (v0.length() > 1e-3f && v1.length() > 1e-3f) {
                const float a0 = std::atan2(v0.y(), v0.x());
                const float a1 = std::atan2(v1.y(), v1.x());
                const float facing = QVector3D::dotProduct(a.axis, (a.anchor - camPos));
                const float sign = (facing > 0.0f) ? 1.0f : -1.0f;
                a.value += double(a1 - a0) * double(sign);
                edited = true;
            }
        }
    }

    m_lastMouse = mouse;
    outJoint = a.jointName;
    outValue = a.value;
    return true;
}

bool AxisGizmo::release() {
    if (!m_dragging) return false;
    m_dragging = false;
    m_active = -1;
    m_hovered = -1;
    return true;
}

bool AxisGizmo::updateHover(const QPointF& mouse, const QMatrix4x4& vp, float w, float h) {
    int want = -1;
    if (m_visible && !m_dragging) want = hitTest(mouse, vp, w, h);
    if (want == m_hovered) return false;
    m_hovered = want;
    return true;
}

std::vector<AxisGizmo::LineBatch> AxisGizmo::overlayLinesCopy() const {
    std::lock_guard<std::mutex> lk(m_overlayMtx);
    return m_overlayLines;
}
