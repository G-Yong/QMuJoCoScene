#include "SurfaceSnapper.h"

#include <mujoco/mujoco.h>

#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>

// ===========================================================================
// 局部系最近点（各 geom 类型）
// ===========================================================================
namespace {

constexpr float kTiny = 1e-9f;

// 点 p 到三角形 abc 的最近点（Ericson, Real-Time Collision Detection 5.1.5）。
QVector3D closestOnTriangle(const QVector3D& p, const QVector3D& a,
                           const QVector3D& b, const QVector3D& c) {
    const QVector3D ab = b - a;
    const QVector3D ac = c - a;
    const QVector3D ap = p - a;

    const float d1 = QVector3D::dotProduct(ab, ap);
    const float d2 = QVector3D::dotProduct(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;                 // 顶点区 a

    const QVector3D bp = p - b;
    const float d3 = QVector3D::dotProduct(ab, bp);
    const float d4 = QVector3D::dotProduct(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;                   // 顶点区 b

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {           // 边 ab
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const QVector3D cp = p - c;
    const float d5 = QVector3D::dotProduct(ab, cp);
    const float d6 = QVector3D::dotProduct(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;                   // 顶点区 c

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {           // 边 ac
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {   // 边 bc
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    // 面内
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

// 椭球最近点：解 λ ≥ 0 使 Σ (p_i·a_i / (a_i²+λ))² = 1，再代回 x_i = p_i·a_i²/(a_i²+λ)。
// 外部点二分即可；内部点按方向缩放到表面。
QVector3D closestOnEllipsoid(const QVector3D& p, float a, float b, float c) {
    const double A = a, B = b, C = c;
    const double px = p.x(), py = p.y(), pz = p.z();

    auto f = [&](double lam) {
        const double x = px * A / (A * A + lam);
        const double y = py * B / (B * B + lam);
        const double z = pz * C / (C * C + lam);
        return x * x + y * y + z * z - 1.0;
    };

    if (f(0.0) <= 0.0) {
        // 内部（或正好在面上）：按方向缩放到表面
        const float qx = p.x() / a, qy = p.y() / b, qz = p.z() / c;
        const float len = std::sqrt(qx * qx + qy * qy + qz * qz);
        if (len < kTiny) return QVector3D(a, 0.0f, 0.0f);
        return QVector3D(qx / len * a, qy / len * b, qz / len * c);
    }

    double hi = 1.0;
    for (int i = 0; i < 80 && f(hi) > 0.0; ++i) hi *= 2.0;
    double lo = 0.0;
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (f(mid) > 0.0) lo = mid; else hi = mid;
        if (hi - lo < 1e-12 * hi) break;
    }
    const double lam = 0.5 * (lo + hi);
    return QVector3D(float(px * A * A / (A * A + lam)),
                     float(py * B * B / (B * B + lam)),
                     float(pz * C * C / (C * C + lam)));
}

// geom 局部系里求最近点。返回 false 表示该类型不支持。
bool closestPointLocal(int type, const QVector3D& p, const float* size,
                       const mjModel* m, int dataId, QVector3D& out) {
    switch (type) {
    case mjGEOM_PLANE:
        out = QVector3D(p.x(), p.y(), 0.0f);
        return true;

    case mjGEOM_SPHERE: {
        const float r = size[0];
        if (p.lengthSquared() < kTiny) { out = QVector3D(0.0f, 0.0f, r); return true; }
        out = p.normalized() * r;
        return true;
    }

    case mjGEOM_CAPSULE: {
        const float r = size[0], h = size[1];
        const float z = std::clamp(p.z(), -h, h);
        const QVector3D axis(0.0f, 0.0f, z);
        QVector3D v = p - axis;
        if (v.lengthSquared() < kTiny) v = QVector3D(1.0f, 0.0f, 0.0f);
        out = axis + v.normalized() * r;
        return true;
    }

    case mjGEOM_CYLINDER: {
        const float r = size[0], h = size[1];
        const float rho = std::sqrt(p.x() * p.x() + p.y() * p.y());
        const bool insideR = rho <= r;
        const bool insideZ = std::abs(p.z()) <= h;
        QVector3D dir(1.0f, 0.0f, 0.0f);
        if (rho > kTiny) dir = QVector3D(p.x() / rho, p.y() / rho, 0.0f);
        const float zCap = (p.z() >= 0.0f) ? h : -h;
        if (insideR && insideZ) {
            // 内部：取更近的那个面（侧面 or 端面）
            if (r - rho <= h - std::abs(p.z()))
                out = QVector3D(dir.x() * r, dir.y() * r, p.z());
            else
                out = QVector3D(p.x(), p.y(), zCap);
        } else if (!insideR && !insideZ) {
            // 侧棱
            out = QVector3D(dir.x() * r, dir.y() * r, zCap);
        } else if (!insideR) {
            out = QVector3D(dir.x() * r, dir.y() * r, p.z());     // 侧面
        } else {
            out = QVector3D(p.x(), p.y(), zCap);                 // 端面
        }
        return true;
    }

    case mjGEOM_BOX: {
        const QVector3D e(size[0], size[1], size[2]);
        const QVector3D clamped(std::clamp(p.x(), -e.x(), e.x()),
                                std::clamp(p.y(), -e.y(), e.y()),
                                std::clamp(p.z(), -e.z(), e.z()));
        if (std::abs(clamped.x() - p.x()) > kTiny ||
            std::abs(clamped.y() - p.y()) > kTiny ||
            std::abs(clamped.z() - p.z()) > kTiny) {
            out = clamped;                                       // 外部：夹取即最近点
            return true;
        }
        // 内部：取最近的一个面
        const float dx = e.x() - std::abs(p.x());
        const float dy = e.y() - std::abs(p.y());
        const float dz = e.z() - std::abs(p.z());
        out = p;
        if (dx <= dy && dx <= dz)      out.setX(p.x() >= 0.0f ? e.x() : -e.x());
        else if (dy <= dz)             out.setY(p.y() >= 0.0f ? e.y() : -e.y());
        else                           out.setZ(p.z() >= 0.0f ? e.z() : -e.z());
        return true;
    }

    case mjGEOM_ELLIPSOID: {
        const float a = size[0], b = size[1], c = size[2];
        if (!(a > 0.0f) || !(b > 0.0f) || !(c > 0.0f)) return false;
        out = closestOnEllipsoid(p, a, b, c);
        return true;
    }

    case mjGEOM_MESH: {
        if (!m || dataId < 0 || dataId >= m->nmesh) return false;
        const int fadr = m->mesh_faceadr[dataId];
        const int fnum = m->mesh_facenum[dataId];
        const int vadr = m->mesh_vertadr[dataId];
        if (fnum <= 0 || !m->mesh_face || !m->mesh_vert) return false;

        float best2 = std::numeric_limits<float>::max();
        QVector3D best;
        bool found = false;
        for (int f = 0; f < fnum; ++f) {
            const int* idx = m->mesh_face + 3 * (fadr + f);
            QVector3D v[3];
            for (int k = 0; k < 3; ++k) {
                const float* vp = m->mesh_vert + 3 * (vadr + idx[k]);
                v[k] = QVector3D(vp[0], vp[1], vp[2]);
            }
            const QVector3D cand = closestOnTriangle(p, v[0], v[1], v[2]);
            const float d2 = (cand - p).lengthSquared();
            if (d2 < best2) {
                best2 = d2;
                best = cand;
                found = true;
                if (best2 <= 0.0f) break;
            }
        }
        if (!found) return false;
        out = best;
        return true;
    }

    default:
        return false;   // hfield / flex / sdf 等：不支持
    }
}

} // namespace

// ===========================================================================
// SurfaceSnapper
// ===========================================================================
void SurfaceSnapper::setThresholdM(float absorb, float release) {
    m_absorbM = std::max(absorb, 1e-6f);
    m_releaseM = std::max(release, m_absorbM);
}

void SurfaceSnapper::reset() {
    m_snapped = false;
}

void SurfaceSnapper::clearExclusions() {
    m_exclModel = nullptr;
    m_exclSiteBody = -1;
    m_exclNbody = 0;
    m_excluded.clear();
}

void SurfaceSnapper::setTrackedBody(const mjModel* m, int siteBodyId) {
    if (!m || siteBodyId < 0 || siteBodyId >= m->nbody) {
        clearExclusions();
        return;
    }
    if (m == m_exclModel && siteBodyId == m_exclSiteBody && m_exclNbody == m->nbody)
        return;   // 缓存命中

    m_exclModel = m;
    m_exclSiteBody = siteBodyId;
    m_exclNbody = m->nbody;
    m_excluded.assign(size_t(m->nbody), 0);

    // 沿 parent 链往上找「世界下第一级祖先」= 机器人基座。
    int root = siteBodyId;
    while (root > 0 && m->body_parentid[root] > 0) root = m->body_parentid[root];
    if (root <= 0) return;   // 直接挂在 world 下：不排除任何 body

    for (int b = 1; b < m->nbody; ++b) {
        int cur = b;
        while (cur > 0) {
            if (cur == root) { m_excluded[size_t(b)] = 1; break; }
            cur = m->body_parentid[cur];
        }
    }
}

bool SurfaceSnapper::computeNearest(const mjModel* m, const mjData* d,
                                    const QVector3D& p, float maxDist, Hit& out) const {
    out = Hit();
    if (!m || !d || m->ngeom <= 0) return false;

    const bool haveExclusion = (m_exclModel == m && int(m_excluded.size()) == m->nbody);
    const float cull = std::max(maxDist, 0.0f);   // 粗筛上限（∞ = 不限）

    float best2 = std::numeric_limits<float>::max();
    QVector3D bestPoint;
    int bestGeom = -1;

    for (int g = 0; g < m->ngeom; ++g) {
        const int bid = m->geom_bodyid[g];
        if (haveExclusion && bid >= 0 && bid < int(m_excluded.size()) && m_excluded[size_t(bid)])
            continue;

        // 包围球粗筛
        const QVector3D gc(static_cast<float>(d->geom_xpos[3 * g]),
                           static_cast<float>(d->geom_xpos[3 * g + 1]),
                           static_cast<float>(d->geom_xpos[3 * g + 2]));
        const float rb = float(m->geom_rbound[g]);
        if ((p - gc).length() - rb > cull) continue;
        // 动态剪枝：已经找到更近的点时，包围球下限都超过它的可以直接跳过。
        if (bestGeom >= 0 && (p - gc).length() - rb > std::sqrt(best2)) continue;

        // 局部系：world = gc + col0*x + col1*y + col2*z（geom_xmat 行主序）
        const mjtNum* xm = d->geom_xmat + 9 * g;
        const QVector3D col0(static_cast<float>(xm[0]), static_cast<float>(xm[3]),
                             static_cast<float>(xm[6]));
        const QVector3D col1(static_cast<float>(xm[1]), static_cast<float>(xm[4]),
                             static_cast<float>(xm[7]));
        const QVector3D col2(static_cast<float>(xm[2]), static_cast<float>(xm[5]),
                             static_cast<float>(xm[8]));
        const QVector3D rel = p - gc;
        const QVector3D local(QVector3D::dotProduct(col0, rel),
                              QVector3D::dotProduct(col1, rel),
                              QVector3D::dotProduct(col2, rel));

        // geom_size 是 mjtNum（double），这里转成 float 给局部系求最近点用。
        const float sz[3] = { float(m->geom_size[3 * g]),
                              float(m->geom_size[3 * g + 1]),
                              float(m->geom_size[3 * g + 2]) };
        QVector3D cl;
        if (!closestPointLocal(m->geom_type[g], local, sz, m, m->geom_dataid[g], cl))
            continue;

        const QVector3D world = gc + col0 * cl.x() + col1 * cl.y() + col2 * cl.z();
        const float d2 = (world - p).lengthSquared();
        if (d2 < best2) {
            best2 = d2;
            bestPoint = world;
            bestGeom = g;
        }
    }

    if (bestGeom < 0) return false;

    out.geomId = bestGeom;
    out.distance = std::sqrt(best2);
    // 已经贴着（或点在内部）就不动位置，避免把 TCP 拉向物体内壁。
    out.point = (out.distance < 0.001f) ? p : bestPoint;
    return true;
}

bool SurfaceSnapper::nearestPoint(const mjModel* m, const mjData* d,
                                  const QVector3D& p, Hit& out) const {
    return computeNearest(m, d, p, std::numeric_limits<float>::max(), out);
}

bool SurfaceSnapper::query(const mjModel* m, const mjData* d, const QVector3D& p, Hit& out) {
    out = Hit();
    if (!m_enabled) {
        reset();
        return false;
    }
    Hit nearest;
    if (!computeNearest(m, d, p, m_releaseM, nearest)) {
        reset();
        return false;
    }

    if (m_snapped) {
        if (nearest.distance > m_releaseM) {   // 拖远了：脱开
            reset();
            return false;
        }
    } else {
        if (nearest.distance > m_absorbM)      // 还没进入吸附范围
            return false;
        m_snapped = true;
    }

    out = nearest;
    return true;
}

// ===========================================================================
// 自检
// ===========================================================================
QString SurfaceSnapper::selfTest() {
    // 内联 MJCF：各类型各一个 geom，互不重叠，便于分别断言。
    static const char* kXml = R"XML(
<mujoco model="snaptest">
  <asset>
    <mesh name="cube"
      vertex="-0.2 -0.2 -0.2  0.2 -0.2 -0.2  0.2 0.2 -0.2  -0.2 0.2 -0.2
              -0.2 -0.2  0.2  0.2 -0.2  0.2  0.2 0.2  0.2  -0.2 0.2  0.2"
      face="0 1 2  0 2 3  4 5 6  4 6 7  0 1 5  0 5 4
            1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1" pos="0 0 0"/>
    <body name="boxBody" pos="1 0 1"><geom name="boxGeom" type="box" size="0.1 0.1 0.1"/></body>
    <body name="sphBody" pos="2 0 1"><geom name="sphGeom" type="sphere" size="0.2"/></body>
    <body name="cylBody" pos="3 0 1"><geom name="cylGeom" type="cylinder" size="0.1 0.2"/></body>
    <body name="capBody" pos="4 0 1"><geom name="capGeom" type="capsule" size="0.1 0.2"/></body>
    <body name="ellBody" pos="0 2 1"><geom name="ellGeom" type="ellipsoid" size="0.2 0.1 0.3"/></body>
    <body name="meshBody" pos="2 2 1"><geom name="meshGeom" type="mesh" mesh="cube"/></body>
  </worldbody>
</mujoco>
)XML";

    QStringList fails;
    char err[1024] = {0};
    mjSpec* spec = mj_parseXMLString(kXml, nullptr, err, int(sizeof(err)));
    if (!spec) {
        return QStringLiteral("parse failed: ") + QString::fromUtf8(err);
    }
    mjModel* m = mj_compile(spec, nullptr);
    if (!m) {
        const char* specErr = mjs_getError(spec);
        const QString why = (specErr && specErr[0]) ? QString::fromUtf8(specErr)
                                                    : QStringLiteral("compile failed");
        mj_deleteSpec(spec);
        return why;
    }
    mj_deleteSpec(spec);

    mjData* d = mj_makeData(m);
    if (!d) {
        mj_deleteModel(m);
        return QStringLiteral("mj_makeData failed");
    }
    mj_forward(m, d);

    SurfaceSnapper snapper;
    auto geomId = [&](const char* name) {
        return mj_name2id(m, mjOBJ_GEOM, name);
    };

    // 逐类型断言最近点（1 µm 容差）。
    struct Case {
        const char* geomName;
        QVector3D   probe;
        QVector3D   expect;
    };
    const Case cases[] = {
        {"floor",   QVector3D(0.0f, 0.0f, 0.05f),  QVector3D(0.0f, 0.0f, 0.0f)},
        {"boxGeom", QVector3D(1.2f, 0.0f, 1.0f),   QVector3D(1.1f, 0.0f, 1.0f)},
        {"sphGeom", QVector3D(2.3f, 0.0f, 1.0f),   QVector3D(2.2f, 0.0f, 1.0f)},
        {"cylGeom", QVector3D(3.15f, 0.0f, 1.05f), QVector3D(3.1f, 0.0f, 1.05f)},
        {"capGeom", QVector3D(4.0f, 0.0f, 1.25f),  QVector3D(4.0f, 0.0f, 1.3f)},
        {"ellGeom", QVector3D(0.3f, 2.0f, 1.0f),   QVector3D(0.2f, 2.0f, 1.0f)},
        {"meshGeom", QVector3D(2.0f, 2.0f, 1.3f),  QVector3D(2.0f, 2.0f, 1.2f)},
    };
    for (const Case& c : cases) {
        SurfaceSnapper::Hit hit;
        if (!snapper.nearestPoint(m, d, c.probe, hit)) {
            fails << QStringLiteral("no hit for ") + QString::fromLatin1(c.geomName);
            continue;
        }
        const int want = geomId(c.geomName);
        if (hit.geomId != want) {
            fails << QStringLiteral("wrong geom for ") + QString::fromLatin1(c.geomName)
                  << QStringLiteral("got") << QString::number(hit.geomId)
                  << QStringLiteral("want") << QString::number(want);
            continue;
        }
        if ((hit.point - c.expect).length() > 1e-6f) {
            fails << QStringLiteral("wrong point for ") + QString::fromLatin1(c.geomName)
                  << QStringLiteral("dist")
                  << QString::number(double((hit.point - c.expect).length()), 'g', 6);
        }
    }

    // 排除子树：把 tracked body 设成 meshBody，再查同一点应吸不到 meshGeom。
    snapper.setTrackedBody(m, mj_name2id(m, mjOBJ_BODY, "meshBody"));
    {
        SurfaceSnapper::Hit hit;
        if (snapper.nearestPoint(m, d, QVector3D(2.0f, 2.0f, 1.3f), hit) &&
            hit.geomId == geomId("meshGeom")) {
            fails << QStringLiteral("exclusion failed: mesh geom still hit");
        }
    }
    snapper.clearExclusions();

    // 回差：10 mm 吸、20 mm 脱。
    {
        SurfaceSnapper s2;
        s2.setThresholdM(0.010f, 0.020f);
        SurfaceSnapper::Hit hit;
        if (!s2.query(m, d, QVector3D(0.0f, 0.0f, 0.005f), hit))
            fails << QStringLiteral("hysteresis: 5 mm should snap");
        if (!s2.query(m, d, QVector3D(0.0f, 0.0f, 0.015f), hit))
            fails << QStringLiteral("hysteresis: 15 mm should stay snapped");
        if (s2.query(m, d, QVector3D(0.0f, 0.0f, 0.030f), hit))
            fails << QStringLiteral("hysteresis: 30 mm should release");
        if (s2.query(m, d, QVector3D(0.0f, 0.0f, 0.015f), hit))
            fails << QStringLiteral("hysteresis: 15 mm should not re-snap");
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return fails.join(QStringLiteral("; "));
}
