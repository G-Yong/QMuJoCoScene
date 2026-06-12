#include "coalcollision.h"

#include <coal/fwd.hh>
#include <coal/data_types.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/BV/OBBRSS.h>
#include <coal/BVH/BVH_model.h>
#include <coal/math/transform.h>

#include <QHash>
#include <memory>
#include <vector>


CoalCollision::CoalCollision(QObject *parent)
    : QObject{parent}, m_coal{new CoalCollisionDetector()}
{
}

void CoalCollision::setMujocoItem(MujocoQuickItem *item)
{
    mMujocoItem = item;
}

// ---------------------------------------------------------------------------
// coal 精确网格碰撞检测（接入 MuJoCo 求解器）
// ---------------------------------------------------------------------------

void CoalCollision::addCoalPair(const QString& bodyA, const QString& bodyB)
{
    if (bodyA.isEmpty() || bodyB.isEmpty() || bodyA == bodyB)
        return;
    for (const auto& p : qAsConst(m_coalPairNames)) {
        if ((p.first == bodyA && p.second == bodyB) ||
            (p.first == bodyB && p.second == bodyA))
            return; // 已存在
    }
    m_coalPairNames.append(qMakePair(bodyA, bodyB));

    // 场景已就绪则立即注册并加入检测对
    if (m_coalSceneReady) {
        registerCoalBody(bodyA);
        registerCoalBody(bodyB);
        if (m_coal->hasBody(bodyA) && m_coal->hasBody(bodyB))
            m_coal->addPair(bodyA, bodyB);
        // 重建窄阶段（新 pair 需要加入 filter）
        uninstallCoalNarrowPhase();
        setupCoalPhysics();
    }
}

void CoalCollision::clearCoalPairs()
{
    teardownCoalState(true);
    m_coalPairNames.clear();
}

void CoalCollision::setCoalSecurityMargin(double margin)
{
    m_coal->setSecurityMargin(margin);
}

void CoalCollision::setCoalNormalFlip(bool flip)
{
    if (m_coalNormalFlip == flip)
        return;
    m_coalNormalFlip = flip;
    // 重装窄阶段以让新的法向符号生效。
    if (m_coalPhysicsInstalled) {
        uninstallCoalNarrowPhase();
        setupCoalPhysics();
    }
}

bool CoalCollision::coalNormalFlip() const
{
    return m_coalNormalFlip;
}

void CoalCollision::setupCoalForLoadedScene()
{
    // 场景切换：旧 bodyId 全部失效，重置运行态（不触碰旧模型）。
    teardownCoalState(false);
    m_coalSceneReady = true;

    // 重新注册所有用户请求的碰撞对并安装窄阶段。
    rebuildCoalAndActivate();
}

void CoalCollision::rebuildCoalAndActivate()
{
    for (const auto& pair : qAsConst(m_coalPairNames)) {
        registerCoalBody(pair.first);
        registerCoalBody(pair.second);
        if (m_coal->hasBody(pair.first) && m_coal->hasBody(pair.second))
            m_coal->addPair(pair.first, pair.second);
    }

    // 直接安装窄阶段（唯一模式）
    setupCoalPhysics();
}

void CoalCollision::teardownCoalState(bool restoreModelState)
{
    uninstallCoalNarrowPhase();

    m_coal->clear();
    m_coalBodyId.clear();
    m_coalBodyName.clear();
    m_coalPhysicsPairs.clear();
}

void CoalCollision::setupCoalPhysics()
{
    // 建立 bodyId → name 反查表（provider 在物理线程用 bodyId 反查网格名）。
    m_coalBodyName.clear();
    for (auto it = m_coalBodyId.begin(); it != m_coalBodyId.end(); ++it)
        m_coalBodyName.insert(it.value(), it.key());

    // 建立接管的 body 对集合（归一化为 (min,max)，filter 顺序无关）。
    m_coalPhysicsPairs.clear();
    for (const auto& pair : qAsConst(m_coalPairNames)) {
        const int a = m_coalBodyId.value(pair.first, -1);
        const int b = m_coalBodyId.value(pair.second, -1);
        if (a < 0 || b < 0) continue;
        m_coalPhysicsPairs.insert(qMakePair(qMin(a, b), qMax(a, b)));
    }

    installCoalNarrowPhase();
}

void CoalCollision::installCoalNarrowPhase()
{
    if (m_coalPhysicsInstalled || m_coalPhysicsPairs.isEmpty())
        return;

    // filter：物理线程调用，判断一对 body 是否由 coal 接管（顺序无关）。
    auto filter = [this](int body1, int body2) -> bool {
        return m_coalPhysicsPairs.contains(qMakePair(qMin(body1, body2),
                                                     qMax(body1, body2)));
    };

    const bool flip = m_coalNormalFlip;

    // provider：物理线程调用，给定两 body 世界位姿，用 coal 计算精确接触。
    auto provider = [this, flip](int body1, const double* xpos1, const double* xmat1,
                                 int body2, const double* xpos2, const double* xmat2,
                                 MujocoQuickItem::ExternalContactPoint* out, int maxOut) -> int
    {
        const QString nameA = m_coalBodyName.value(body1);
        const QString nameB = m_coalBodyName.value(body2);
        if (nameA.isEmpty() || nameB.isEmpty())
            return 0;

        // MuJoCo xmat 为 row-major 3x3，转 QQuaternion 设置 coal 世界位姿。
        auto toQuat = [](const double* m) -> QQuaternion {
            float r[9];
            for (int i = 0; i < 9; ++i) r[i] = static_cast<float>(m[i]);
            return QQuaternion::fromRotationMatrix(QMatrix3x3(r)).normalized();
        };
        const QVector3D posA(static_cast<float>(xpos1[0]),
                             static_cast<float>(xpos1[1]),
                             static_cast<float>(xpos1[2]));
        const QVector3D posB(static_cast<float>(xpos2[0]),
                             static_cast<float>(xpos2[1]),
                             static_cast<float>(xpos2[2]));
        m_coal->updatePose(nameA, posA, toQuat(xmat1));
        m_coal->updatePose(nameB, posB, toQuat(xmat2));

        const CoalCollisionResult res = m_coal->detectPair(nameA, nameB);
        if (!res.colliding)
            return 0;

        if (res.contacts.isEmpty() || maxOut <= 0)
            return 0;

        // ---------------------------------------------------------------
        // 接触归并（关键）：coal 的 mesh×mesh 窄阶段会为"每一对相交三角形"
        // 各产生一个接触点。对一个三角化的小球落在弯曲滑道上的情形，会同时得到
        // 几十个接触点，而且：
        //   · 每个点的法向是各自三角形的法向，逐面抖动、方向发散；
        //   · 个别三角形法向甚至朝向相反（指向小球内部）；
        //   · 每个点的穿透深度各不相同且偏小。
        // 把这一大堆互相冲突的约束直接喂给 MuJoCo 求解器，会让质量极小
        //（0.5 g）的小球受到方向矛盾的接触力 —— 表现就是"抽动 + 缓慢穿透"。
        //
        // 修复：把同一对 body 的所有接触点归并成一个稳定接触：
        //   1) 以穿透最深的接触法向为参考方向；
        //   2) 其余法向若与参考反向（点积<0）则翻转，避免反向法向相互抵消；
        //   3) 按穿透深度加权平均得到统一法向（单一接触流形法向）；
        //   4) 接触点取穿透最深处，带符号距离取最深（最负）值。
        // 这样 MuJoCo 得到的是"单法向 + 单接触点"的干净接触，求解稳定。
        // ---------------------------------------------------------------
        int deepest = 0;
        for (int i = 1; i < res.contacts.size(); ++i) {
            if (res.contacts[i].penetrationDepth < res.contacts[deepest].penetrationDepth)
                deepest = i;
        }

        QVector3D ref = res.contacts[deepest].normal;
        if (ref.lengthSquared() < 1e-12f)
            return 0;                 // 退化法向，放弃本对接触
        ref.normalize();

        QVector3D accum(0.0f, 0.0f, 0.0f);
        for (const CoalContactPoint& cp : res.contacts) {
            QVector3D nrm = cp.normal;
            if (nrm.lengthSquared() < 1e-12f)
                continue;
            nrm.normalize();
            if (QVector3D::dotProduct(nrm, ref) < 0.0f)
                nrm = -nrm;           // 翻转朝向相反的三角形法向
            const float w = static_cast<float>(qAbs(cp.penetrationDepth)) + 1e-6f;
            accum += nrm * w;
        }
        const QVector3D unified =
            accum.lengthSquared() > 1e-12f ? accum.normalized() : ref;

        // coal 的法向约定：normal 从 o1（=nameA=body1=geom[0]）指向 o2
        //（=nameB=body2=geom[1]），与 MuJoCo "frame row0 从 geom[0] 指向 geom[1]"
        // 完全一致，默认无需取反。flip=true 时取反，仅作异常兜底开关。
        const float s = flip ? -1.0f : 1.0f;
        const CoalContactPoint& best = res.contacts[deepest];
        out[0].pos[0] = best.position.x();
        out[0].pos[1] = best.position.y();
        out[0].pos[2] = best.position.z();
        out[0].normal[0] = s * unified.x();
        out[0].normal[1] = s * unified.y();
        out[0].normal[2] = s * unified.z();
        // coal 的 penetration_depth 是"带符号距离"：穿透时为负，与 MuJoCo
        // mjContact.dist（负值表示穿透）约定一致，直接透传，切勿取反。
        out[0].dist = best.penetrationDepth;
        return 1;
    };

    mMujocoItem->setExternalNarrowPhase(filter, provider);
    m_coalPhysicsInstalled = mMujocoItem->externalNarrowPhaseInstalled();
}

void CoalCollision::uninstallCoalNarrowPhase()
{
    if (!m_coalPhysicsInstalled)
        return;
    mMujocoItem->setExternalNarrowPhase(MujocoQuickItem::ExternalPairFilter(),
                                        MujocoQuickItem::ExternalNarrowPhaseFn());
    m_coalPhysicsInstalled = false;
}

void CoalCollision::registerCoalBody(const QString& name)
{
    if (m_coal->hasBody(name))
        return;
    const int bodyId = findBodyId(name);
    if (bodyId < 0) {
        qWarning() << "[coal] body not found in scene:" << name;
        return;
    }
    const BodyMeshData mesh = mMujocoItem->bodyCollisionMesh(bodyId);
    if (!mesh.valid || mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
        qWarning() << "[coal] body has no triangle mesh geom:" << name;
        return;
    }
    if (!m_coal->registerBody(name, mesh.vertices, mesh.indices)) {
        qWarning() << "[coal] failed to build collision model for body:" << name;
        return;
    }
    m_coalBodyId.insert(name, bodyId);

    qDebug() << "registered coal body:" << name << "with" << mesh.vertices.size() << "vertices and"
             << (mesh.indices.size() / 3) << "triangles";
}

int CoalCollision::findBodyId(const QString& name) const
{
    const int count = mMujocoItem->objectCount();
    for (int i = 0; i < count; ++i) {
        if (mMujocoItem->objectInfo(i).name == name)
            return i;
    }
    return -1;
}



namespace {
using MeshModel = coal::BVHModel<coal::OBBRSS>;

struct BodyEntry {
    std::shared_ptr<MeshModel>            model;
    std::shared_ptr<coal::CollisionObject> object;
};
} // namespace

struct CoalCollisionDetector::Impl {
    QHash<QString, BodyEntry>                bodies;
    QVector<QPair<QString, QString>>         pairs;
    double                                   securityMargin = 0.0;
};

CoalCollisionDetector::CoalCollisionDetector()
    : d(new Impl)
{
}

CoalCollisionDetector::~CoalCollisionDetector() = default;

bool CoalCollisionDetector::registerBody(const QString& name,
                                         const QVector<QVector3D>& vertices,
                                         const QVector<int>& indices)
{
    if (name.isEmpty() || vertices.isEmpty() || indices.size() < 3)
        return false;

    std::vector<coal::Vec3s> ps;
    ps.reserve(vertices.size());
    for (const QVector3D& v : vertices)
        ps.emplace_back(coal::Scalar(v.x()), coal::Scalar(v.y()), coal::Scalar(v.z()));

    std::vector<coal::Triangle32> ts;
    ts.reserve(indices.size() / 3);
    const int vertexCount = vertices.size();
    for (int i = 0; i + 2 < indices.size(); i += 3) {
        const int a = indices[i];
        const int b = indices[i + 1];
        const int c = indices[i + 2];
        if (a < 0 || b < 0 || c < 0 ||
            a >= vertexCount || b >= vertexCount || c >= vertexCount)
            continue;
        ts.emplace_back(static_cast<std::uint32_t>(a),
                        static_cast<std::uint32_t>(b),
                        static_cast<std::uint32_t>(c));
    }
    if (ts.empty())
        return false;

    auto model = std::make_shared<MeshModel>();
    model->beginModel(static_cast<unsigned int>(ts.size()),
                      static_cast<unsigned int>(ps.size()));
    model->addSubModel(ps, ts);
    model->endModel();
    model->computeLocalAABB();

    BodyEntry entry;
    entry.model  = model;
    entry.object = std::make_shared<coal::CollisionObject>(
        model, coal::Transform3s::Identity());

    d->bodies.insert(name, std::move(entry));
    return true;
}

bool CoalCollisionDetector::hasBody(const QString& name) const
{
    return d->bodies.contains(name);
}

void CoalCollisionDetector::removeBody(const QString& name)
{
    d->bodies.remove(name);
}

void CoalCollisionDetector::clear()
{
    d->bodies.clear();
    d->pairs.clear();
}

void CoalCollisionDetector::setPairs(const QVector<QPair<QString, QString>>& pairs)
{
    d->pairs = pairs;
}

void CoalCollisionDetector::addPair(const QString& a, const QString& b)
{
    if (a.isEmpty() || b.isEmpty() || a == b)
        return;
    for (const auto& p : qAsConst(d->pairs)) {
        if ((p.first == a && p.second == b) || (p.first == b && p.second == a))
            return;
    }
    d->pairs.append(qMakePair(a, b));
}

void CoalCollisionDetector::clearPairs()
{
    d->pairs.clear();
}

QVector<QPair<QString, QString>> CoalCollisionDetector::pairs() const
{
    return d->pairs;
}

void CoalCollisionDetector::updatePose(const QString& name,
                                       const QVector3D& position,
                                       const QQuaternion& orientation)
{
    auto it = d->bodies.find(name);
    if (it == d->bodies.end() || !it->object)
        return;

    QQuaternion q = orientation.normalized();
    coal::Quaternion3f cq(coal::Scalar(q.scalar()),
                          coal::Scalar(q.x()),
                          coal::Scalar(q.y()),
                          coal::Scalar(q.z()));
    coal::Transform3s tf;
    tf.setQuatRotation(cq);
    tf.setTranslation(coal::Vec3s(coal::Scalar(position.x()),
                                  coal::Scalar(position.y()),
                                  coal::Scalar(position.z())));
    it->object->setTransform(tf);
    it->object->computeAABB();
}

QVector<CoalCollisionResult> CoalCollisionDetector::detect()
{
    QVector<CoalCollisionResult> results;
    results.reserve(d->pairs.size());

    coal::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;
    request.enable_distance_lower_bound = true;
    request.security_margin = coal::Scalar(d->securityMargin);

    for (const auto& pair : qAsConst(d->pairs)) {
        auto itA = d->bodies.find(pair.first);
        auto itB = d->bodies.find(pair.second);
        if (itA == d->bodies.end() || itB == d->bodies.end())
            continue;
        if (!itA->object || !itB->object)
            continue;

        coal::CollisionResult res;
        coal::collide(itA->object.get(), itB->object.get(), request, res);

        CoalCollisionResult out;
        out.bodyA = pair.first;
        out.bodyB = pair.second;
        out.colliding = res.isCollision();
        out.distanceLowerBound = double(res.distance_lower_bound);

        if (out.colliding) {
            const std::size_t n = res.numContacts();
            out.contacts.reserve(int(n));
            for (std::size_t i = 0; i < n; ++i) {
                const coal::Contact& c = res.getContact(int(i));
                CoalContactPoint cp;
                cp.position = QVector3D(float(c.pos.x()), float(c.pos.y()), float(c.pos.z()));
                cp.normal   = QVector3D(float(c.normal.x()), float(c.normal.y()), float(c.normal.z()));
                cp.penetrationDepth = double(c.penetration_depth);
                out.contacts.append(cp);
            }
        }
        results.append(out);
        res.clear();
    }
    return results;
}

CoalCollisionResult CoalCollisionDetector::detectPair(const QString& a, const QString& b)
{
    CoalCollisionResult out;
    out.bodyA = a;
    out.bodyB = b;

    auto itA = d->bodies.find(a);
    auto itB = d->bodies.find(b);
    if (itA == d->bodies.end() || itB == d->bodies.end())
        return out;
    if (!itA->object || !itB->object)
        return out;

    coal::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;
    request.enable_distance_lower_bound = true;
    request.security_margin = coal::Scalar(d->securityMargin);

    coal::CollisionResult res;
    coal::collide(itA->object.get(), itB->object.get(), request, res);

    out.colliding = res.isCollision();
    out.distanceLowerBound = double(res.distance_lower_bound);
    if (out.colliding) {
        const std::size_t n = res.numContacts();
        out.contacts.reserve(int(n));
        for (std::size_t i = 0; i < n; ++i) {
            const coal::Contact& c = res.getContact(int(i));
            CoalContactPoint cp;
            cp.position = QVector3D(float(c.pos.x()), float(c.pos.y()), float(c.pos.z()));
            cp.normal   = QVector3D(float(c.normal.x()), float(c.normal.y()), float(c.normal.z()));
            cp.penetrationDepth = double(c.penetration_depth);
            out.contacts.append(cp);
        }
    }
    res.clear();
    return out;
}

void CoalCollisionDetector::setSecurityMargin(double margin)
{
    d->securityMargin = margin;
}

double CoalCollisionDetector::securityMargin() const
{
    return d->securityMargin;
}