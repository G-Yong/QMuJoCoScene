#include "pointcloudcollision.h"
#include "CoalBodyRegistry.h"

#include <mujoco/mujoco.h>

#include <coal/data_types.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/broadphase/broadphase_callbacks.h>

#include <QHash>
#include <QVariant>
#include <QVariantList>
#include <QDebug>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

// 每个穿透点的接触快照，注入 mjData.contact 供 MuJoCo 原生可视化与 ContactInfo 查询。
struct PointSyntheticContact {
    QVector3D pos;     // 接触点世界坐标
    QVector3D normal;  // coal 法向（body 表面 → 点，朝外）
    double    dist;    // 带符号距离：负值 = 穿透
    int       geomId;  // body 首个 geom 的 MuJoCo id（-1 = 无）
};

// coal::DynamicAABBTreeCollisionManager 的碰撞回调适配器。
// 子类化 CollisionCallBackBase，在其 collide() 中做窄相 + 力累积。
struct PointCloudCallback : coal::CollisionCallBackBase {
    const coal::CollisionRequest& request;
    QVector<unsigned char>&       hitFlags;
    const QVector<QVector3D>&     points;
    double                        sign;
    QVector3D                     vLin;
    const mjtNum*                 com;       // body 质心 (xipos)
    double                        k;
    double                        c;
    coal::CollisionObject*        bodyObj;
    QVector3D&                    force;
    QVector3D&                    torque;
    // 可选：收集穿透接触数据供后续注入 mjData（nullptr 时跳过收集）。
    QVector<PointSyntheticContact>* synContacts  = nullptr;
    int                             synBodyGeomId = -1;

    PointCloudCallback(const coal::CollisionRequest& req,
                       QVector<unsigned char>&       hf,
                       const QVector<QVector3D>&     pts,
                       double                        s,
                       const QVector3D&              vl,
                       const mjtNum*                 cm,
                       double                        stiffness,
                       double                        damping,
                       coal::CollisionObject*        body,
                       QVector3D&                    f,
                       QVector3D&                    t)
        : request(req), hitFlags(hf), points(pts), sign(s), vLin(vl), com(cm),
          k(stiffness), c(damping), bodyObj(body), force(f), torque(t) {}

    bool collide(coal::CollisionObject* o1,
                 coal::CollisionObject* o2) override;
};

// PointCloudCallback::collide —— 宽相回调内的窄相 + 力累积。
bool PointCloudCallback::collide(coal::CollisionObject* o1,
                                 coal::CollisionObject* o2)
{
    // 宽相通知：o1/o2 之一是 body，另一个是点球。
    coal::CollisionObject* ptObj = (o1 == bodyObj) ? o2 : o1;

    // 窄相精确碰撞（body→point，法向约定 o1→o2）。
    coal::CollisionResult res;
    coal::collide(bodyObj, ptObj, request, res);
    if (!res.isCollision())
        return false; // 继续遍历

    const int i = static_cast<int>(
        reinterpret_cast<intptr_t>(ptObj->getUserData()));
    if (i < 0 || i >= points.size())
        return false;
    hitFlags[i] = 1;

    // 取穿透最深的接触点。
    const std::size_t nc = res.numContacts();
    int deepest = -1;
    double bestDepth = 0.0;
    for (std::size_t cIdx = 0; cIdx < nc; ++cIdx) {
        const coal::Contact& ct = res.getContact(int(cIdx));
        if (deepest < 0 || ct.penetration_depth < bestDepth) {
            bestDepth = ct.penetration_depth;
            deepest   = int(cIdx);
        }
    }
    if (deepest < 0)
        return false;

    const coal::Contact& ct = res.getContact(deepest);
    const double overlap = std::max(0.0, -static_cast<double>(ct.penetration_depth));
    if (overlap <= 0.0)
        return false;

    // 推离方向：coal 法向是 body→point，把 body 推离 → -normal。
    QVector3D nrm(static_cast<float>(ct.normal.x()),
                  static_cast<float>(ct.normal.y()),
                  static_cast<float>(ct.normal.z()));
    if (nrm.lengthSquared() < 1e-12f)
        return false;
    nrm.normalize();
    const QVector3D pushDir = nrm * static_cast<float>(-sign);

    // 弹簧-阻尼，只推不拉：F = max(0, k*overlap - c*(v·pushDir))
    const double vAlong = static_cast<double>(QVector3D::dotProduct(vLin, pushDir));
    const double fMag = std::max(0.0, k * overlap - c * vAlong);
    if (fMag <= 0.0)
        return false;
    const QVector3D F = pushDir * static_cast<float>(fMag);

    force += F;
    // 力矩 = (接触点 - 质心) × F。
    const QVector3D cp(static_cast<float>(ct.pos.x()),
                       static_cast<float>(ct.pos.y()),
                       static_cast<float>(ct.pos.z()));
    const QVector3D arm =
        cp - QVector3D(static_cast<float>(com[0]),
                       static_cast<float>(com[1]),
                       static_cast<float>(com[2]));
    torque += QVector3D::crossProduct(arm, F);

    // 收集接触数据供注入 mjData.contact（可视化 / ContactInfo 查询）。
    if (synContacts)
        synContacts->append({cp, nrm, -overlap, synBodyGeomId});

    return false; // 继续遍历
}

} // namespace

// 从接触法向构建 MuJoCo row-major 3×3 接触帧（行0=法向，行1/2=两切向）。
static void buildMjContactFrame(const QVector3D& n, mjtNum frame[9])
{
    frame[0] = mjtNum(n.x()); frame[1] = mjtNum(n.y()); frame[2] = mjtNum(n.z());
    const QVector3D aux = (qAbs(n.z()) < 0.9f) ? QVector3D(0.0f, 0.0f, 1.0f)
                                                 : QVector3D(1.0f, 0.0f, 0.0f);
    const QVector3D t1 = QVector3D::crossProduct(aux, n).normalized();
    frame[3] = mjtNum(t1.x()); frame[4] = mjtNum(t1.y()); frame[5] = mjtNum(t1.z());
    const QVector3D t2 = QVector3D::crossProduct(n, t1).normalized();
    frame[6] = mjtNum(t2.x()); frame[7] = mjtNum(t2.y()); frame[8] = mjtNum(t2.z());
}

struct PointCloudCollision::Impl {
    MujocoQuickItem* item = nullptr;

    QVector<QVector3D> points;          // 点云（世界坐标）
    double             radius = 0.01;   // 每点半径（coal 碰撞 + 渲染）

    // 每个点一个 coal Sphere 碰撞对象（位姿固定，body 动时只移动 body）。
    std::shared_ptr<coal::Sphere>                       pointShape;
    std::vector<std::shared_ptr<coal::CollisionObject>> pointObjects;

    QStringList                     bodyNames;
    QHash<QString, CoalBodyEntry>   bodies;  // 已注册（含 coal 网格）的 body

    double k = 3000.0;   // 刚度
    double c = 30.0;     // 阻尼
    bool   flip = false; // 法向取反兜底

    QVector4D baseColor {0.2f, 0.8f, 1.0f, 1.0f};
    QVector4D hitColor  {1.0f, 0.2f, 0.15f, 1.0f};

    int cloudId = -1;    // MujocoQuickItem 里的点云 id
    // 复用外部点云：>=0 时不自建渲染点云，highlight 时以 baseColors 为底色。
    int externalCloudId = -1;
    QVector<QVector4D> baseColors;  // 逐点底色（可空 => 用统一 baseColor）
    QVector<unsigned char> prevHitFlags; // 上帧命中状态，仅变化时才上传颜色

    // 点球宽相加速器：静态点云建一次，每帧 collide(bodyObj) 只访问 AABB 相交的点。
    std::unique_ptr<coal::DynamicAABBTreeCollisionManager> bvhManager;
};

PointCloudCollision::PointCloudCollision(QObject* parent)
    : QObject(parent), d(new Impl)
{
    d->pointShape = std::make_shared<coal::Sphere>(coal::Scalar(d->radius));
}

PointCloudCollision::~PointCloudCollision() = default;

void PointCloudCollision::setMujocoItem(MujocoQuickItem* item)
{
    d->item = item;
}

void PointCloudCollision::setRenderCloud(int cloudId, const QVector<QVector4D>& baseColors)
{
    d->externalCloudId = cloudId;
    d->baseColors      = baseColors;
    if (cloudId >= 0)
        d->cloudId = cloudId;
}

// ---------------------------------------------------------------------------
// 点云数据 / 渲染
// ---------------------------------------------------------------------------

void PointCloudCollision::setPoints(const QVector<QVector3D>& worldPoints)
{
    d->points = worldPoints;
    rebuildCoalPoints();

    if (!d->item)
        return;

    // 复用外部点云时，渲染由调用方负责（本类只在 update() 里改命中点颜色）。
    if (d->externalCloudId >= 0) {
        d->cloudId = d->externalCloudId;
        return;
    }

    // 渲染：首次 addPointCloud，之后 updatePointCloudPoints。
    QVariantList list;
    list.reserve(d->points.size());
    for (const QVector3D& p : d->points)
        list.append(QVariant::fromValue(p));

    if (d->cloudId < 0) {
        d->cloudId = d->item->addPointCloud(list,
                                            static_cast<float>(d->radius),
                                            MujocoQuickItem::PointStyleSphere,
                                            d->baseColor);
    } else {
        d->item->updatePointCloudPoints(d->cloudId, list);
    }
}

int PointCloudCollision::pointCount() const
{
    return d->points.size();
}

void PointCloudCollision::setPointRadius(double radius)
{
    if (radius <= 0.0) radius = 0.01;
    d->radius = radius;
    // coal 球是共享形状，半径变了要重建（连带每点对象引用新形状）。
    d->pointShape = std::make_shared<coal::Sphere>(coal::Scalar(d->radius));
    rebuildCoalPoints();
    if (d->item && d->cloudId >= 0)
        d->item->setPointCloudPointSize(d->cloudId, static_cast<float>(d->radius));
}

void PointCloudCollision::rebuildCoalPoints()
{
    d->pointObjects.clear();
    d->pointObjects.reserve(d->points.size());
    for (int i = 0; i < d->points.size(); ++i) {
        const QVector3D& p = d->points[i];
        coal::Transform3s tf;
        tf.setTranslation(coal::Vec3s(coal::Scalar(p.x()),
                                      coal::Scalar(p.y()),
                                      coal::Scalar(p.z())));
        auto obj = std::make_shared<coal::CollisionObject>(d->pointShape, tf);
        // 用 userData 存点索引，broadphase 回调中可直接取回。
        obj->setUserData(reinterpret_cast<void*>(static_cast<intptr_t>(i)));
        obj->computeAABB();
        d->pointObjects.push_back(std::move(obj));
    }

    // 重建 BVH 宽相管理器（点云静态，只建一次，之后每帧 collide 查询）。
    if (!d->bvhManager)
        d->bvhManager = std::make_unique<coal::DynamicAABBTreeCollisionManager>();
    d->bvhManager->clear();
    for (auto& obj : d->pointObjects)
        d->bvhManager->registerObject(obj.get());
    d->bvhManager->setup();
}

void PointCloudCollision::setStiffness(double k) { d->k = k; }
void PointCloudCollision::setDamping(double c)   { d->c = c; }
void PointCloudCollision::setNormalFlip(bool flip) { d->flip = flip; }
void PointCloudCollision::setBaseColor(const QVector4D& rgba) { d->baseColor = rgba; }
void PointCloudCollision::setHitColor(const QVector4D& rgba)  { d->hitColor = rgba; }

// ---------------------------------------------------------------------------
// 碰撞 body 注册（coal 网格）
// ---------------------------------------------------------------------------

void PointCloudCollision::addCollisionBody(const QString& bodyName)
{
    if (bodyName.isEmpty() || d->bodyNames.contains(bodyName))
        return;
    d->bodyNames.append(bodyName);
    // 若场景已就绪则立即注册网格。
    if (d->item) {
        CoalBodyEntry entry;
        if (buildBodyMeshEntry(d->item, bodyName, &entry, QStringLiteral("pointcloud")))
            d->bodies.insert(bodyName, std::move(entry));
    }
}

void PointCloudCollision::clearCollisionBodies()
{
    d->bodyNames.clear();
    d->bodies.clear();
}

void PointCloudCollision::setupForLoadedScene()
{
    // 场景切换：旧 bodyId 失效，重建注册。
    d->bodies.clear();
    for (const QString& name : d->bodyNames) {
        CoalBodyEntry entry;
        if (buildBodyMeshEntry(d->item, name, &entry, QStringLiteral("pointcloud")))
            d->bodies.insert(name, std::move(entry));
    }

    // 复用外部点云时，cloudId 由调用方在重建点云后通过 setRenderCloud 提供，
    // 这里不重置、也不自建。
    if (d->externalCloudId < 0)
        d->cloudId = -1;
    if (!d->points.isEmpty())
        setPoints(d->points);
}

// ---------------------------------------------------------------------------
// 每帧：碰撞检测 + 注入惩罚力
// ---------------------------------------------------------------------------

void PointCloudCollision::update()
{
    if (!d->item || d->points.isEmpty() || d->bodies.isEmpty())
        return;

    QVector<unsigned char> hitFlags(d->points.size(), 0);

    coal::CollisionRequest request;
    request.enable_contact   = true;
    request.num_max_contacts = 4;

    const double sign = d->flip ? -1.0 : 1.0;

    d->item->withMutableSimulation([&](mjModel* m, mjData* dat) {
        QVector<PointSyntheticContact> allSynContacts;
        for (auto it = d->bodies.begin(); it != d->bodies.end(); ++it) {
            CoalBodyEntry& be = it.value();
            if (be.bodyId < 0 || be.bodyId >= m->nbody || !be.object)
                continue;

            // 1) body 当前世界位姿 → 同步给 coal。
            const mjtNum* xp = dat->xpos + 3 * be.bodyId;   // 世界位置
            const mjtNum* xm = dat->xmat + 9 * be.bodyId;   // row-major 3x3
            coal::Matrix3s R;
            for (int r = 0; r < 3; ++r)
                for (int cidx = 0; cidx < 3; ++cidx)
                    R(r, cidx) = coal::Scalar(xm[3 * r + cidx]);
            coal::Transform3s tf;
            tf.setRotation(R);
            tf.setTranslation(coal::Vec3s(coal::Scalar(xp[0]),
                                          coal::Scalar(xp[1]),
                                          coal::Scalar(xp[2])));
            be.object->setTransform(tf);
            be.object->computeAABB();

            // body 质心（施力点 / 力臂基准）与世界线速度（阻尼用）。
            const mjtNum* com = dat->xipos + 3 * be.bodyId;
            mjtNum vel6[6] = {0};
            mj_objectVelocity(m, dat, mjOBJ_BODY, be.bodyId, vel6, /*flg_local=*/0);
            const QVector3D vLin(static_cast<float>(vel6[3]),
                                 static_cast<float>(vel6[4]),
                                 static_cast<float>(vel6[5]));

            // 2) BVH 宽相 + 窄相：DynamicAABBTreeCollisionManager 找出 AABB 相交
            //    的点球候选集，通过 PointCloudCallback 适配器做窄相 + 力累积。
            QVector3D force(0, 0, 0);
            QVector3D torque(0, 0, 0);

            if (d->bvhManager && !d->bvhManager->empty()) {
                PointCloudCallback cb(request, hitFlags, d->points, sign,
                                      vLin, com, d->k, d->c,
                                      be.object.get(), force, torque);
                cb.synContacts   = &allSynContacts;
                cb.synBodyGeomId = (be.bodyId > 0 && be.bodyId < m->nbody)
                                   ? m->body_geomadr[be.bodyId] : -1;
                d->bvhManager->collide(be.object.get(), &cb);
            }

            // 3) 写回 xfrc_applied（世界系，作用于质心）。每帧先清零本 body 的
            //    分量再写入，避免上一帧的力残留累加。
            mjtNum* f = dat->xfrc_applied + 6 * be.bodyId;
            f[0] = force.x();  f[1] = force.y();  f[2] = force.z();
            f[3] = torque.x(); f[4] = torque.y(); f[5] = torque.z();
        }

        // 4) 把惩罚力计算中的穿透点注入 mjData.contact。
        //    物理已由 xfrc_applied 处理；此处仅为可视化 / MujocoQuickItem::contacts 查询。
        //    efc_address=-1 让求解器跳过（不重复施力）；
        //    exclude=0   让 mjv_updateScene 渲染接触点球（需开启 mjVIS_CONTACTPOINT）；
        //    normalForce 将为 0（mj_contactForce 对 efc<0 返回零）。
        for (const PointSyntheticContact& sc : qAsConst(allSynContacts)) {
            if (dat->ncon >= m->nconmax)
                break;
            mjContact& mc = dat->contact[dat->ncon];
            std::memset(&mc, 0, sizeof(mjContact));
            mc.pos[0] = mjtNum(sc.pos.x());
            mc.pos[1] = mjtNum(sc.pos.y());
            mc.pos[2] = mjtNum(sc.pos.z());
            buildMjContactFrame(sc.normal, mc.frame);
            mc.dist        = mjtNum(sc.dist);
            mc.geom[0]     = sc.geomId;   // body 的首个 mesh geom
            mc.geom[1]     = -1;          // 点云点无对应 geom
            mc.geom1       = sc.geomId;   // 兼容字段（已废弃）
            mc.geom2       = -1;
            mc.exclude     = 0;
            mc.efc_address = -1;
            mc.dim         = 3;
            ++dat->ncon;
        }
    });

    // 4) 高亮：穿透点染成 hitColor，其余 baseColor。
    refreshRenderColors(hitFlags);
}

void PointCloudCollision::refreshRenderColors(const QVector<unsigned char>& hitFlags)
{
    if (!d->item || d->cloudId < 0 || hitFlags.size() != d->points.size())
        return;
    // 点数可达几万，每帧重传整个颜色缓冲开销很大；仅在命中集变化时上传。
    if (d->prevHitFlags == hitFlags)
        return;
    d->prevHitFlags = hitFlags;

    const bool perPointBase = (d->baseColors.size() == d->points.size());
    QVariantList colors;
    colors.reserve(hitFlags.size());
    for (int i = 0; i < hitFlags.size(); ++i) {
        const QVector4D base = perPointBase ? d->baseColors[i] : d->baseColor;
        colors.append(QVariant::fromValue(hitFlags[i] ? d->hitColor : base));
    }
    d->item->setPointCloudColors(d->cloudId, colors);
}
