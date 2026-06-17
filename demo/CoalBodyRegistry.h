#ifndef COALBODYREGISTRY_H
#define COALBODYREGISTRY_H

// ---------------------------------------------------------------------------
// CoalBodyRegistry — MuJoCo body 到 coal 网格模型的注册工具。
//
// 提供两个 collision demo 共用的基础设施：
//   1. findBodyId()     — body 名称 → MuJoCo bodyId
//   2. buildCoalModel() — 从顶点/索引构建 coal BVHModel + CollisionObject
//
// 两个调用方各自有不同的存储/业务逻辑：
//   · PointCloudCollision  — 点球 vs body 惩罚力注入
//   · CoalCollision / CoalCollisionDetector — body vs body 精确碰撞
// ---------------------------------------------------------------------------

#include <QString>
#include <QVector>
#include <QVector3D>
#include <QDebug>
#include <memory>

#include <coal/fwd.hh>
#include <coal/BV/OBBRSS.h>
#include <coal/BVH/BVH_model.h>
#include <coal/collision_object.h>

#include "MujocoQuickItem.h"

// ---------------------------------------------------------------------------
// CoalBodyEntry — 一个已注册 body 的 coal 碰撞表示。
// ---------------------------------------------------------------------------
struct CoalBodyEntry {
    int                                         bodyId      = -1;
    std::shared_ptr<coal::BVHModel<coal::OBBRSS>> model;
    std::shared_ptr<coal::CollisionObject>        object;
    double                                        boundRadius = 0.0;

    bool valid() const { return model && object; }
};

// ---------------------------------------------------------------------------
// findBodyId — 在 MujocoQuickItem 中按名称查找 body 的 0-base id。
// ---------------------------------------------------------------------------
inline int findBodyId(const MujocoQuickItem* item, const QString& name)
{
    if (!item) return -1;
    const int count = item->objectCount();
    for (int i = 0; i < count; ++i) {
        if (item->objectInfo(i).name == name)
            return i;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// buildCoalModel — 从三角网格顶点/索引构建 coal BVHModel + CollisionObject。
//
// vertices  — body 局部坐标系顶点列表
// indices   — 三角面索引（长度须为 3 的倍数）
//
// 返回 true 并填充 out；无效输入返回 false。
// ---------------------------------------------------------------------------
inline bool buildCoalModel(const QVector<QVector3D>& vertices,
                           const QVector<int>&        indices,
                           CoalBodyEntry*             out)
{
    if (!out || vertices.isEmpty() || indices.size() < 3)
        return false;

    const int vertexCount = vertices.size();

    std::vector<coal::Vec3s> ps;
    ps.reserve(vertexCount);
    for (const QVector3D& v : vertices)
        ps.emplace_back(coal::Scalar(v.x()), coal::Scalar(v.y()), coal::Scalar(v.z()));

    std::vector<coal::Triangle32> ts;
    ts.reserve(indices.size() / 3);
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

    auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
    model->beginModel(static_cast<unsigned int>(ts.size()),
                      static_cast<unsigned int>(ps.size()));
    model->addSubModel(ps, ts);
    model->endModel();
    model->computeLocalAABB();

    out->model       = model;
    out->object      = std::make_shared<coal::CollisionObject>(
        model, coal::Transform3s::Identity());
    out->boundRadius = static_cast<double>(model->aabb_radius);
    return true;
}

// ---------------------------------------------------------------------------
// buildBodyMeshEntry — 组合 findBodyId + bodyCollisionMesh + buildCoalModel。
//
// 从 MuJoCo body 名称出发，拿到网格数据并构建 coal 模型。bodyId 同时写入 out。
// 返回 true 表示成功，失败填充 errMsg（可选）。
// ---------------------------------------------------------------------------
inline bool buildBodyMeshEntry(const MujocoQuickItem* item,
                               const QString&          name,
                               CoalBodyEntry*          out,
                               const QString&          logTag = QString(),
                               QString*                errMsg = nullptr)
{
    if (!item || !out) return false;

    const int bodyId = findBodyId(item, name);
    if (bodyId < 0) {
        const QString msg = QStringLiteral("[%1] body not found in scene: %2")
            .arg(logTag.isEmpty() ? QStringLiteral("coalbody") : logTag)
            .arg(name);
        qWarning() << msg;
        if (errMsg) *errMsg = msg;
        return false;
    }

    const BodyMeshData mesh = item->bodyCollisionMesh(bodyId);
    if (!mesh.valid || mesh.vertices.isEmpty() || mesh.indices.isEmpty()) {
        const QString msg = QStringLiteral("[%1] body has no triangle mesh geom: %2")
            .arg(logTag.isEmpty() ? QStringLiteral("coalbody") : logTag)
            .arg(name);
        qWarning() << msg;
        if (errMsg) *errMsg = msg;
        return false;
    }

    CoalBodyEntry entry;
    if (!buildCoalModel(mesh.vertices, mesh.indices, &entry)) {
        const QString msg = QStringLiteral("[%1] failed to build collision model: %2")
            .arg(logTag.isEmpty() ? QStringLiteral("coalbody") : logTag)
            .arg(name);
        qWarning() << msg;
        if (errMsg) *errMsg = msg;
        return false;
    }

    entry.bodyId = bodyId;
    *out = std::move(entry);

    if (!logTag.isEmpty()) {
        qDebug() << "[" << logTag << "] registered body:" << name
                 << "verts" << mesh.vertices.size()
                 << "tris" << (mesh.indices.size() / 3);
    }
    return true;
}

#endif // COALBODYREGISTRY_H
