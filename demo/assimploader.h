#ifndef ASSIMPLOADER_H
#define ASSIMPLOADER_H

// ---------------------------------------------------------------------------
// 通用 3D 模型顶点提取器。
// 使用 coal 内置的 Assimp 集成（coal/mesh_loader/assimp.h）加载模型，
// 支持所有 Assimp 格式：.ply .stl .obj .fbx .dae .3ds .gltf 等。
//
// 不需要在本项目中直接引入 assimp 头文件或链接 assimp 库——
// coal 的 DLL 已经内部链接了 assimp；本头文件只依赖 coal。
//
// 输出顶点列表（不含颜色，适合按高度色等外部策略着色），
// 可直接用于 MujocoQuickItem::addPointCloud 渲染与 coal 碰撞。
// ---------------------------------------------------------------------------

#include <QString>
#include <QVector>
#include <QDebug>

#include <Eigen/Dense>

#include <coal/mesh_loader/assimp.h>   // coal::internal::Loader + buildMesh
#include <coal/BVH/BVH_model.h>        // coal::Vec3s, Triangle32 (via internal)

#include "plyloader.h"                 // loadPly — 纯点云 fallback（无面片时）

// 输出结构（与旧 PlyCloud / 旧 MeshCloud 兼容）
struct MeshCloud {
    QVector<float> positions;   //  xyz 扁平，长度 = count * 3
    QVector<float> colors;      // rgba[0.0-1.0] 假如模型文件本身带颜色就填充颜色,没有则由调用方自己填充
    int            count   = 0;
    bool           hasColor = false; // 模型是否带颜色
};

// ---------------------------------------------------------------------------
// loadMesh — 读取任意 Assimp 支持的模型文件并提取顶点位置。
//
// 参数：
//   path  - 文件路径（toStdString() 传给 coal Loader，通常 UTF-8 路径可用）
//   out   - 输出 MeshCloud；返回 false 时内容未定义
//   err   - 可选错误信息输出
//   scale - 各轴缩放（默认 1,1,1）
//
// 坐标系说明：
//   Assimp / coal 不统一各格式的坐标系；本函数直接输出原始坐标，
//   由调用方根据模型实际朝向做轴变换（与原 plyloader 做法一致）。
// ---------------------------------------------------------------------------
inline bool loadMesh(const QString& path, MeshCloud* out, QString* err = nullptr,
                     const coal::Vec3s& scale = coal::Vec3s::Ones())
{
    if (!out) return false;

    // PLY 纯点云回退：coal/Assimp 的 Loader 和 buildMesh 都依赖面片，
    // 对 element face 0 的纯点云会直接报 "No meshes remaining"。
    // 此时用本地 plyloader 直接读取 vertex 元素位置。
    auto tryPlyFallback = [&]() -> bool {
        if (!path.toLower().endsWith(QStringLiteral(".ply")))
            return false;
        PlyCloud ply;
        QString plyErr;
        if (!loadPly(path, &ply, &plyErr) || ply.count <= 0) {
            if (err) *err = plyErr.isEmpty() ? QStringLiteral("ply fallback failed: %1").arg(path) : plyErr;
            return false;
        }
        out->positions.clear();
        out->positions.reserve(ply.count * 3);
        for (int i = 0; i < ply.count; ++i) {
            Eigen::Vector3f p(
                ply.positions[3 * i + 0] * scale[0],
                ply.positions[3 * i + 1] * scale[1],
                ply.positions[3 * i + 2] * scale[2]);
            out->positions << p.x() << p.y() << p.z();
        }
        out->colors   = ply.colors;
        out->count    = ply.count;
        out->hasColor = ply.hasColor;
        return true;
    };

    coal::internal::Loader loader;
    try {
        loader.load(path.toStdString());
    } catch (const std::exception& e) {
        // coal/Assimp 加载失败（如纯点云 "No meshes remaining"）→ 回退 plyloader
        if (tryPlyFallback()) return true;
        if (err) *err = QString::fromLatin1(e.what());
        return false;
    }

    if (!loader.scene) {
        if (err) *err = QStringLiteral("coal/assimp failed to load: %1").arg(path);
        return false;
    }

    // buildMesh 遍历 aiScene 所有子网格，三角化后填充 tv.vertices_。
    coal::internal::TriangleAndVertices tv;
    coal::internal::buildMesh(scale, loader.scene, 0, tv);

    if (tv.vertices_.empty()) {
        // 有 scene 但 buildMesh 没产出顶点（如空 mesh）→ 回退 plyloader
        if (tryPlyFallback()) return true;
        if (err) *err = QStringLiteral("no vertices found in: %1").arg(path);
        return false;
    }

    const int n = static_cast<int>(tv.vertices_.size());
    out->positions.clear();
    out->positions.reserve(n * 3);
    for (const coal::Vec3s& v : tv.vertices_) {
        out->positions << static_cast<float>(v[0])
                       << static_cast<float>(v[1])
                       << static_cast<float>(v[2]);
    }
    out->colors.clear();
    out->count    = n;
    out->hasColor = false;
    return true;
}

#endif // ASSIMPLOADER_H
