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

#include <coal/mesh_loader/assimp.h>   // coal::internal::Loader + buildMesh
#include <coal/BVH/BVH_model.h>        // coal::Vec3s, Triangle32 (via internal)

// 输出结构（与旧 PlyCloud / 旧 MeshCloud 兼容）
struct MeshCloud {
    QVector<float> positions;   // xyz 扁平，长度 = count * 3
    QVector<float> colors;      // 始终为空（颜色由调用方按高度等策略生成）
    int            count   = 0;
    bool           hasColor = false;
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

    coal::internal::Loader loader;
    try {
        loader.load(path.toStdString());
    } catch (const std::exception& e) {
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
