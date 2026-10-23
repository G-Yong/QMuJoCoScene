#pragma once
// ---------------------------------------------------------------------------
// 描边（线稿叠加）用的纯数据类型。
//
// 只依赖 QtGui 的 QVector4D，不含 MuJoCo / simulate 头，因此可以随
// MujocoQuickItem.h 一起部署到 deploy/include（客户编译只需要这些 POD）。
//
// 数据流：
//   物理线程（持 sim.mtx）按 mjData 采样 → std::vector<OutlineGeomSample>
//   → 渲染线程按 scn.geoms 可见集过滤 → OutlineRenderer::buildGeometry()
// ---------------------------------------------------------------------------

#include <QVector4D>

// 单个 geom 的世界位姿与渲染所需的静态属性（每次采样都从 mjModel/mjData 现拷，
// 不做跨帧指针缓存：模型重编译会换数组）。
struct OutlineGeomSample {
    int   geomId   = -1;      // mjModel geom id
    int   geomType = -1;      // mjtGeom；不支持的类型渲染时跳过
    int   meshId   = -1;      // geom_type == mjGEOM_MESH 时的 geom_dataid
    float size[3]  = {0.0f, 0.0f, 0.0f};                        // mjModel::geom_size
    float xpos[3]  = {0.0f, 0.0f, 0.0f};                        // mjData::geom_xpos
    float xmat[9]  = {1.0f, 0.0f, 0.0f,                          // mjData::geom_xmat
                      0.0f, 1.0f, 0.0f,                          // （行主序）
                      0.0f, 0.0f, 1.0f};
};

// 描边的外观参数。GUI 线程写、渲染线程读（MujocoQuickItem 用互斥量保护）。
struct OutlineStyle {
    QVector4D color {0.0f, 0.0f, 0.0f, 1.0f};
    float widthPx = 1.2f;             // 线宽（像素）
    bool  creases = true;             // 是否画同一物体内部的折痕线
    // 二面角超过该值才认定是折痕。网格模型（STL/STEP 三角化）在兜面/倒角上
    // 面片法线本身有噪声，阈值设低（如 30°）会把面片噪声全画成线，显得很脏；
    // 60° 左右接近 CAD 的"shaded with edges"观感。
    float creaseAngleDeg = 60.0f;
    float depthThreshold = 0.02f;     // 二阶差分的相对阈值（真正的深度断裂）
};
