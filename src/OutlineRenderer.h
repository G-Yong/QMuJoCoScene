#pragma once
// ---------------------------------------------------------------------------
// OutlineRenderer —— 整场景描边（线稿叠加）
//
// 两阶段（都在 MuJoCo 渲染线程、GL context current 时调用）：
//   buildGeometry()  把当前可见 geom 的视空间法线 + 深度画进自建 G-buffer
//                    （RGBA8 法线 + DEPTH24，尺寸 = 场景视口）。
//   composite()      全屏边缘检测（法线夹角 + 相对深度差），把线混合进
//                    con_.offFBO —— 只加线，不触碰 mjr_render 的光照/材质。
//
// 调用顺序（见 MujocoQuickItem::onRenderOverlay）：
//   mjr_render → buildGeometry → 点云/倒影 → composite → 接触点标记 → gizmo
//
// 设计文档：docs/superpowers/specs/2026-10-22-scene-outline-design.md
// ---------------------------------------------------------------------------

#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <unordered_map>
#include <vector>

#include "OutlineTypes.h"
#include "OverlayCamera.h"

struct mjModel_;
typedef mjModel_ mjModel;

class QOpenGLShaderProgram;

class OutlineRenderer {
public:
    OutlineRenderer();
    ~OutlineRenderer();

    OutlineRenderer(const OutlineRenderer&) = delete;
    OutlineRenderer& operator=(const OutlineRenderer&) = delete;

    // 把可见 geom 的法线/深度写进自建 FBO。不触碰 targetFbo。
    // samples：物理线程采样的全量 geom（索引 = geomId）；visibleMask：按 scn.geoms
    // 过滤后的可见标志（长度 = mjModel::ngeom）。model 只用于读取 mesh 顶点/面。
    void buildGeometry(const OverlayCamera& cam, const mjModel* model,
                       const std::vector<OutlineGeomSample>& samples,
                       const std::vector<unsigned char>& visibleMask);

    // 全屏边缘检测 → 混合进 targetFbo 的场景矩形。之后恢复整幅视口与
    // 后续叠加层（接触点标记 / gizmo）依赖的 GL 状态。
    void composite(unsigned int targetFbo, const OverlayCamera& cam,
                   const OutlineStyle& style);

    // 在 GL context 仍 current 时释放所有 GPU 资源（renderThread 退出前调用）。
    void releaseGL();

    bool ready() const { return m_normalProg != nullptr && m_edgeProg != nullptr; }

private:
    struct MeshGpu {
        unsigned int vao = 0;
        unsigned int vbo = 0;      // 交错 pos(3) + normal(3)
        unsigned int ibo = 0;
        int          indexCount = 0;
    };

    bool ensurePrograms();
    bool ensureTargets(int w, int h);
    // 模型网格（key = meshId）与单位基元（key = kPrimitiveKeyBase + ...）共用缓存。
    MeshGpu& ensureMeshSlot(int key);
    MeshGpu* ensureModelMesh(const mjModel* model, int meshId);
    MeshGpu* ensurePrimitive(int geomType, float capsuleHalfLengthRatio);
    bool uploadMesh(MeshGpu& gpu, const std::vector<float>& interleaved,
                    const std::vector<unsigned int>& indices);
    // 模型重编译/重载后 mesh 数组会换掉：清空模型网格缓存（保留基元）。
    void clearModelMeshes(const mjModel* model);
    void drawObject(const MeshGpu& gpu, const QMatrix4x4& modelM, const OverlayCamera& cam);

    enum { kPrimitiveKeyBase = 1000000 };

    QOpenGLShaderProgram* m_normalProg = nullptr;
    QOpenGLShaderProgram* m_edgeProg   = nullptr;
    // shader 编译/链接失败后锁存，避免每帧重试并重复刷日志。
    bool m_programsFailed = false;

    int m_locNormalMVP         = -1;
    int m_locNormalMat         = -1;

    int m_locEdgeNormalTex     = -1;
    int m_locEdgeDepthTex      = -1;
    int m_locEdgeTexel         = -1;
    int m_locEdgeWidthPx       = -1;
    int m_locEdgeDepthThresh   = -1;
    int m_locEdgeNormalCos     = -1;
    int m_locEdgeCreases       = -1;
    int m_locEdgeColor         = -1;
    int m_locEdgeNear          = -1;
    int m_locEdgeFar           = -1;
    int m_locEdgeOrtho         = -1;

    std::unordered_map<int, MeshGpu> m_meshes;   // key: meshId / kPrimitiveKeyBase+...
    const mjModel* m_cachedModel = nullptr;
    int            m_cachedNmesh = -1;
    const float*   m_cachedMeshVert = nullptr;

    int          m_gbufW = 0;
    int          m_gbufH = 0;
    unsigned int m_gbufFbo      = 0;
    unsigned int m_gbufColorTex = 0;
    unsigned int m_gbufDepthTex = 0;
    unsigned int m_dummyVao     = 0;   // 全屏三角形
};
