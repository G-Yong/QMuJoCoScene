#pragma once
// ---------------------------------------------------------------------------
// PointCloudRenderer
//
// 把点云用 GPU 的 GL_POINTS 一次性绘制进 MuJoCo 的离屏帧缓冲（con_.offFBO），
// 支持百万级点：每个点云一个 VBO（位置 + 逐点颜色），一次 draw call。
//
// 关键：必须与 MuJoCo 的渲染保持一致才能正确深度互遮挡。MuJoCo 用 reverse-Z
// （glDepthFunc(GL_GEQUAL) + clearDepth(0) + 投影里翻转 Z）。本渲染器在
// beginFrame() 里完全复制 MuJoCo render_gl3.c 的 setView() 投影/深度约定。
//
// 线程：所有方法都必须在 MuJoCo 渲染线程、且对应的 GL context 已 current 时调用
//       （由 MujocoQuickItem::onRenderOverlay 在 SwapBuffers 内驱动）。
//
// 渲染样式（参考 RViz 点云显示，全部是正对相机的 point sprite）：
//   Pixel  —— 固定屏幕像素大小（最快，pointSize 单位像素）
//   Square —— 世界尺寸方片，随距离透视缩放（pointSize 单位米 = 半边长）
//   Circle —— 世界尺寸圆片
//   Sphere —— 世界尺寸、正对相机、做球面着色（RViz 的 Spheres/Billboards）
// ---------------------------------------------------------------------------

#include <QMatrix4x4>
#include <QVector4D>
#include <QVector3D>
#include <memory>
#include <unordered_map>
#include <vector>

class QOpenGLShaderProgram;
class QOpenGLContext;

class PointCloudRenderer {
public:
    // 与 MujocoQuickItem::PointCloudStyle 对应的整型值（0..3）。
    enum Style {
        StylePixel  = 0,
        StyleSquare = 1,
        StyleCircle = 2,
        StyleSphere = 3
    };

    PointCloudRenderer();
    ~PointCloudRenderer();

    PointCloudRenderer(const PointCloudRenderer&) = delete;
    PointCloudRenderer& operator=(const PointCloudRenderer&) = delete;

    // 上传 / 更新某点云的位置（xyz 扁平，单位米）。count = 点数。
    // data 长度须 >= count*3。会按需创建该 cloudId 的 GPU 资源。
    void uploadPositions(int cloudId, const float* xyz, int count);
    // 上传 / 更新逐点颜色（rgba 扁平，0~1）。count 须与位置点数一致。
    // colors 为空（count<=0）表示清除逐点颜色，绘制时回退到统一颜色。
    void uploadColors(int cloudId, const float* rgba, int count);
    // 删除某点云的 GPU 资源。
    void removeCloud(int cloudId);
    // 删除所有不在 keepIds 中的点云 GPU 资源（清理已移除的点云）。
    void retainOnly(const std::vector<int>& keepIds);

    // 开始一帧：绑定目标 FBO、设置视口、复制 MuJoCo 的投影/视图/深度状态。
    // averagedCamera 的字段来自 mjv_averageCamera 的结果（见 .cpp）。
    void beginFrame(unsigned int targetFbo, int viewportW, int viewportH,
                    const float camPos[3], const float camForward[3],
                    const float camUp[3],
                    float frustumCenter, float frustumWidth,
                    float frustumBottom, float frustumTop,
                    float frustumNear, float frustumFar,
                    bool orthographic,
                    bool sceneTransform, const float translate[3],
                    const float rotateQuat[4], float scale);

    // 绘制一个点云。pointSize：Pixel 样式为像素，其余为世界半径（米）。
    // uniformColor 在该点云没有逐点颜色时使用。
    void drawCloud(int cloudId, int style, float pointSize,
                   const QVector4D& uniformColor);

    // 绘制该点云关于平面 z=planeZ 的地面倒影：点被镜像到平面下方，
    // 透明度乘以 alphaScale 后叠加到已渲染的地面上（深度测试关闭）。
    void drawCloudReflected(int cloudId, int style, float pointSize,
                            const QVector4D& uniformColor,
                            float planeZ, float alphaScale);

    // 结束一帧：恢复必要的 GL 状态（解绑 program / VAO）。
    void endFrame();

    // 在 GL context 仍 current 时释放所有 GPU 资源（renderThread 退出前调用）。
    void releaseGL();

    // 是否已成功初始化 shader（首帧惰性创建）。
    bool ready() const { return m_program != nullptr; }

private:
    struct GpuCloud {
        unsigned int vao        = 0;
        unsigned int posVbo     = 0;
        unsigned int colVbo     = 0;
        int          count      = 0;
        bool         hasColors  = false;
        bool         vaoColorBound = false; // 颜色属性是否已绑进 VAO
    };

    bool ensureProgram();
    GpuCloud& ensureCloud(int cloudId);

    QOpenGLShaderProgram* m_program = nullptr;
    std::unordered_map<int, GpuCloud> m_clouds;

    // 当前帧的矩阵（beginFrame 计算）。
    QMatrix4x4 m_view;
    QMatrix4x4 m_proj;
    int        m_viewportH = 0;
    float      m_projYY    = 1.0f; // proj[1][1]，世界尺寸 → 像素换算用
    QVector3D  m_camPos;           // 当前帧相机世界坐标（倒影投影用）

    // uniform / attrib 位置缓存。
    int m_locMVP   = -1;
    int m_locMV    = -1;
    int m_locStyle = -1;
    int m_locPointSize = -1;
    int m_locViewportH = -1;
    int m_locProjYY    = -1;
    int m_locUseUniformColor = -1;
    int m_locUniformColor    = -1;
    int m_locAlphaScale      = -1;
    int m_locReflect         = -1;
    int m_locCamPos          = -1;
    int m_locPlaneZ          = -1;
};
