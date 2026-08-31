#include "MujocoQuickItem.h"
#include "MujocoQuickItemHelpers.h"
#include "QtPlatformUIAdapter.h"
#include "PointCloudRenderer.h"

#include "simulate.h"
#include <mujoco/mujoco.h>
#include <mujoco/mjui.h>

#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QQuickWindow>
#include <QMouseEvent>
#include <QHoverEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QGuiApplication>
#include <QScreen>
#include <QDebug>
#include <QMetaObject>
#include <QThread>
#include <QSGSimpleTextureNode>
#include <QFileInfo>
#include <QTemporaryFile>
#include <QDir>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <vector>

namespace {
constexpr double  syncMisalign       = 0.1;
constexpr double  simRefreshFraction = 0.7;
constexpr int     kErrorLength       = 1024;
using Seconds = std::chrono::duration<double>;
} // namespace

// 把辅助函数 / 结构暴露给本翻译单元（包括下面的 static 自由函数与
// MujocoQuickItem 成员函数实现）。
using namespace mqi_detail;

// 前向声明：锁内把控制值写穿到 d->ctrl（定义在文件后段）。
static inline void applyControlImmediately(mujoco::Simulate& sim, int id, double value);

// 缓入缓出函数（cubic ease-in-out），用于相机过渡。
static float easeInOutCubic(float t) {
    return t < 0.5f ? 4.0f * t * t * t
                     : 1.0f - std::pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
}

// ===========================================================================
// MujocoFboRenderer：Qt Quick scenegraph 渲染线程
// ===========================================================================
namespace {
class MujocoFboRenderer : public QQuickFramebufferObject::Renderer {
public:
    explicit MujocoFboRenderer() = default;

    QOpenGLFramebufferObject* createFramebufferObject(const QSize& size) override {
        QOpenGLFramebufferObjectFormat fmt;
        // 我们只需要颜色 attachment：blit 不需要深度 / 模板，
        // 去掉可以节省大窗口下的显存带宽。
        fmt.setAttachment(QOpenGLFramebufferObject::NoAttachment);
        fmt.setSamples(0);
        return new QOpenGLFramebufferObject(size, fmt);
    }

    void synchronize(QQuickFramebufferObject* qitem) override {
        m_item = static_cast<MujocoQuickItem*>(qitem);
    }

    void render() override {
        if (!m_item) return;
        unsigned int srcTex = m_item->currentSourceTexture();
        QSize srcSize       = m_item->currentSourceSize();
        QOpenGLFramebufferObject* dst = framebufferObject();
        if (!dst) return;

        auto* glctx = QOpenGLContext::currentContext();
        if (!glctx) return;
        QOpenGLExtraFunctions* gl = glctx->extraFunctions();

        // 即便还没有源纹理，也至少把 Quick FBO 清成不透明背景，
        // 避免出现未定义内容
        gl->glViewport(0, 0, dst->width(), dst->height());
        gl->glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
        gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!srcTex || srcSize.isEmpty()) return;

        // 用一个临时 read FBO 把共享纹理 attach 上来，
        // blit 到 Quick 提供的 draw FBO
        if (!m_readFbo) {
            gl->glGenFramebuffers(1, &m_readFbo);
        }
        gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, m_readFbo);
        gl->glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, srcTex, 0);
        gl->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dst->handle());
        gl->glBlitFramebuffer(0, 0, srcSize.width(), srcSize.height(),
                              0, 0, dst->width(), dst->height(),
                              GL_COLOR_BUFFER_BIT, GL_LINEAR);
        // 解绑，避免影响 Quick 后续状态
        gl->glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        gl->glBindFramebuffer(GL_FRAMEBUFFER, dst->handle());
        m_item->window()->resetOpenGLState();

        // 告知 mujoco 渲染线程：共享纹理已被取走，可以生成下一帧。
        m_item->notifyFrameConsumed();
    }

    ~MujocoFboRenderer() override {
        if (m_readFbo && QOpenGLContext::currentContext()) {
            QOpenGLContext::currentContext()->extraFunctions()
                ->glDeleteFramebuffers(1, &m_readFbo);
        }
    }

private:
    MujocoQuickItem* m_item = nullptr;
    unsigned int     m_readFbo = 0;
};
} // namespace

// ===========================================================================
// MujocoQuickItem
// ===========================================================================
MujocoQuickItem::MujocoQuickItem(QQuickItem* parent)
    : QQuickFramebufferObject(parent) {
    qRegisterMetaType<JointInfo>();
    qRegisterMetaType<ActuatorInfo>();
    qRegisterMetaType<SceneObjectInfo>();
    qRegisterMetaType<ContactInfo>();
    qRegisterMetaType<CameraState>();
    setMirrorVertically(true); // mjr 是 OpenGL bottom-up，Quick 绘制时翻一下
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptHoverEvents(true);
    setFlag(ItemHasContents, true);
    setActiveFocusOnTab(true);
}

MujocoQuickItem::~MujocoQuickItem() {
    // 若本实例仍持有外部窄阶段接入，先注销（并在本实例是最后一个时
    // 恢复全局碰撞函数表），避免 closeScene 之后留下指向已析构实例的全局回调。
    setExternalNarrowPhase(nullptr, nullptr);
    closeScene();
}

QQuickFramebufferObject::Renderer* MujocoQuickItem::createRenderer() const {
    return new MujocoFboRenderer();
}

void MujocoQuickItem::setXmlPath(const QString& path) {
    if (m_xmlPath == path) return;
    m_xmlPath = path;
    emit xmlPathChanged();
    if (m_running.load()) loadScene(path);
}

unsigned int MujocoQuickItem::currentSourceTexture() const {
    return m_adapterRaw ? m_adapterRaw->offscreenColorTexture() : 0u;
}
QSize MujocoQuickItem::currentSourceSize() const {
    if (!m_adapterRaw) return {};
    return {m_adapterRaw->offscreenWidth(), m_adapterRaw->offscreenHeight()};
}

void MujocoQuickItem::notifyFrameConsumed() {
    if (m_adapterRaw) m_adapterRaw->NotifyConsumed();
}

// ----------------------------------------------------------------- 场景生命周期 ----
void MujocoQuickItem::setLastError(const QString& err) {
    std::lock_guard<std::mutex> lk(m_errorMtx);
    m_lastError = err;
}

QString MujocoQuickItem::lastError() const {
    std::lock_guard<std::mutex> lk(m_errorMtx);
    return m_lastError;
}

bool MujocoQuickItem::ensureBackendStarted() {
    if (m_running.exchange(true)) {
        return true; // 已启动
    }

    // 1) 离屏 surface（QWindow-less 渲染）
    m_surface = new QOffscreenSurface();
    QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
    if (fmt.majorVersion() < 3) {
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
    }
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    m_surface->setFormat(fmt);
    m_surface->create();

    // 2) GL context，必须 Qt 主线程创建；与 Qt Quick 共享 (Qt::AA_ShareOpenGLContexts)
    m_ctx = new QOpenGLContext();
    m_ctx->setFormat(fmt);
    m_ctx->setShareContext(QOpenGLContext::globalShareContext());
    if (!m_ctx->create()) {
        const QString err = QStringLiteral("GL context create failed");
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        delete m_ctx; m_ctx = nullptr;
        if (m_surface) { m_surface->destroy(); delete m_surface; m_surface = nullptr; }
        m_running.store(false);
        return false;
    }

    // 推到渲染线程（通过先释放线程亲和性到 nullptr，再由渲染线程接手）
    m_ctx->moveToThread(nullptr);
    m_surface->moveToThread(nullptr);

    updateGeometryToAdapter();

    m_renderThread  = std::thread(&MujocoQuickItem::renderThreadMain,  this);
    m_physicsThread = std::thread(&MujocoQuickItem::physicsThreadMain, this);

    return true;
}

void MujocoQuickItem::closeScene() {
    if (!m_running.exchange(false)) return;

    if (m_sim) m_sim->exitrequest.store(1);
    if (m_adapterRaw) m_adapterRaw->PostClose();

    if (m_physicsThread.joinable()) m_physicsThread.join();
    if (m_renderThread.joinable())  m_renderThread.join();

    if (m_userScene) {
        mjv_freeScene(m_userScene);
        delete m_userScene;
        m_userScene = nullptr;
    }
    m_trajectories.clear();
    m_staticVisualGeomCount = 0;
    {
        std::lock_guard<std::mutex> lk(m_pointCloudMtx);
        m_pointClouds.clear();
    }

    m_sim.reset();
    // 渲染线程退出前已把 m_ctx / m_surface 的线程亲和性释放为 nullptr，
    // 这里可以安全地从当前（主）线程拾起、销毁。
    if (m_ctx) {
        m_ctx->moveToThread(QThread::currentThread());
        delete m_ctx;
        m_ctx = nullptr;
    }
    if (m_surface) {
        m_surface->moveToThread(QThread::currentThread());
        m_surface->destroy();
        delete m_surface;
        m_surface = nullptr;
    }
    m_adapterRaw = nullptr;

    // 清理待加载请求和临时文件
    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile.clear();
    }
    m_hasPendingLoad.store(false);
    m_tempSceneFile.reset();
}

bool MujocoQuickItem::loadScene(const QString& filename) {
    QFileInfo checkFile(filename);
    if (filename.isEmpty() || !checkFile.exists() || !checkFile.isFile()) {
        const QString err = QStringLiteral("Scene file does not exist: %1").arg(filename);
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    if (!ensureBackendStarted()) {
        emit sceneLoadFailed(lastError());
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = filename;
    }
    m_hasPendingLoad.store(true);
    return true;
}

bool MujocoQuickItem::loadSceneFromData(const QByteArray& data, const QString& format) {
    if (data.isEmpty()) {
        const QString err = QStringLiteral("Scene data is empty");
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    QString suffix = format.trimmed().toLower();
    if (suffix.startsWith(QLatin1Char('.'))) suffix.remove(0, 1);
    if (suffix != QLatin1String("xml") && suffix != QLatin1String("mjb")) {
        const QString err = QStringLiteral("Unsupported scene format: %1 (expected 'xml' or 'mjb')").arg(format);
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }

    // 写入临时文件 (保留实例存活，以便 mujoco 异步加载期间文件不会被删除)
    auto tmp = std::unique_ptr<QTemporaryFile>(new QTemporaryFile(
        QDir::tempPath() + QStringLiteral("/mujoco_scene_XXXXXX.") + suffix));
    tmp->setAutoRemove(true);
    if (!tmp->open()) {
        const QString err = QStringLiteral("Failed to create temporary scene file: %1").arg(tmp->errorString());
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }
    if (tmp->write(data) != data.size()) {
        const QString err = QStringLiteral("Failed to write scene data to temporary file: %1").arg(tmp->errorString());
        qWarning() << "MujocoQuickItem:" << err;
        setLastError(err);
        emit sceneLoadFailed(err);
        return false;
    }
    tmp->flush();
    const QString tmpPath = tmp->fileName();
    tmp->close();

    if (!ensureBackendStarted()) {
        emit sceneLoadFailed(lastError());
        return false;
    }

    // 保活：新临时文件覆盖之前的，物理线程加载完成后旧文件可被回收。
    m_tempSceneFile = std::move(tmp);

    {
        std::lock_guard<std::mutex> lk(m_pendingMtx);
        m_pendingFile = tmpPath;
    }
    m_hasPendingLoad.store(true);
    return true;
}

void MujocoQuickItem::withSimulation(std::function<void(const mjModel*, mjData*)> callback) const {
    if (!m_sim) return;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    if (!m_sim->m_ || !m_sim->d_) return;
    callback(m_sim->m_, m_sim->d_);
}

void MujocoQuickItem::withMutableSimulation(std::function<void(mjModel*, mjData*)> callback) {
    if (!m_sim) return;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    if (!m_sim->m_ || !m_sim->d_) return;
    callback(m_sim->m_, m_sim->d_);
    lk.unlock();
    requestRenderUpdate();
}

namespace {
qint64 mqiNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

bool MujocoQuickItem::isSettledLocked(const mjModel* m, mjData* d, double posTol, double velTol)
{
    // 所有关节速度趋零
    for (int i = 0; i < m->nv; ++i)
        if (std::abs(d->qvel[i]) > velTol) return false;
    // 被驱动关节位置收敛到 ctrl 目标（position 伺服：ctrl = gear * qpos）
    for (int a = 0; a < m->nu; ++a) {
        if (m->actuator_trntype[a] != mjTRN_JOINT)
            continue;
        const int jid = m->actuator_trnid[2 * a];
        if (jid < 0 || jid >= m->njnt)
            continue;
        const int type = m->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            continue;   // 只检查单自由度被驱动关节
        const double target = d->ctrl[a];
        const double cur = d->qpos[m->jnt_qposadr[jid]] * m->actuator_gear[6 * a];
        if (std::abs(cur - target) > posTol) return false;
    }
    return true;
}

bool MujocoQuickItem::isSettled(double posTol, double velTol) const
{
    bool settled = true;
    withSimulation([&](const mjModel* m, mjData* d) {
        settled = isSettledLocked(m, d, posTol, velTol);
    });
    return settled;
}

void MujocoQuickItem::stopWhenSettled(double posTol, double velTol, int stableMs, int timeoutMs)
{
    if (!m_sim) return;
    std::lock_guard<std::recursive_mutex> lk(m_sim->mtx);
    if (m_sim->run == 0) return;   // 已停则无需等待
    m_settleStop.active    = true;
    m_settleStop.posTol    = posTol;
    m_settleStop.velTol    = velTol;
    m_settleStop.stableMs  = stableMs;
    m_settleStop.timeoutMs = timeoutMs;
    m_settleStop.armMs     = mqiNowMs();
    m_settleStop.settledMs = -1;
}

void MujocoQuickItem::cancelStopWhenSettled()
{
    if (!m_sim) return;
    std::lock_guard<std::recursive_mutex> lk(m_sim->mtx);
    m_settleStop.active    = false;
    m_settleStop.settledMs = -1;
}

void MujocoQuickItem::requestRenderUpdate() {
    if (QThread::currentThread() == thread()) {
        update();
    } else {
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    }
}

void MujocoQuickItem::applyBooleanPropertiesTo(mujoco::Simulate& sim) {
    sim.run = boolToInt(m_simulationRunning.load());
    sim.help = boolToInt(m_helpVisible.load());
    sim.info = boolToInt(m_infoVisible.load());
    sim.profiler = boolToInt(m_profilerVisible.load());
    sim.sensor = boolToInt(m_sensorVisible.load());
    sim.pause_update = boolToInt(m_pauseUpdateEnabled.load());
    sim.busywait = boolToInt(m_busyWaitEnabled.load());
    sim.ui0_enable = boolToInt(m_leftUiVisible.load());
    sim.ui1_enable = boolToInt(m_rightUiVisible.load());
    sim.status_overlay = boolToInt(m_statusOverlayVisible.load());

    const int fullscreen = boolToInt(m_fullscreenRequested.load());
    if (sim.fullscreen != fullscreen && sim.platform_ui) {
        sim.platform_ui->ToggleFullscreen();
    }
    sim.fullscreen = fullscreen;

    sim.vsync = boolToInt(m_vSyncEnabled.load());
    if (sim.platform_ui) sim.platform_ui->SetVSync(sim.vsync);

    sim.pending_.ui_update_simulation = true;
    sim.pending_.ui_update_rendering = true;
}

bool MujocoQuickItem::withSimulateLocked(const std::function<void(mujoco::Simulate&)>& callback) {
    if (!m_sim) return false;
    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        callback(*m_sim);
    }
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::ensureUserSceneLocked(mujoco::Simulate& sim) {
    if (!m_userScene) {
        m_userScene = new mjvScene;
        mjv_defaultScene(m_userScene);
        mjv_makeScene(nullptr, m_userScene, mujoco::Simulate::kMaxGeom);
    }
    if (!m_userScene->geoms || m_userScene->maxgeom <= 0) {
        setLastError(QStringLiteral("Failed to create MuJoCo user scene"));
        return false;
    }
    sim.user_scn = m_userScene;
    return true;
}

void MujocoQuickItem::setSimulationRunning(bool running) {
    if (m_simulationRunning.exchange(running) == running) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.run = boolToInt(running);
        if (sim.run) {
            sim.scrub_index = 0;
            sim.pert.active = 0;
        }
        sim.pending_.ui_update_simulation = true;
    });
    emit simulationRunningChanged();
}

bool MujocoQuickItem::toggleSimulationRunning() {
    setSimulationRunning(!simulationRunning());
    return true;
}

bool MujocoQuickItem::stepSimulationForward() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || sim.run) return;
        if (sim.scrub_index < 0) {
            sim.scrub_index++;
            sim.pending_.load_from_history = true;
            sim.pending_.ui_update_simulation = true;
        } else {
            mj_step(sim.m_, sim.d_);
            sim.AddToHistory();
            bumpHistoryDepth(sim);
        }
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::stepSimulationBackward() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_) return;
        sim.run = 0;
        // 硬件下界 1-nhistory_，同时不得回退到尚未记录的区间（-historyDepth）。
        const int lower = mjMAX(1 - sim.nhistory_, -m_historyDepth.load());
        sim.scrub_index = mjMAX(sim.scrub_index - 1, lower);
        sim.pending_.load_from_history = true;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    if (applied && m_simulationRunning.exchange(false)) {
        emit simulationRunningChanged();
    }
    return applied;
}

bool MujocoQuickItem::seekHistory(int scrubIndex) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_) return;
        // 夹取到 [lower, 0]：lower 取硬件下界与已记录深度中更靠近 0 的一侧。
        const int lower = mjMAX(1 - sim.nhistory_, -m_historyDepth.load());
        int idx = scrubIndex;
        if (idx > 0)     idx = 0;
        if (idx < lower) idx = lower;
        sim.run = 0;
        sim.scrub_index = idx;
        sim.pending_.load_from_history = true;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    if (applied && m_simulationRunning.exchange(false)) {
        emit simulationRunningChanged();
    }
    return applied;
}

void MujocoQuickItem::bumpHistoryDepth(const mujoco::Simulate& sim) {
    const int cap = sim.nhistory_ > 0 ? sim.nhistory_ - 1 : 0;
    const int d = m_historyDepth.load();
    if (d < cap) m_historyDepth.store(d + 1);
}

bool MujocoQuickItem::resetSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_) return;
        sim.pending_.reset = true;
        sim.scrub_index = 0;
        m_historyDepth.store(0);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::resetSimulationToState(const QVariantMap& jointValues,
                                             const QVariantMap& controlValues) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_) return;

        // 1) 全量物理复位（等价 resetSimulation 的 mj_resetData 语义）。
        mj_resetData(sim.m_, sim.d_);
        sim.scrub_index = 0;
        m_historyDepth.store(0);

        // 2) 同步 Simulate 的 qpos/ctrl 影子缓存，避免下一次 Sync 用旧缓存回写。
        resyncSimulateQposCaches(sim);
        const int nu = static_cast<int>(sim.m_->nu);
        for (int i = 0; i < nu; ++i)
            applyControlImmediately(sim, i, sim.d_->ctrl[i]);

        // 3) 覆盖指定关节的 qpos（按名，仅 hinge/slide）。
        for (auto it = jointValues.cbegin(); it != jointValues.cend(); ++it) {
            const int id = mj_name2id(sim.m_, mjOBJ_JOINT, it.key().toUtf8().constData());
            if (!isValidIndex(id, static_cast<int>(sim.m_->njnt))) continue;
            const int type = sim.m_->jnt_type[id];
            if (type != mjJNT_SLIDE && type != mjJNT_HINGE) continue;
            setHingeJointValue(sim.m_, sim.d_, sim.qpos_, sim.qpos_prev_, id,
                               it.value().toDouble());
        }

        // 4) 覆盖指定执行器的 ctrl（按名），让位置伺服保持在目标姿态。
        for (auto it = controlValues.cbegin(); it != controlValues.cend(); ++it) {
            const int id = mj_name2id(sim.m_, mjOBJ_ACTUATOR, it.key().toUtf8().constData());
            if (!isValidIndex(id, nu)) continue;
            applyControlImmediately(sim, id, it.value().toDouble());
        }

        // 5) 前向运动学刷新世界位姿，标记 UI 重绘。
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::zeroControls() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_) return;
        sim.pending_.zero_ctrl = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setKeyframeIndex(int index) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !isValidIndex(index, sim.nkey_)) return;
        sim.key = index;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::loadKeyframe() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || !isValidIndex(sim.key, sim.nkey_)) return;
        sim.pending_.load_key = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveKeyframe() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || !sim.m_ || !sim.d_ || !isValidIndex(sim.key, sim.nkey_)) return;
        sim.pending_.save_key = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveSceneAsXml(const QString& filename) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.save_xml = filename.toStdString();
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::saveSceneAsMjb(const QString& filename) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.save_mjb = filename.toStdString();
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::addPrimitive(PrimitiveType type,
                                   const QVector3D& position,
                                   const QVector3D& size,
                                   double mass,
                                   bool freeJoint,
                                   const QString& name) {
    QVariantList types;
    QVariantList positions;
    QVariantList sizes;
    types.append(static_cast<int>(type));
    positions.append(QVariant::fromValue(position));
    sizes.append(QVariant::fromValue(size));

    const QVariantList bodyIds = addPrimitiveRequests(types, positions, sizes,
                                                      mass, freeJoint, name, true);
    return !bodyIds.isEmpty();
}

QVariantList MujocoQuickItem::addPrimitives(const QVariantList& types,
                                            const QVariantList& positions,
                                            const QVariantList& sizes,
                                            double mass,
                                            bool freeJoint,
                                            const QString& namePrefix)
{
    return addPrimitiveRequests(types, positions, sizes,
                                mass, freeJoint, namePrefix, false);
}

QVariantList MujocoQuickItem::addPrimitiveRequests(const QVariantList& types,
                                                   const QVariantList& positions,
                                                   const QVariantList& sizes,
                                                   double mass,
                                                   bool freeJoint,
                                                   const QString& namePrefix,
                                                   bool useExactSingleName)
{
    std::vector<PrimitiveRequest> requests;
    QString error;
    const QString effectivePrefix = namePrefix.trimmed().isEmpty() ? QStringLiteral("primitive") : namePrefix.trimmed();
    if (!buildPrimitiveRequests(types, positions, sizes, QVariantList(),
                                QVector4D(0.2f, 0.6f, 0.9f, 1.0f),
                                effectivePrefix, &requests, &error)) {
        setLastError(error);
        return {};
    }
    if (useExactSingleName && requests.size() == 1) {
        requests[0].name = namePrefix.trimmed();
    }
    if (requests.empty()) {
        setLastError(QString());
        return {};
    }

    if (!m_sim) {
        setLastError(QStringLiteral("Scene is not loaded"));
        return {};
    }

    mjModel* modelForReload = nullptr;
    mjData* dataForReload = nullptr;
    QByteArray displayedFilename;
    std::vector<QString> addedBodyNames;
    QVariantList bodyIds;

    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        if (!m_sim->m_ || !m_sim->d_) {
            setLastError(QStringLiteral("Scene is not loaded"));
            return {};
        }
        if (!m_editSpec) {
            setLastError(QStringLiteral("Current scene is not editable; load an XML scene before adding primitives"));
            return {};
        }

        mjSpec* spec = mj_copySpec(m_editSpec);
        if (!spec) {
            setLastError(QStringLiteral("Failed to copy MuJoCo spec"));
            return {};
        }

        if (!mj_copyBack(spec, m_sim->m_)) {
            mj_deleteSpec(spec);
            setLastError(QStringLiteral("Failed to copy current model values back to MuJoCo spec"));
            return {};
        }

        mjsBody* world = mjs_findBody(spec, "world");
        if (!world) {
            mj_deleteSpec(spec);
            setLastError(QStringLiteral("Failed to find MuJoCo world body in spec"));
            return {};
        }

        addedBodyNames.reserve(requests.size());
        for (size_t i = 0; i < requests.size(); ++i) {
            const PrimitiveRequest& request = requests[i];
            const QString baseName = request.name.trimmed().isEmpty()
                ? QStringLiteral("primitive_%1").arg(m_sim->m_->nbody + static_cast<int>(i))
                : request.name;
            const QString bodyName = uniqueObjectName(m_sim->m_, addedBodyNames, mjOBJ_BODY, baseName);
            const QString geomName = uniqueObjectName(m_sim->m_, {}, mjOBJ_GEOM, bodyName + QStringLiteral("_geom"));

            mjsBody* body = mjs_addBody(world, nullptr);
            mjsGeom* geom = body ? mjs_addGeom(body, nullptr) : nullptr;
            if (!body || !geom) {
                mj_deleteSpec(spec);
                setLastError(QStringLiteral("Failed to add primitive body or geom"));
                return {};
            }

            const QByteArray bodyNameUtf8 = bodyName.toUtf8();
            const QByteArray geomNameUtf8 = geomName.toUtf8();
            mjs_setName(body->element, bodyNameUtf8.constData());
            mjs_setName(geom->element, geomNameUtf8.constData());

            copyVec3(body->pos, request.position);
            geom->type = static_cast<mjtGeom>(request.geomType);
            setPrimitiveSize(geom, request.geomType, request.size);
            if (mass > 0.0) geom->mass = mass;
            geom->rgba[0] = request.rgba.x();
            geom->rgba[1] = request.rgba.y();
            geom->rgba[2] = request.rgba.z();
            geom->rgba[3] = request.rgba.w();

            if (freeJoint && !mjs_addFreeJoint(body)) {
                mj_deleteSpec(spec);
                setLastError(QStringLiteral("Failed to add free joint to primitive body"));
                return {};
            }
            addedBodyNames.push_back(bodyName);
        }

        const int rc = mj_recompile(spec, nullptr, m_sim->m_, m_sim->d_);
        if (rc != 0) {
            const char* err = mjs_getError(spec);
            const QString reason = err && err[0]
                ? QString::fromUtf8(err)
                : QStringLiteral("Failed to recompile MuJoCo model after adding primitives");
            mj_deleteSpec(spec);
            setLastError(reason);
            return {};
        }

        mj_deleteSpec(m_editSpec);
        m_editSpec = spec;
        // mj_recompile 之后 sim.qpos_ / qpos_prev_ 仍然保持旧 nq 大小，
        // 必须立刻 resize 到新 nq 并从 d->qpos 同步：否则 Simulate::Sync()
        // 在 [old_nq, new_nq) 范围会读到越界内存（可能为 0），并把它写回
        // d->qpos，从而把刚加入的 free-joint body 的位置 "清零" 到原点。
        resyncSimulateQposCaches(*m_sim);
        for (size_t i = 0; i < addedBodyNames.size(); ++i) {
            const int addedBodyId = mj_name2id(m_sim->m_, mjOBJ_BODY, addedBodyNames[i].toUtf8().constData());
            bodyIds.append(addedBodyId);
            const int freeJointId = freeJointIndexForBody(m_sim->m_, addedBodyId);
            if (freeJointId >= 0) {
                setFreeJointPosition(m_sim->m_, m_sim->d_,
                                     m_sim->qpos_, m_sim->qpos_prev_,
                                     freeJointId, requests[i].position);
            }
        }
        mj_forward(m_sim->m_, m_sim->d_);
        markUiRefresh(*m_sim);
        modelForReload = m_sim->m_;
        dataForReload = m_sim->d_;
        displayedFilename = QByteArray(m_sim->filename);
    }

    if (modelForReload && dataForReload) {
        m_sim->Load(modelForReload, dataForReload, displayedFilename.constData());
    }
    setLastError(QString());
    requestRenderUpdate();
    return bodyIds;
}

int MujocoQuickItem::addVisualPrimitive(PrimitiveType type,
                                        const QVector3D& position,
                                        const QVector3D& size,
                                        const QVector4D& rgba) {
    const int geomType = primitiveGeomType(type);
    if (geomType == mjGEOM_NONE) {
        setLastError(QStringLiteral("Unsupported primitive type: %1").arg(primitiveTypeName(type)));
        return -1;
    }

    if (!m_sim) {
        setLastError(QStringLiteral("Scene is not loaded"));
        return -1;
    }

    int addedIndex = -1;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!ensureUserSceneLocked(sim)) return;
        if (m_staticVisualGeomCount >= m_userScene->maxgeom) {
            setLastError(QStringLiteral("MuJoCo user scene geom buffer is full"));
            return;
        }

        mjtNum geomSize[3] = {};
        mjtNum geomPos[3] = {};
        const mjtNum identity[9] = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        const float color[4] = {rgba.x(), rgba.y(), rgba.z(), rgba.w()};
        fillPrimitiveSize(geomSize, geomType, size);
        copyVec3(geomPos, position);

        addedIndex = m_staticVisualGeomCount++;
        mjv_initGeom(m_userScene->geoms + addedIndex,
                     geomType, geomSize, geomPos, identity, color);
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        setLastError(QString());
    });
    return addedIndex;
}

QVariantList MujocoQuickItem::addVisualPrimitives(const QVariantList& types,
                                                  const QVariantList& positions,
                                                  const QVariantList& sizes,
                                                  const QVariantList& rgba) {
    std::vector<PrimitiveRequest> requests;
    QString error;
    if (!buildPrimitiveRequests(types, positions, sizes, rgba,
                                QVector4D(0.2f, 0.6f, 0.9f, 1.0f),
                                QString(), &requests, &error)) {
        setLastError(error);
        return {};
    }
    if (requests.empty()) {
        setLastError(QString());
        return {};
    }

    if (!m_sim) {
        setLastError(QStringLiteral("Scene is not loaded"));
        return {};
    }

    QVariantList addedIndices;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!ensureUserSceneLocked(sim)) return;
        if (requests.size() > static_cast<size_t>(m_userScene->maxgeom - m_staticVisualGeomCount)) {
            setLastError(QStringLiteral("MuJoCo user scene geom buffer is full"));
            return;
        }

        const mjtNum identity[9] = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        for (const PrimitiveRequest& request : requests) {
            mjtNum geomSize[3] = {};
            mjtNum geomPos[3] = {};
            const float color[4] = {request.rgba.x(), request.rgba.y(), request.rgba.z(), request.rgba.w()};
            fillPrimitiveSize(geomSize, request.geomType, request.size);
            copyVec3(geomPos, request.position);

            const int addedIndex = m_staticVisualGeomCount++;
            mjv_initGeom(m_userScene->geoms + addedIndex,
                         request.geomType, geomSize, geomPos, identity, color);
            addedIndices.append(addedIndex);
        }
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        setLastError(QString());
    });
    return addedIndices;
}

int MujocoQuickItem::visualPrimitiveCount() const
{
    int count = 0;
    if (!m_sim) return count;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    count = m_staticVisualGeomCount;
    return count;
}

bool MujocoQuickItem::setVisualPrimitivePosition(int index, const QVector3D& position)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!m_userScene || index < 0 || index >= m_staticVisualGeomCount) return;
        copyVec3(m_userScene->geoms[index].pos, position);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setVisualPrimitiveSize(int index, const QVector3D& size)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!m_userScene || index < 0 || index >= m_staticVisualGeomCount) return;
        fillPrimitiveSize(m_userScene->geoms[index].size,
                          m_userScene->geoms[index].type,
                          size);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

void MujocoQuickItem::clearVisualPrimitives()
{
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!m_userScene) return;
        m_staticVisualGeomCount = 0;
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        setLastError(QString());
    });
}

// ===========================================================================
// 轨迹（尾迹）实现
// ===========================================================================

MujocoQuickItem::TrajectoryState* MujocoQuickItem::findTrajectory(int trajectoryId) {
    for (TrajectoryState& t : m_trajectories) {
        if (t.id == trajectoryId) return &t;
    }
    return nullptr;
}

const MujocoQuickItem::TrajectoryState* MujocoQuickItem::findTrajectory(int trajectoryId) const {
    for (const TrajectoryState& t : m_trajectories) {
        if (t.id == trajectoryId) return &t;
    }
    return nullptr;
}

void MujocoQuickItem::rebuildTrajectoryGeomsLocked() {
    if (!m_userScene || !m_userScene->geoms) return;
    int g = m_staticVisualGeomCount;
    const int maxg = m_userScene->maxgeom;
    for (const TrajectoryState& t : m_trajectories) {
        if (!t.visible || t.points.size() < 2) continue;
        const int geomType = t.useLine ? mjGEOM_LINE : mjGEOM_CAPSULE;
        const float color[4] = {t.rgba.x(), t.rgba.y(), t.rgba.z(), t.rgba.w()};
        for (size_t i = 1; i < t.points.size(); ++i) {
            if (g >= maxg) break;
            mjvGeom* geom = &m_userScene->geoms[g];
            mjv_initGeom(geom, geomType, nullptr, nullptr, nullptr, color);
            mjtNum from[3] = { t.points[i-1].x(), t.points[i-1].y(), t.points[i-1].z() };
            mjtNum to[3]   = { t.points[i].x(),   t.points[i].y(),   t.points[i].z()   };
            mjv_connector(geom, geomType, t.width, from, to);
            ++g;
        }
        if (g >= maxg) break;
    }
    m_userScene->ngeom = g;
}

// ===========================================================================
// 点云实现（GPU GL_POINTS 叠加层）
// ===========================================================================
// CPU 侧只维护扁平 float 缓冲与样式/颜色等参数（受 m_pointCloudMtx 保护）；
// 真正的 GPU 上传与绘制在渲染线程的 onRenderOverlay() 中完成。

MujocoQuickItem::PointCloudState* MujocoQuickItem::findPointCloud(int cloudId) {
    for (PointCloudState& pc : m_pointClouds) {
        if (pc.id == cloudId) return &pc;
    }
    return nullptr;
}

const MujocoQuickItem::PointCloudState* MujocoQuickItem::findPointCloud(int cloudId) const {
    for (const PointCloudState& pc : m_pointClouds) {
        if (pc.id == cloudId) return &pc;
    }
    return nullptr;
}

int MujocoQuickItem::addPointCloud(const QVariant& points,
                                   float pointSize,
                                   PointCloudStyle style,
                                   const QVector4D& rgba) {
    std::vector<QVector3D> parsed;
    if (!variantToPointList(points, &parsed)) {
        setLastError(QStringLiteral("Invalid point cloud data"));
        return -1;
    }
    if (pointSize <= 0.0f)
        pointSize = (style == PointStylePixel) ? 3.0f : 0.01f;

    int newId = -1;
    {
        std::lock_guard<std::mutex> lk(m_pointCloudMtx);
        PointCloudState pc;
        pc.id        = m_nextPointCloudId++;
        pc.style     = static_cast<int>(style);
        pc.pointSize = pointSize;
        pc.rgba      = rgba;
        pc.positions.reserve(parsed.size() * 3);
        for (const QVector3D& p : parsed) {
            pc.positions.push_back(p.x());
            pc.positions.push_back(p.y());
            pc.positions.push_back(p.z());
        }
        newId = pc.id;
        m_pointClouds.push_back(std::move(pc));
    }
    setLastError(QString());
    requestRenderUpdate();
    return newId;
}

void MujocoQuickItem::setPointCloudData(int cloudId,
                                        const QVector<float>& positions,
                                        const QVector<float>& colors) {
    if (positions.size() % 3 != 0) {
        setLastError(QStringLiteral("Point cloud positions size must be a multiple of 3"));
        return;
    }
    const int count = positions.size() / 3;
    if (!colors.isEmpty() && colors.size() != count * 4) {
        setLastError(QStringLiteral("Point cloud colors size (%1) must be point count*4 (%2)")
                     .arg(colors.size()).arg(count * 4));
        return;
    }
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return;
    pc->positions.assign(positions.begin(), positions.end());
    pc->colors.assign(colors.begin(), colors.end());
    pc->dirtyPositions = true;
    pc->dirtyColors = true;
    setLastError(QString());
    requestRenderUpdate();
}

bool MujocoQuickItem::removePointCloud(int cloudId) {
    bool removed = false;
    {
        std::lock_guard<std::mutex> lk(m_pointCloudMtx);
        for (auto it = m_pointClouds.begin(); it != m_pointClouds.end(); ++it) {
            if (it->id == cloudId) {
                m_pointClouds.erase(it);
                removed = true;
                break;
            }
        }
    }
    if (removed) requestRenderUpdate();
    return removed;
}

void MujocoQuickItem::clearPointClouds() {
    {
        std::lock_guard<std::mutex> lk(m_pointCloudMtx);
        if (m_pointClouds.empty()) return;
        m_pointClouds.clear();
    }
    requestRenderUpdate();
}

bool MujocoQuickItem::updatePointCloudPoints(int cloudId, const QVariant& points) {
    std::vector<QVector3D> parsed;
    if (!variantToPointList(points, &parsed)) {
        setLastError(QStringLiteral("Invalid point cloud data"));
        return false;
    }
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->positions.resize(parsed.size() * 3);
    for (size_t i = 0; i < parsed.size(); ++i) {
        pc->positions[3 * i + 0] = parsed[i].x();
        pc->positions[3 * i + 1] = parsed[i].y();
        pc->positions[3 * i + 2] = parsed[i].z();
    }
    // 点数变化时旧的逐点颜色不再匹配，回退到统一颜色。
    if (pc->colors.size() != parsed.size() * 4) {
        pc->colors.clear();
        pc->dirtyColors = true;
    }
    pc->dirtyPositions = true;
    setLastError(QString());
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudColors(int cloudId, const QVariant& colors) {
    std::vector<QVector4D> parsed;
    if (!variantToColorList(colors, &parsed)) {
        setLastError(QStringLiteral("Invalid point cloud colors"));
        return false;
    }
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    const size_t pointCount = pc->positions.size() / 3;
    if (!parsed.empty() && parsed.size() != pointCount) {
        setLastError(QStringLiteral("Point cloud color count (%1) does not match point count (%2)")
                     .arg(parsed.size()).arg(pointCount));
        return false;
    }
    pc->colors.resize(parsed.size() * 4);
    for (size_t i = 0; i < parsed.size(); ++i) {
        pc->colors[4 * i + 0] = parsed[i].x();
        pc->colors[4 * i + 1] = parsed[i].y();
        pc->colors[4 * i + 2] = parsed[i].z();
        pc->colors[4 * i + 3] = parsed[i].w();
    }
    pc->dirtyColors = true;
    setLastError(QString());
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudVisible(int cloudId, bool visible) {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->visible = visible;
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudColor(int cloudId, const QVector4D& rgba) {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->rgba = rgba;
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudPointSize(int cloudId, float pointSize) {
    if (pointSize <= 0.0f) pointSize = 1.0f;
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->pointSize = pointSize;
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudStyle(int cloudId, PointCloudStyle style) {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->style = static_cast<int>(style);
    requestRenderUpdate();
    return true;
}

bool MujocoQuickItem::setPointCloudGroundReflection(int cloudId, bool enable,
                                                    float planeZ, float intensity) {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return false;
    pc->groundReflection    = enable;
    pc->reflectionPlaneZ    = planeZ;
    pc->reflectionIntensity = std::clamp(intensity, 0.0f, 1.0f);
    requestRenderUpdate();
    return true;
}

int MujocoQuickItem::pointCloudCount() const {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    return static_cast<int>(m_pointClouds.size());
}

int MujocoQuickItem::pointCloudPointCount(int cloudId) const {
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    const PointCloudState* pc = findPointCloud(cloudId);
    return pc ? static_cast<int>(pc->positions.size() / 3) : 0;
}

QVector<QVector3D> MujocoQuickItem::pointCloudPointsRaw(int cloudId) const {
    QVector<QVector3D> result;
    std::lock_guard<std::mutex> lk(m_pointCloudMtx);
    const PointCloudState* pc = findPointCloud(cloudId);
    if (!pc) return result;
    const size_t n = pc->positions.size() / 3;
    result.reserve(static_cast<int>(n));
    for (size_t i = 0; i < n; ++i) {
        result.append(QVector3D(pc->positions[3 * i + 0],
                                pc->positions[3 * i + 1],
                                pc->positions[3 * i + 2]));
    }
    return result;
}

QVariantList MujocoQuickItem::pointCloudPoints(int cloudId) const {
    QVariantList result;
    const QVector<QVector3D> points = pointCloudPointsRaw(cloudId);
    result.reserve(points.size());
    for (const QVector3D& p : points) result.append(QVariant::fromValue(p));
    return result;
}

// 渲染线程：把点云用 GL_POINTS 画进 MuJoCo 离屏 FBO（共享深度，正确互遮挡）。
void MujocoQuickItem::onRenderOverlay(unsigned int targetFbo, int viewWidth, int viewHeight) {
    if (!m_sim || viewWidth <= 0 || viewHeight <= 0) return;

    struct DrawItem { int id; int style; float size; QVector4D color; bool visible;
                      bool reflect; float planeZ; float reflectIntensity; };
    std::vector<DrawItem> draws;
    std::vector<int> ids;
    bool anyVisible = false;
    {
        std::lock_guard<std::mutex> lk(m_pointCloudMtx);
        if (m_pointClouds.empty()) {
            if (m_pointRenderer) m_pointRenderer->retainOnly({});
            return;
        }
        if (!m_pointRenderer) m_pointRenderer.reset(new PointCloudRenderer());
        ids.reserve(m_pointClouds.size());
        draws.reserve(m_pointClouds.size());
        for (PointCloudState& pc : m_pointClouds) {
            ids.push_back(pc.id);
            if (pc.dirtyPositions) {
                const int count = static_cast<int>(pc.positions.size() / 3);
                m_pointRenderer->uploadPositions(pc.id, pc.positions.data(), count);
                pc.dirtyPositions = false;
            }
            if (pc.dirtyColors) {
                const int ccount = static_cast<int>(pc.colors.size() / 4);
                m_pointRenderer->uploadColors(
                    pc.id, pc.colors.empty() ? nullptr : pc.colors.data(), ccount);
                pc.dirtyColors = false;
            }
            draws.push_back({pc.id, pc.style, pc.pointSize, pc.rgba, pc.visible,
                             pc.groundReflection, pc.reflectionPlaneZ,
                             pc.reflectionIntensity});
            if (pc.visible && !pc.positions.empty()) anyVisible = true;
        }
        m_pointRenderer->retainOnly(ids);
    }
    if (!anyVisible) return;

    // 相机：与 mjr_render 的 setView 一致，mono 取 camera[0]/camera[1] 的平均。
    const mjvScene& scn = m_sim->scn;
    mjvGLCamera cam = mjv_averageCamera(&scn.camera[0], &scn.camera[1]);

    m_pointRenderer->beginFrame(
        targetFbo, viewWidth, viewHeight,
        cam.pos, cam.forward, cam.up,
        cam.frustum_center, cam.frustum_width, cam.frustum_bottom, cam.frustum_top,
        cam.frustum_near, cam.frustum_far, cam.orthographic != 0,
        scn.enabletransform != 0, scn.translate, scn.rotate, scn.scale);
    // 先画倒影，再画实点云盖在上面。
    // 跟随 MuJoCo 的 mjRND_REFLECTION 标志：场景 UI 关闭 "Reflections" 时
    // 同步不渲染点云倒影，与 MuJoCo 内置几何反射的开关保持一致。
    const bool mujocoReflectionOn = (scn.flags[mjRND_REFLECTION] != 0);
    if (mujocoReflectionOn) {
        for (const DrawItem& it : draws) {
            if (!it.visible || !it.reflect) continue;
            m_pointRenderer->drawCloudReflected(it.id, it.style, it.size, it.color,
                                                it.planeZ, it.reflectIntensity);
        }
    }
    for (const DrawItem& it : draws) {
        if (!it.visible) continue;
        m_pointRenderer->drawCloud(it.id, it.style, it.size, it.color);
    }
    m_pointRenderer->endFrame();
}

void MujocoQuickItem::sampleTrackedTrajectoriesLocked(const mjModel* m, const mjData* d) {
    if (!m || !d) return;
    bool anyChange = false;
    for (TrajectoryState& t : m_trajectories) {
        // 站点（site）优先于 body：若设置了 site，则解析其当前 id（每次都重新解析，
        // 以容纳重编译/重加载导致的 id 变动）；否则使用 trackedBodyId。
        QVector3D p;
        bool have = false;
        if (!t.trackedSiteName.isEmpty()) {
            t.trackedSiteId = mj_name2id(m, mjOBJ_SITE, t.trackedSiteName.toUtf8().constData());
            if (t.trackedSiteId >= 0 && t.trackedSiteId < m->nsite) {
                const mjtNum* xp = d->site_xpos + 3 * t.trackedSiteId;
                p = QVector3D(float(xp[0]), float(xp[1]), float(xp[2]));
                have = true;
            }
        } else if (t.trackedBodyId >= 0 && t.trackedBodyId < m->nbody) {
            const mjtNum* xp = d->xpos + 3 * t.trackedBodyId;
            p = QVector3D(float(xp[0]), float(xp[1]), float(xp[2]));
            have = true;
        }
        if (!have) continue;
        // 即使 minDistance==0 也过滤掉与上一个采样点完全相同的点：
        // 否则在仿真暂停 / 物体静止时，每帧都会 push 一个重复点并触发
        // rebuildTrajectoryGeomsLocked()，引起不必要的 user_scn 重建，
        // 与 Qt Quick / mujoco 渲染线程的帧节拍互相挤压导致掉到 ~30fps。
        if (!t.points.empty()) {
            const QVector3D& last = t.points.back();
            const float dist = (p - last).length();
            const float minDist = std::max(float(t.minDistance), 1e-6f);
            if (dist < minDist) continue;
        }
        t.points.push_back(p);
        while (static_cast<int>(t.points.size()) > t.maxPoints) t.points.pop_front();
        anyChange = true;
    }
    if (anyChange) rebuildTrajectoryGeomsLocked();
}

int MujocoQuickItem::addTrajectory(int maxPoints,
                                   float width,
                                   const QVector4D& rgba,
                                   bool useLine) {
    if (maxPoints < 2) maxPoints = 2;
    if (width <= 0.0f) width = 1.0f;
    if (!m_sim) {
        setLastError(QStringLiteral("Scene is not loaded"));
        return -1;
    }
    int newId = -1;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!ensureUserSceneLocked(sim)) return;
        TrajectoryState t;
        t.id        = m_nextTrajectoryId++;
        t.maxPoints = maxPoints;
        t.width     = width;
        t.rgba      = rgba;
        t.useLine   = useLine;
        m_trajectories.push_back(std::move(t));
        newId = m_trajectories.back().id;
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        setLastError(QString());
    });
    return newId;
}

bool MujocoQuickItem::removeTrajectory(int trajectoryId) {
    bool removed = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        for (auto it = m_trajectories.begin(); it != m_trajectories.end(); ++it) {
            if (it->id == trajectoryId) {
                m_trajectories.erase(it);
                removed = true;
                break;
            }
        }
        if (removed) {
            rebuildTrajectoryGeomsLocked();
            markUiRefresh(sim);
        }
    });
    return removed;
}

void MujocoQuickItem::clearTrajectories() {
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (m_trajectories.empty()) return;
        m_trajectories.clear();
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
    });
}

bool MujocoQuickItem::appendTrajectoryPoint(int trajectoryId, const QVector3D& point) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        t->points.push_back(point);
        while (static_cast<int>(t->points.size()) > t->maxPoints) t->points.pop_front();
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::clearTrajectoryPoints(int trajectoryId) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        if (!t->points.empty()) {
            t->points.clear();
            rebuildTrajectoryGeomsLocked();
            markUiRefresh(sim);
        }
        ok = true;
    });
    return ok;
}

int MujocoQuickItem::trajectoryCount() const {
    int n = 0;
    if (!m_sim) return n;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    n = static_cast<int>(m_trajectories.size());
    return n;
}

int MujocoQuickItem::trajectoryPointCount(int trajectoryId) const {
    int n = 0;
    if (!m_sim) return n;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    const TrajectoryState* t = findTrajectory(trajectoryId);
    if (t) n = static_cast<int>(t->points.size());
    return n;
}

bool MujocoQuickItem::setTrajectoryVisible(int trajectoryId, bool visible) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        if (t->visible != visible) {
            t->visible = visible;
            rebuildTrajectoryGeomsLocked();
            markUiRefresh(sim);
        }
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::setTrajectoryColor(int trajectoryId, const QVector4D& rgba) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        t->rgba = rgba;
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::setTrajectoryWidth(int trajectoryId, float width) {
    if (width <= 0.0f) width = 1.0f;
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        t->width = width;
        rebuildTrajectoryGeomsLocked();
        markUiRefresh(sim);
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::setTrajectoryMaxPoints(int trajectoryId, int maxPoints) {
    if (maxPoints < 2) maxPoints = 2;
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        t->maxPoints = maxPoints;
        bool changed = false;
        while (static_cast<int>(t->points.size()) > t->maxPoints) {
            t->points.pop_front();
            changed = true;
        }
        if (changed) {
            rebuildTrajectoryGeomsLocked();
            markUiRefresh(sim);
        }
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::setTrajectoryTrackedBody(int trajectoryId,
                                               int bodyId,
                                               double minDistance) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        // 设置 body 跟踪时清空 site 跟踪（互斥）。
        t->trackedSiteName.clear();
        t->trackedSiteId  = -1;
        t->trackedBodyId  = bodyId;
        t->minDistance    = minDistance;
        ok = true;
    });
    return ok;
}

bool MujocoQuickItem::setTrajectoryTrackedSite(int trajectoryId,
                                               const QString& siteName,
                                               double minDistance) {
    bool ok = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        TrajectoryState* t = findTrajectory(trajectoryId);
        if (!t) return;
        t->trackedSiteName = siteName.trimmed();
        // 立即尝试解析一次 id（若失败留 -1，由 sample 时再重试）。
        if (!t->trackedSiteName.isEmpty() && sim.m_) {
            t->trackedSiteId = mj_name2id(sim.m_, mjOBJ_SITE,
                                          t->trackedSiteName.toUtf8().constData());
        } else {
            t->trackedSiteId = -1;
        }
        t->trackedBodyId = -1;
        t->minDistance   = minDistance;
        ok = true;
    });
    return ok;
}

QStringList MujocoQuickItem::siteNames() const {
    QStringList names;
    if (!m_sim) return names;
    std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
    const mjModel* m = m_sim->m_;
    if (!m) return names;
    names.reserve(m->nsite);
    for (int i = 0; i < m->nsite; ++i) {
        const char* n = mj_id2name(m, mjOBJ_SITE, i);
        names.append(n ? QString::fromUtf8(n) : QString());
    }
    return names;
}

int MujocoQuickItem::addStaticObstacle(PrimitiveType type,
                                       const QVector3D& position,
                                       const QVector3D& size,
                                       const QVector4D& rgba,
                                       int contype,
                                       int conaffinity,
                                       const QString& name)
{
    QVariantList types;
    QVariantList positions;
    QVariantList sizes;
    QVariantList colors;
    types.append(static_cast<int>(type));
    positions.append(QVariant::fromValue(position));
    sizes.append(QVariant::fromValue(size));
    colors.append(QVariant::fromValue(rgba));

    const QVariantList bodyIds = addStaticObstacleRequests(types, positions, sizes, colors,
                                                           contype, conaffinity, name, true);
    return bodyIds.isEmpty() ? -1 : bodyIds.first().toInt();
}

QVariantList MujocoQuickItem::addStaticObstacles(const QVariantList& types,
                                                 const QVariantList& positions,
                                                 const QVariantList& sizes,
                                                 const QVariantList& rgba,
                                                 int contype,
                                                 int conaffinity,
                                                 const QString& namePrefix)
{
    return addStaticObstacleRequests(types, positions, sizes, rgba,
                                     contype, conaffinity, namePrefix, false);
}

QVariantList MujocoQuickItem::addStaticObstacleRequests(const QVariantList& types,
                                                        const QVariantList& positions,
                                                        const QVariantList& sizes,
                                                        const QVariantList& rgba,
                                                        int contype,
                                                        int conaffinity,
                                                        const QString& namePrefix,
                                                        bool useExactSingleName)
{
    std::vector<PrimitiveRequest> requests;
    QString error;
    const QString effectivePrefix = namePrefix.trimmed().isEmpty() ? QStringLiteral("obstacle") : namePrefix.trimmed();
    if (!buildPrimitiveRequests(types, positions, sizes, rgba,
                                QVector4D(0.9f, 0.25f, 0.15f, 0.8f),
                                effectivePrefix, &requests, &error)) {
        setLastError(error);
        return {};
    }
    if (useExactSingleName && requests.size() == 1) {
        requests[0].name = namePrefix.trimmed();
    }
    if (requests.empty()) {
        setLastError(QString());
        return {};
    }

    if (!m_sim) {
        setLastError(QStringLiteral("Scene is not loaded"));
        return {};
    }

    mjModel* modelForReload = nullptr;
    mjData* dataForReload = nullptr;
    QByteArray displayedFilename;
    std::vector<QString> addedBodyNames;
    QVariantList bodyIds;

    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        if (!m_sim->m_ || !m_sim->d_) {
            setLastError(QStringLiteral("Scene is not loaded"));
            return {};
        }
        if (!m_editSpec) {
            setLastError(QStringLiteral("Current scene is not editable; load an XML scene before adding obstacles"));
            return {};
        }

        mjSpec* spec = mj_copySpec(m_editSpec);
        if (!spec) {
            setLastError(QStringLiteral("Failed to copy MuJoCo spec"));
            return {};
        }

        if (!mj_copyBack(spec, m_sim->m_)) {
            mj_deleteSpec(spec);
            setLastError(QStringLiteral("Failed to copy current model values back to MuJoCo spec"));
            return {};
        }

        mjsBody* world = mjs_findBody(spec, "world");
        if (!world) {
            mj_deleteSpec(spec);
            setLastError(QStringLiteral("Failed to find MuJoCo world body in spec"));
            return {};
        }

        addedBodyNames.reserve(requests.size());
        for (size_t i = 0; i < requests.size(); ++i) {
            const PrimitiveRequest& request = requests[i];
            const QString baseName = request.name.trimmed().isEmpty()
                ? QStringLiteral("obstacle_%1").arg(m_sim->m_->nbody + static_cast<int>(i))
                : request.name;
            const QString bodyName = uniqueObjectName(m_sim->m_, addedBodyNames, mjOBJ_BODY, baseName);
            const QString geomName = uniqueObjectName(m_sim->m_, {}, mjOBJ_GEOM, bodyName + QStringLiteral("_geom"));

            mjsBody* body = mjs_addBody(world, nullptr);
            mjsGeom* geom = body ? mjs_addGeom(body, nullptr) : nullptr;
            if (!body || !geom) {
                mj_deleteSpec(spec);
                setLastError(QStringLiteral("Failed to add obstacle body or geom"));
                return {};
            }

            const QByteArray bodyNameUtf8 = bodyName.toUtf8();
            const QByteArray geomNameUtf8 = geomName.toUtf8();
            mjs_setName(body->element, bodyNameUtf8.constData());
            mjs_setName(geom->element, geomNameUtf8.constData());
            copyVec3(body->pos, request.position);
            geom->type = static_cast<mjtGeom>(request.geomType);
            setPrimitiveSize(geom, request.geomType, request.size);
            geom->mass = 0.0;
            geom->contype = contype;
            geom->conaffinity = conaffinity;
            geom->rgba[0] = request.rgba.x();
            geom->rgba[1] = request.rgba.y();
            geom->rgba[2] = request.rgba.z();
            geom->rgba[3] = request.rgba.w();
            addedBodyNames.push_back(bodyName);
        }

        const int rc = mj_recompile(spec, nullptr, m_sim->m_, m_sim->d_);
        if (rc != 0) {
            const char* err = mjs_getError(spec);
            const QString reason = err && err[0]
                ? QString::fromUtf8(err)
                : QStringLiteral("Failed to recompile MuJoCo model after adding obstacles");
            mj_deleteSpec(spec);
            setLastError(reason);
            return {};
        }

        mj_deleteSpec(m_editSpec);
        m_editSpec = spec;
        for (const QString& bodyName : addedBodyNames) {
            bodyIds.append(mj_name2id(m_sim->m_, mjOBJ_BODY, bodyName.toUtf8().constData()));
        }
        mj_forward(m_sim->m_, m_sim->d_);
        markUiRefresh(*m_sim);
        modelForReload = m_sim->m_;
        dataForReload = m_sim->d_;
        displayedFilename = QByteArray(m_sim->filename);
    }

    if (modelForReload && dataForReload) {
        m_sim->Load(modelForReload, dataForReload, displayedFilename.constData());
    }
    setLastError(QString());
    requestRenderUpdate();
    return bodyIds;
}

// ---------------------------------------------------------------------------
// 关节查询与控制
// ---------------------------------------------------------------------------

static constexpr int kJntQposDim[] = {7, 4, 1, 1}; // free, ball, slide, hinge
static const char* const kJntTypeName[] = {"free", "ball", "slide", "hinge"};

static SceneObjectInfo buildSceneObjectInfo(const mjModel* model, const mjData* data, int bodyId)
{
    SceneObjectInfo info;
    if (!model || !data || bodyId < 0 || bodyId >= model->nbody) return info;

    info.bodyId = bodyId;
    info.name = objectName(model, mjOBJ_BODY, bodyId);
    info.parentBodyId = model->body_parentid[bodyId];
    info.parentName = objectName(model, mjOBJ_BODY, info.parentBodyId);
    info.position = vectorFrom3(data->xpos + 3 * bodyId);
    info.localPosition = vectorFrom3(model->body_pos + 3 * bodyId);
    info.orientation = quaternionFrom4(data->xquat + 4 * bodyId);
    info.localOrientation = quaternionFrom4(model->body_quat + 4 * bodyId);
    info.mass = static_cast<double>(model->body_mass[bodyId]);
    info.jointCount = model->body_jntnum[bodyId];
    info.geomCount = model->body_geomnum[bodyId];
    info.movable = model->body_dofnum[bodyId] > 0;

    if (info.geomCount > 0) {
        const int geomId = model->body_geomadr[bodyId];
        info.firstGeomId = geomId;
        info.firstGeomName = objectName(model, mjOBJ_GEOM, geomId);
        info.firstGeomType = model->geom_type[geomId];
        info.firstGeomTypeName = geomTypeName(info.firstGeomType);
        info.firstGeomSize = vectorFrom3(model->geom_size + 3 * geomId);
        const float* rgba = model->geom_rgba + 4 * geomId;
        info.firstGeomRgba = QVector4D(rgba[0], rgba[1], rgba[2], rgba[3]);
        info.firstGeomContype    = model->geom_contype[geomId];
        info.firstGeomConaffinity = model->geom_conaffinity[geomId];
    }
    return info;
}

static bool setBodyGeomSize(mjModel* model, int bodyId, const QVector3D& size)
{
    if (!model || bodyId < 0 || bodyId >= model->nbody) return false;

    const int geomCount = model->body_geomnum[bodyId];
    const int firstGeom = model->body_geomadr[bodyId];
    if (geomCount <= 0 || firstGeom < 0) return false;

    const mjtNum sx = positiveOr(size.x(), 0.001);
    const mjtNum sy = positiveOr(size.y(), sx);
    const mjtNum sz = positiveOr(size.z(), sy);
    for (int i = 0; i < geomCount; ++i) {
        mjtNum* geomSize = model->geom_size + 3 * (firstGeom + i);
        geomSize[0] = sx;
        geomSize[1] = sy;
        geomSize[2] = sz;
    }
    return true;
}

static bool scaleBodyGeoms(mjModel* model, int bodyId, const QVector3D& scale)
{
    if (!model || bodyId < 0 || bodyId >= model->nbody) return false;
    const int geomCount = model->body_geomnum[bodyId];
    const int firstGeom = model->body_geomadr[bodyId];
    if (geomCount <= 0 || firstGeom < 0) return false;

    const mjtNum sx = positiveOr(scale.x(), 1.0);
    const mjtNum sy = positiveOr(scale.y(), sx);
    const mjtNum sz = positiveOr(scale.z(), sy);
    for (int i = 0; i < geomCount; ++i) {
        mjtNum* geomSize = model->geom_size + 3 * (firstGeom + i);
        geomSize[0] *= sx;
        geomSize[1] *= sy;
        geomSize[2] *= sz;
    }
    return true;
}

int MujocoQuickItem::objectCount() const
{
    int count = 0;
    withSimulation([&](const mjModel* model, mjData*) {
        count = model->nbody;
    });
    return count;
}

SceneObjectInfo MujocoQuickItem::objectInfo(int index) const
{
    SceneObjectInfo result;
    withSimulation([&](const mjModel* model, mjData* data) {
        if (index < 0 || index >= model->nbody) return;
        result = buildSceneObjectInfo(model, data, index);
    });
    return result;
}

QVariantList MujocoQuickItem::objects() const
{
    QVariantList result;
    withSimulation([&](const mjModel* model, mjData* data) {
        result.reserve(model->nbody);
        for (int bodyId = 0; bodyId < model->nbody; ++bodyId) {
            result.append(QVariant::fromValue(buildSceneObjectInfo(model, data, bodyId)));
        }
    });
    return result;
}

bool MujocoQuickItem::setObjectPosition(int bodyId, const QVector3D& position)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_ || bodyId < 0 || bodyId >= sim.m_->nbody) return;

        const int freeJointId = freeJointIndexForBody(sim.m_, bodyId);
        if (freeJointId >= 0) {
            setFreeJointPosition(sim.m_, sim.d_,
                                 sim.qpos_, sim.qpos_prev_,
                                 freeJointId, position);
        } else {
            setBodyLocalPositionFromWorld(sim.m_, sim.d_, bodyId, position);
            mj_setConst(sim.m_, sim.d_);
        }
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectOrientation(int bodyId, const QQuaternion& orientation)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_ || bodyId < 0 || bodyId >= sim.m_->nbody) return;

        const int freeJointId = freeJointIndexForBody(sim.m_, bodyId);
        if (freeJointId >= 0) {
            setFreeJointOrientation(sim.m_, sim.d_,
                                    sim.qpos_, sim.qpos_prev_,
                                    freeJointId, orientation);
        } else {
            setBodyLocalOrientationFromWorld(sim.m_, sim.d_, bodyId, orientation);
            mj_setConst(sim.m_, sim.d_);
        }
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectPose(int bodyId,
                                    const QVector3D& position,
                                    const QQuaternion& orientation)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_ || bodyId < 0 || bodyId >= sim.m_->nbody) return;

        const int freeJointId = freeJointIndexForBody(sim.m_, bodyId);
        if (freeJointId >= 0) {
            setFreeJointPosition(sim.m_, sim.d_,
                                 sim.qpos_, sim.qpos_prev_,
                                 freeJointId, position);
            setFreeJointOrientation(sim.m_, sim.d_,
                                    sim.qpos_, sim.qpos_prev_,
                                    freeJointId, orientation);
        } else {
            setBodyLocalPositionFromWorld(sim.m_, sim.d_, bodyId, position);
            setBodyLocalOrientationFromWorld(sim.m_, sim.d_, bodyId, orientation);
            mj_setConst(sim.m_, sim.d_);
        }
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectLocalPosition(int bodyId, const QVector3D& localPosition)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_ || bodyId < 0 || bodyId >= sim.m_->nbody) return;
        const int freeJointId = freeJointIndexForBody(sim.m_, bodyId);
        if (freeJointId >= 0) {
            // free joint body：仍走 qpos（free joint 的 pos 即局部于世界）
            setFreeJointPosition(sim.m_, sim.d_,
                                 sim.qpos_, sim.qpos_prev_,
                                 freeJointId, localPosition);
        } else {
            // 静态 body：直接写 model->body_pos，等同于 XML 的 pos 属性
            mjtNum* bodyPos = sim.m_->body_pos + 3 * bodyId;
            bodyPos[0] = static_cast<mjtNum>(localPosition.x());
            bodyPos[1] = static_cast<mjtNum>(localPosition.y());
            bodyPos[2] = static_cast<mjtNum>(localPosition.z());
            mj_setConst(sim.m_, sim.d_);
        }
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectLocalPose(int bodyId,
                                         const QVector3D& localPosition,
                                         const QQuaternion& localOrientation)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_ || bodyId < 0 || bodyId >= sim.m_->nbody) return;
        const int freeJointId = freeJointIndexForBody(sim.m_, bodyId);
        if (freeJointId >= 0) {
            setFreeJointPosition(sim.m_, sim.d_,
                                 sim.qpos_, sim.qpos_prev_,
                                 freeJointId, localPosition);
            setFreeJointOrientation(sim.m_, sim.d_,
                                    sim.qpos_, sim.qpos_prev_,
                                    freeJointId, localOrientation);
        } else {
            // 直接写 model->body_pos / body_quat（XML pos/quat 属性语义）
            mjtNum* bodyPos = sim.m_->body_pos + 3 * bodyId;
            bodyPos[0] = static_cast<mjtNum>(localPosition.x());
            bodyPos[1] = static_cast<mjtNum>(localPosition.y());
            bodyPos[2] = static_cast<mjtNum>(localPosition.z());
            mjtNum localQuat[4];
            quaternionToMj(localOrientation, localQuat);
            mju_normalize4(localQuat);
            mjtNum* bodyQuat = sim.m_->body_quat + 4 * bodyId;
            for (int i = 0; i < 4; ++i) bodyQuat[i] = localQuat[i];
            mj_setConst(sim.m_, sim.d_);
        }
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectSize(int bodyId, const QVector3D& size)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!setBodyGeomSize(sim.m_, bodyId, size)) return;
        mj_setConst(sim.m_, sim.d_);
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::scaleObject(int bodyId, const QVector3D& scale)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!scaleBodyGeoms(sim.m_, bodyId, scale)) return;
        mj_setConst(sim.m_, sim.d_);
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectColor(int bodyId, const QVector4D& rgba)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.m_;
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;
        for (int i = 0; i < geomCount; ++i) {
            float* dst = model->geom_rgba + 4 * (firstGeom + i);
            dst[0] = rgba.x();
            dst[1] = rgba.y();
            dst[2] = rgba.z();
            dst[3] = rgba.w();
        }
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

QVector<QVector4D> MujocoQuickItem::objectGeomColors(int bodyId) const
{
    QVector<QVector4D> result;
    withSimulation([&](const mjModel* model, mjData*) {
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;
        result.reserve(geomCount);
        for (int i = 0; i < geomCount; ++i) {
            const float* src = model->geom_rgba + 4 * (firstGeom + i);
            result.append(QVector4D(src[0], src[1], src[2], src[3]));
        }
    });
    return result;
}

bool MujocoQuickItem::setObjectGeomColors(int bodyId, const QVector<QVector4D>& colors)
{
    if (colors.isEmpty()) return false;
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.m_;
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;
        for (int i = 0; i < geomCount; ++i) {
            const QVector4D& c = colors[qMin(i, colors.size() - 1)];
            float* dst = model->geom_rgba + 4 * (firstGeom + i);
            dst[0] = c.x(); dst[1] = c.y(); dst[2] = c.z(); dst[3] = c.w();
        }
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectContype(int bodyId, int contype)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.m_;
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;
        for (int i = 0; i < geomCount; ++i)
            model->geom_contype[firstGeom + i] = contype;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setObjectConaffinity(int bodyId, int conaffinity)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.m_;
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;
        for (int i = 0; i < geomCount; ++i)
            model->geom_conaffinity[firstGeom + i] = conaffinity;
        applied = true;
    });
    return applied;
}

int MujocoQuickItem::objectContype(int bodyId) const
{
    int result = 0;
    withSimulation([&](const mjModel* model, mjData*) {
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int firstGeom = model->body_geomadr[bodyId];
        if (model->body_geomnum[bodyId] > 0 && firstGeom >= 0)
            result = model->geom_contype[firstGeom];
    });
    return result;
}

int MujocoQuickItem::objectConaffinity(int bodyId) const
{
    int result = 0;
    withSimulation([&](const mjModel* model, mjData*) {
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int firstGeom = model->body_geomadr[bodyId];
        if (model->body_geomnum[bodyId] > 0 && firstGeom >= 0)
            result = model->geom_conaffinity[firstGeom];
    });
    return result;
}

BodyMeshData MujocoQuickItem::bodyCollisionMesh(int bodyId) const
{
    BodyMeshData result;
    withSimulation([&](const mjModel* model, mjData*) {
        if (!model || bodyId < 0 || bodyId >= model->nbody) return;
        const int geomCount = model->body_geomnum[bodyId];
        const int firstGeom = model->body_geomadr[bodyId];
        if (geomCount <= 0 || firstGeom < 0) return;

        for (int g = 0; g < geomCount; ++g) {
            const int geomId = firstGeom + g;
            if (model->geom_type[geomId] != mjGEOM_MESH) continue;
            const int meshId = model->geom_dataid[geomId];
            if (meshId < 0 || meshId >= model->nmesh) continue;

            const int vertAdr = model->mesh_vertadr[meshId];
            const int vertNum = model->mesh_vertnum[meshId];
            const int faceAdr = model->mesh_faceadr[meshId];
            const int faceNum = model->mesh_facenum[meshId];
            if (vertNum <= 0 || faceNum <= 0) continue;

            // geom 局部位姿（相对 body），把 mesh 顶点变换到 body 坐标系。
            const QVector3D geomPos = vectorFrom3(model->geom_pos + 3 * geomId);
            QQuaternion geomQuat = quaternionFrom4(model->geom_quat + 4 * geomId);
            geomQuat.normalize();

            const int base = result.vertices.size();
            result.vertices.reserve(base + vertNum);
            for (int v = 0; v < vertNum; ++v) {
                const float* mv = model->mesh_vert + 3 * (vertAdr + v);
                const QVector3D local(mv[0], mv[1], mv[2]);
                result.vertices.append(geomPos + geomQuat.rotatedVector(local));
            }
            result.indices.reserve(result.indices.size() + faceNum * 3);
            for (int f = 0; f < faceNum; ++f) {
                const int* face = model->mesh_face + 3 * (faceAdr + f);
                result.indices.append(base + face[0]);
                result.indices.append(base + face[1]);
                result.indices.append(base + face[2]);
            }
            result.valid = true;
        }
    });
    return result;
}

// ---------------------------------------------------------------------------
// 外部窄阶段碰撞接入
// ---------------------------------------------------------------------------
std::vector<MujocoQuickItem*> MujocoQuickItem::s_narrowPhaseHosts;
std::mutex                    MujocoQuickItem::s_narrowPhaseMutex;
void*                         MujocoQuickItem::s_origMeshCollisionFn = nullptr;

MujocoQuickItem* MujocoQuickItem::hostForModel(const mjModel* m)
{
    if (!m) return nullptr;
    std::lock_guard<std::mutex> lk(s_narrowPhaseMutex);
    for (MujocoQuickItem* h : s_narrowPhaseHosts) {
        // 每个实例拥有独立的 mjModel/mjData；当前正在碰撞的 m 唯一对应其中一个
        // 实例（即正在 step/forward 的那一个，必然存活）。
        if (h && h->m_sim && h->m_sim->m_ == m)
            return h;
    }
    return nullptr;
}

namespace {
// 返回 body 的第一个 mesh geom 的 geom id；无则 -1。用于多 geom body 去重：
// 一个 body 对只在其代表 mesh geom 对上计算一次外部碰撞。
int firstMeshGeomOfBody(const mjModel* m, int body)
{
    if (!m || body < 0 || body >= m->nbody) return -1;
    const int n   = m->body_geomnum[body];
    const int adr = m->body_geomadr[body];
    for (int i = 0; i < n; ++i) {
        const int g = adr + i;
        if (g >= 0 && g < m->ngeom && m->geom_type[g] == mjGEOM_MESH)
            return g;
    }
    return -1;
}
} // namespace

int MujocoQuickItem::externalMeshCollisionThunk(const mjModel* m, mjData* d,
                                                mjContact* con, int g1, int g2,
                                                double margin)
{
    // 全局表是进程级的，被 patch 后所有实例的 mesh×mesh 都会走到这里；
    // 按传入的 mjModel* 反查安装了外部窄阶段的对应实例，未安装或不匹配
    // 的实例回退到 MuJoCo 原始 mesh 碰撞。
    MujocoQuickItem* host = hostForModel(m);
    auto fallback = [&]() -> int {
        if (s_origMeshCollisionFn) {
            auto fn = reinterpret_cast<mjfCollision>(s_origMeshCollisionFn);
            return fn(m, d, con, g1, g2, margin);
        }
        return 0;
    };

    if (!host || !host->m_extPairFilter || !host->m_extNarrowPhase || !m || !d)
        return fallback();

    const int b1 = m->geom_bodyid[g1];
    const int b2 = m->geom_bodyid[g2];

    // 不归外部库接管的 body 对 → 回退到 MuJoCo 原始 mesh 碰撞。
    if (!host->m_extPairFilter(b1, b2))
        return fallback();

    // 去重：被接管的 body 对只在代表 mesh geom 对上计算一次，其余几何对返回
    // 0 个接触（已被代表对覆盖），不再回退到默认碰撞。
    if (g1 != firstMeshGeomOfBody(m, b1) || g2 != firstMeshGeomOfBody(m, b2))
        return 0;

    ExternalContactPoint buf[mjMAXCONPAIR];
    const int produced = host->m_extNarrowPhase(
        b1, d->xpos + 3 * b1, d->xmat + 9 * b1,
        b2, d->xpos + 3 * b2, d->xmat + 9 * b2,
        buf, mjMAXCONPAIR);

    int written = 0;
    for (int i = 0; i < produced && written < mjMAXCONPAIR; ++i) {
        // 只注入穿透 / 在 margin 内的接触；分离接触交给求解器无意义。
        if (buf[i].dist >= margin) continue;

        mjContact& c = con[written];
        // 近相位碰撞函数只需填 dist / pos / frame；friction / solref / solimp /
        // dim / geom 等由 MuJoCo 的 mj_collideGeoms 在返回后补全。
        c.dist   = buf[i].dist;
        c.pos[0] = buf[i].pos[0];
        c.pos[1] = buf[i].pos[1];
        c.pos[2] = buf[i].pos[2];

        // 构造正交接触帧：row0 = 法向（geom[0]→geom[1]），row1/row2 = 切向。
        mjtNum nrm[3] = { buf[i].normal[0], buf[i].normal[1], buf[i].normal[2] };
        if (mju_normalize3(nrm) < mjMINVAL) continue; // 退化法向，丢弃

        mjtNum ref[3] = { 1, 0, 0 };
        if (std::fabs(nrm[0]) > 0.9) { ref[0] = 0; ref[1] = 1; ref[2] = 0; }
        const mjtNum dot = nrm[0]*ref[0] + nrm[1]*ref[1] + nrm[2]*ref[2];
        mjtNum t1[3] = { ref[0] - dot*nrm[0], ref[1] - dot*nrm[1], ref[2] - dot*nrm[2] };
        mju_normalize3(t1);
        mjtNum t2[3];
        mju_cross(t2, nrm, t1);

        c.frame[0] = nrm[0]; c.frame[1] = nrm[1]; c.frame[2] = nrm[2];
        c.frame[3] = t1[0];  c.frame[4] = t1[1];  c.frame[5] = t1[2];
        c.frame[6] = t2[0];  c.frame[7] = t2[1];  c.frame[8] = t2[2];
        ++written;
    }
    return written;
}

void MujocoQuickItem::setExternalNarrowPhase(ExternalPairFilter filter,
                                             ExternalNarrowPhaseFn provider)
{
    auto apply = [&]() {
        if (filter && provider) {
            m_extPairFilter  = std::move(filter);
            m_extNarrowPhase = std::move(provider);
            std::lock_guard<std::mutex> lk(s_narrowPhaseMutex);
            // 注册本实例（去重）。
            if (std::find(s_narrowPhaseHosts.begin(), s_narrowPhaseHosts.end(), this)
                    == s_narrowPhaseHosts.end()) {
                s_narrowPhaseHosts.push_back(this);
            }
            // 第一个安装者负责 patch 全局表（只 patch 一次）。
            if (!s_origMeshCollisionFn) {
                s_origMeshCollisionFn =
                    reinterpret_cast<void*>(mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_MESH]);
                mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_MESH] =
                    reinterpret_cast<mjfCollision>(&MujocoQuickItem::externalMeshCollisionThunk);
            }
        } else {
            // 卸载：从注册表移除本实例；最后一个卸载者恢复原始 mesh 碰撞函数。
            m_extPairFilter  = nullptr;
            m_extNarrowPhase = nullptr;
            std::lock_guard<std::mutex> lk(s_narrowPhaseMutex);
            s_narrowPhaseHosts.erase(
                std::remove(s_narrowPhaseHosts.begin(), s_narrowPhaseHosts.end(), this),
                s_narrowPhaseHosts.end());
            if (s_narrowPhaseHosts.empty() && s_origMeshCollisionFn) {
                mjCOLLISIONFUNC[mjGEOM_MESH][mjGEOM_MESH] =
                    reinterpret_cast<mjfCollision>(s_origMeshCollisionFn);
                s_origMeshCollisionFn = nullptr;
            }
        }
    };

    // 安装/卸载必须与本实例物理线程串行：在 sim.mtx 锁内换入/换出。
    if (m_sim) {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        apply();
    } else {
        apply();
    }
}

bool MujocoQuickItem::externalNarrowPhaseInstalled() const
{
    std::lock_guard<std::mutex> lk(s_narrowPhaseMutex);
    return std::find(s_narrowPhaseHosts.begin(), s_narrowPhaseHosts.end(), this)
               != s_narrowPhaseHosts.end()
        && static_cast<bool>(m_extPairFilter)
        && static_cast<bool>(m_extNarrowPhase);
}


static QList<ContactInfo> buildContactSnapshot(mjModel* m, mjData* d)
{
    QList<ContactInfo> result;
    if (!m || !d || d->ncon <= 0) return result;
    result.reserve(d->ncon);

    auto nameOf = [m](int objType, int id) -> QString {
        if (id < 0) return QString();
        const char* n = mj_id2name(m, objType, id);
        return (n && n[0] != '\0') ? QString::fromUtf8(n)
                                   : QStringLiteral("#%1").arg(id);
    };

    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& c = d->contact[i];
        ContactInfo info;
        info.geom0Id = c.geom[0];
        info.geom1Id = c.geom[1];
        info.body0Id = (c.geom[0] >= 0) ? m->geom_bodyid[c.geom[0]] : -1;
        info.body1Id = (c.geom[1] >= 0) ? m->geom_bodyid[c.geom[1]] : -1;
        info.geom0Name = nameOf(mjOBJ_GEOM, c.geom[0]);
        info.geom1Name = nameOf(mjOBJ_GEOM, c.geom[1]);
        info.body0Name = nameOf(mjOBJ_BODY, info.body0Id);
        info.body1Name = nameOf(mjOBJ_BODY, info.body1Id);
        info.dist        = static_cast<double>(c.dist);
        info.active      = (c.exclude == 0) && (c.efc_address >= 0);
        info.penetrating = (c.dist < 0);
        // 法向接触力（mj_contactForce 返回 6D 质心力，force[0] 为法向分量）
        mjtNum force[6] = {};
        mj_contactForce(m, d, i, force);
        info.normalForce = static_cast<double>(force[0]);
        info.position = QVector3D(static_cast<float>(c.pos[0]),
                                  static_cast<float>(c.pos[1]),
                                  static_cast<float>(c.pos[2]));
        // 接触帧首行为接触法向（row-major，3x3）
        info.normal = QVector3D(static_cast<float>(c.frame[0]),
                                static_cast<float>(c.frame[1]),
                                static_cast<float>(c.frame[2]));
        result.append(std::move(info));
    }
    return result;
}

static bool sameContactInfo(const ContactInfo& lhs, const ContactInfo& rhs)
{
    return lhs.geom0Id == rhs.geom0Id
        && lhs.geom1Id == rhs.geom1Id
        && lhs.body0Id == rhs.body0Id
        && lhs.body1Id == rhs.body1Id
        && lhs.geom0Name == rhs.geom0Name
        && lhs.geom1Name == rhs.geom1Name
        && lhs.body0Name == rhs.body0Name
        && lhs.body1Name == rhs.body1Name
        && lhs.dist == rhs.dist
        && lhs.active == rhs.active
        && lhs.penetrating == rhs.penetrating
        && lhs.normalForce == rhs.normalForce
        && lhs.position == rhs.position
        && lhs.normal == rhs.normal;
}

static bool sameContactSnapshot(const QList<ContactInfo>& lhs, const QList<ContactInfo>& rhs)
{
    if (lhs.size() != rhs.size()) return false;
    for (int i = 0; i < lhs.size(); ++i) {
        if (!sameContactInfo(lhs.at(i), rhs.at(i))) return false;
    }
    return true;
}

int MujocoQuickItem::jointCount() const
{
    int count = 0;
    withSimulation([&](const mjModel* m, mjData*) {
        count = static_cast<int>(m->njnt);
    });
    return count;
}

JointInfo MujocoQuickItem::jointInfo(int index) const
{
    JointInfo result;
    withSimulation([&](const mjModel* m, mjData*) {
        if (!isValidIndex(index, static_cast<int>(m->njnt))) return;
        int type = m->jnt_type[index];
        const char* rawName = mj_id2name(m, mjOBJ_JOINT, index);
        result.name      = rawName ? QString::fromUtf8(rawName)
                                   : QStringLiteral("joint_%1").arg(index);
        result.type      = type;
        result.typeName  = QString::fromLatin1(kJntTypeName[type]);
        result.qposDim   = kJntQposDim[type];
        result.limited   = (m->jnt_limited[index] != 0);
        result.rangeMin  = result.limited ? m->jnt_range[2 * index]     : 0.0;
        result.rangeMax  = result.limited ? m->jnt_range[2 * index + 1] : 0.0;
        result.stiffness = m->jnt_stiffness[index];
        result.qposadr   = m->jnt_qposadr[index];
    });
    return result;
}

QVariantList MujocoQuickItem::jointPosition(int index) const
{
    QVariantList result;
    withSimulation([&](const mjModel* m, mjData* d) {
        if (!isValidIndex(index, static_cast<int>(m->njnt))) return;
        int dim = kJntQposDim[m->jnt_type[index]];
        int adr = m->jnt_qposadr[index];
        for (int k = 0; k < dim; ++k)
            result.append(d->qpos[adr + k]);
    });
    return result;
}

bool MujocoQuickItem::setJointPosition(int index, const QVariantList& values)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int dim = kJntQposDim[sim.m_->jnt_type[index]];
        if (values.size() != dim) return;
        int adr = sim.m_->jnt_qposadr[index];
        for (int k = 0; k < dim; ++k)
            sim.qpos_[adr + k] = values[k].toDouble();
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointValue(int index, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int type = sim.m_->jnt_type[index];
        if (type != mjJNT_SLIDE && type != mjJNT_HINGE) return;
        sim.qpos_[sim.m_->jnt_qposadr[index]] = value;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointValueByName(const QString& name, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int id = mj_name2id(sim.m_, mjOBJ_JOINT, name.toUtf8().constData());
        if (!isValidIndex(id, static_cast<int>(sim.m_->njnt))) return;
        int type = sim.m_->jnt_type[id];
        if (type != mjJNT_SLIDE && type != mjJNT_HINGE) return;
        sim.qpos_[sim.m_->jnt_qposadr[id]] = value;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointValueByNameSync(const QString& name, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int id = mj_name2id(sim.m_, mjOBJ_JOINT, name.toUtf8().constData());
        if (!isValidIndex(id, static_cast<int>(sim.m_->njnt))) return;
        const int type = sim.m_->jnt_type[id];
        if (type != mjJNT_SLIDE && type != mjJNT_HINGE) return;
        setHingeJointValue(sim.m_, sim.d_, sim.qpos_, sim.qpos_prev_, id, value);
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointAnchorPos(int index, const QVector3D& pos)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int type = sim.m_->jnt_type[index];
        if (type == mjJNT_FREE) return;  // free joint pos 无意义
        mjtNum* jpos = sim.m_->jnt_pos + 3 * index;
        jpos[0] = static_cast<mjtNum>(pos.x());
        jpos[1] = static_cast<mjtNum>(pos.y());
        jpos[2] = static_cast<mjtNum>(pos.z());
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointAxis(int index, const QVector3D& axis)
{
    QVector3D n = axis.normalized();
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->njnt))) return;
        int type = sim.m_->jnt_type[index];
        if (type != mjJNT_SLIDE && type != mjJNT_HINGE) return;  // free/ball 没有 axis
        mjtNum* jaxis = sim.m_->jnt_axis + 3 * index;
        jaxis[0] = static_cast<mjtNum>(n.x());
        jaxis[1] = static_cast<mjtNum>(n.y());
        jaxis[2] = static_cast<mjtNum>(n.z());
        mj_forward(sim.m_, sim.d_);
        markUiRefresh(sim);
        applied = true;
    });
    return applied;
}

QVariantList MujocoQuickItem::joints() const
{
    QVariantList result;
    withSimulation([&](const mjModel* m, mjData* d) {
        result.reserve(static_cast<int>(m->nq));
        for (int i = 0; i < static_cast<int>(m->nq); ++i)
            result.append(d->qpos[i]);
    });
    return result;
}

bool MujocoQuickItem::setJoints(const QVariantList& values)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int nq = static_cast<int>(sim.m_->nq);
        // 允许只传入部分关节值：只写入前 count 个 qpos，其余保持当前值不变。
        const int count = qMin(static_cast<int>(values.size()), nq);
        if (count <= 0) return;
        for (int i = 0; i < count; ++i)
            sim.qpos_[i] = values[i].toDouble();
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    return applied;
}

QVariantList MujocoQuickItem::setJointsAndDetect(const QVariantMap& values)
{
    QVariantList result;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (values.isEmpty()) return;
        bool anyWritten = false;
        for (auto it = values.cbegin(); it != values.cend(); ++it) {
            const int id = mj_name2id(sim.m_, mjOBJ_JOINT,
                                      it.key().toUtf8().constData());
            if (!isValidIndex(id, static_cast<int>(sim.m_->njnt))) continue;
            const int type = sim.m_->jnt_type[id];
            const int adr  = sim.m_->jnt_qposadr[id];
            const int dim  = kJntQposDim[type];
            if (dim == 1) {
                // hinge / slide：接受单个数值
                bool ok = false;
                const double v = it.value().toDouble(&ok);
                if (!ok) continue;
                sim.qpos_[adr] = v;
                sim.d_->qpos[adr] = v;
                anyWritten = true;
            } else {
                // ball (dim=4) / free (dim=7)：接受 QVariantList
                const QVariantList sub = it.value().toList();
                if (sub.size() != dim) continue;
                for (int k = 0; k < dim; ++k) {
                    const double v = sub[k].toDouble();
                    sim.qpos_[adr + k] = v;
                    sim.d_->qpos[adr + k] = v;
                }
                anyWritten = true;
            }
        }
        if (!anyWritten) return;
        mj_forward(sim.m_, sim.d_);
        QList<ContactInfo> contacts = buildContactSnapshot(sim.m_, sim.d_);
        result.reserve(contacts.size());
        for (const auto& c : contacts)
            result.append(QVariant::fromValue(c));
        QMetaObject::invokeMethod(this, [this, contacts = std::move(contacts)] {
            if (!sameContactSnapshot(m_contactSnapshot, contacts)) {
                m_contactSnapshot = contacts;
                emit contactsChanged();
            }
        }, Qt::QueuedConnection);
        sim.pending_.ui_update_simulation = true;
    });
    return result;
}

QVariantList MujocoQuickItem::setJointsAndDetect(const QVariantList& values)
{
    QVariantList result;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int nq = static_cast<int>(sim.m_->nq);
        // 允许只传入部分关节值：只写入前 count 个 qpos，其余保持当前值不变。
        const int count = qMin(static_cast<int>(values.size()), nq);
        if (count <= 0) return;
        // 写入 qpos_ 和 d->qpos，确保 mj_forward 读取最新值
        for (int i = 0; i < count; ++i) {
            sim.qpos_[i] = values[i].toDouble();
            sim.d_->qpos[i] = sim.qpos_[i];
        }
        // 同步执行前向运动学：更新所有 body 世界位姿 + 碰撞检测
        mj_forward(sim.m_, sim.d_);
        // 构建接触快照
        QList<ContactInfo> contacts = buildContactSnapshot(sim.m_, sim.d_);
        result.reserve(contacts.size());
        for (const auto& c : contacts)
            result.append(QVariant::fromValue(c));
        // 异步推送快照到 GUI 线程，保持 contactCount()/contacts() 一致
        QMetaObject::invokeMethod(this, [this, contacts = std::move(contacts)] {
            if (!sameContactSnapshot(m_contactSnapshot, contacts)) {
                m_contactSnapshot = contacts;
                emit contactsChanged();
            }
        }, Qt::QueuedConnection);
        sim.pending_.ui_update_simulation = true;
    });
    return result;
}

// ---------------------------------------------------------------------------
// 驱动器控制接口
// ---------------------------------------------------------------------------

// 把 mjtTrn / mjtGain / mjtBias 转为可读字符串
static const char* const kTrnTypeName[]  = {"joint", "jointInParent", "sliderCrank", "tendon", "site", "body"};
static const char* const kGainTypeName[] = {"fixed", "affine", "muscle", "dcmotor", "user"};
static const char* const kBiasTypeName[] = {"none", "affine", "muscle", "dcmotor", "user"};

int MujocoQuickItem::actuatorCount() const
{
    int count = 0;
    withSimulation([&](const mjModel* m, mjData*) {
        count = static_cast<int>(m->nu);
    });
    return count;
}

ActuatorInfo MujocoQuickItem::actuatorInfo(int index) const
{
    ActuatorInfo result;
    withSimulation([&](const mjModel* m, mjData*) {
        if (!isValidIndex(index, static_cast<int>(m->nu))) return;

        const char* rawName = mj_id2name(m, mjOBJ_ACTUATOR, index);
        result.name = rawName ? QString::fromUtf8(rawName)
                              : QStringLiteral("actuator_%1").arg(index);

        const int trn = m->actuator_trntype[index];
        result.trnType = trn;
        result.trnTypeName = (trn >= 0 && trn < static_cast<int>(std::size(kTrnTypeName)))
                                 ? QString::fromLatin1(kTrnTypeName[trn])
                                 : QStringLiteral("unknown");

        const int gn = m->actuator_gaintype[index];
        result.gainType = gn;
        result.gainTypeName = (gn >= 0 && gn < static_cast<int>(std::size(kGainTypeName)))
                                  ? QString::fromLatin1(kGainTypeName[gn])
                                  : QStringLiteral("unknown");

        const int bs = m->actuator_biastype[index];
        result.biasType = bs;
        result.biasTypeName = (bs >= 0 && bs < static_cast<int>(std::size(kBiasTypeName)))
                                  ? QString::fromLatin1(kBiasTypeName[bs])
                                  : QStringLiteral("unknown");

        result.ctrlMin  = m->actuator_ctrlrange[2 * index];
        result.ctrlMax  = m->actuator_ctrlrange[2 * index + 1];
        result.forceMin = m->actuator_forcerange[2 * index];
        result.forceMax = m->actuator_forcerange[2 * index + 1];
        result.gear     = m->actuator_gear[6 * index];

        // 提取关联 joint（通过 transmission 的第一个 id）
        const int jntId = m->actuator_trnid[2 * index];
        if (jntId >= 0 && jntId < m->njnt) {
            result.jointId = jntId;
            const char* jntName = mj_id2name(m, mjOBJ_JOINT, jntId);
            result.jointName = jntName ? QString::fromUtf8(jntName)
                                       : QStringLiteral("joint_%1").arg(jntId);
        }
    });
    return result;
}

int MujocoQuickItem::actuatorIndex(const QString& name) const
{
    int idx = -1;
    withSimulation([&](const mjModel* m, mjData*) {
        const int id = mj_name2id(m, mjOBJ_ACTUATOR, name.toUtf8().constData());
        idx = (id >= 0 && id < static_cast<int>(m->nu)) ? id : -1;
    });
    return idx;
}

double MujocoQuickItem::control(int index) const
{
    double val = std::numeric_limits<double>::quiet_NaN();
    withSimulation([&](const mjModel* m, mjData* d) {
        if (!isValidIndex(index, static_cast<int>(m->nu))) return;
        val = d->ctrl[index];
    });
    return val;
}

// 写穿到 d->ctrl：在 sim.mtx 锁内把控制值同时写入 staging(ctrl_)、物理数据
// (d_->ctrl) 与已应用值(ctrl_prev_)，让值在下一个 mj_step 立即生效，而不是攒在
// ctrl_ 里等下一帧 RenderLoop 的 Sync() 按渲染帧(约60Hz)批量写回——否则 50Hz 的
// 关节指令与渲染帧相位偶合时会"两帧一起跳"，物理端直线中段冒出随机突起。
static inline void applyControlImmediately(mujoco::Simulate& sim, int id, double value)
{
    if (!sim.m_ || id < 0 || id >= static_cast<int>(sim.m_->nu))
        return;
    sim.ctrl_[id] = value;
    if (sim.d_)
        sim.d_->ctrl[id] = value;
    if (static_cast<size_t>(id) < sim.ctrl_prev_.size())
        sim.ctrl_prev_[id] = value;
    sim.pending_.ui_update_simulation = true;
}

bool MujocoQuickItem::setControl(int index, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        if (!isValidIndex(index, static_cast<int>(sim.m_->nu))) return;
        applyControlImmediately(sim, index, value);
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setControlByName(const QString& name, double value)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int id = mj_name2id(sim.m_, mjOBJ_ACTUATOR, name.toUtf8().constData());
        if (!isValidIndex(id, static_cast<int>(sim.m_->nu))) return;
        applyControlImmediately(sim, id, value);
        applied = true;
    });
    return applied;
}

QVariantList MujocoQuickItem::controls() const
{
    QVariantList result;
    withSimulation([&](const mjModel* m, mjData* d) {
        result.reserve(static_cast<int>(m->nu));
        for (int i = 0; i < static_cast<int>(m->nu); ++i)
            result.append(d->ctrl[i]);
    });
    return result;
}

bool MujocoQuickItem::setControls(const QVariantList& values)
{
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_ || !sim.d_) return;
        const int nu = static_cast<int>(sim.m_->nu);
        if (values.size() != nu) return;
        for (int i = 0; i < nu; ++i)
            applyControlImmediately(sim, i, values[i].toDouble());
        applied = true;
    });
    return applied;
}

// ---------------------------------------------------------------------------
// 碰撞检测查询
// ---------------------------------------------------------------------------

int MujocoQuickItem::contactCount() const
{
    return m_contactSnapshot.size();
}

ContactInfo MujocoQuickItem::contact(int index) const
{
    if (index < 0 || index >= m_contactSnapshot.size()) return ContactInfo{};
    return m_contactSnapshot.at(index);
}

QVariantList MujocoQuickItem::contacts() const
{
    QVariantList result;
    result.reserve(m_contactSnapshot.size());
    for (const ContactInfo& c : m_contactSnapshot)
        result.append(QVariant::fromValue(c));
    return result;
}

bool MujocoQuickItem::setControlNoise(double scale, double rate) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.ctrl_noise_std = scale;
        sim.ctrl_noise_rate = rate;
        sim.pending_.ui_update_simulation = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setRealTimeSpeedIndex(int index) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const int count = int(sizeof(mujoco::Simulate::percentRealTime) /
                              sizeof(mujoco::Simulate::percentRealTime[0]));
        if (sim.is_passive_ || !isValidIndex(index, count)) return;
        sim.real_time_index = index;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::speedUpSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.is_passive_ || sim.real_time_index <= 0) return;
        sim.real_time_index--;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::slowDownSimulation() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const int count = int(sizeof(mujoco::Simulate::percentRealTime) /
                              sizeof(mujoco::Simulate::percentRealTime[0]));
        if (sim.is_passive_ || sim.real_time_index >= count - 1) return;
        sim.real_time_index++;
        sim.speed_changed = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFreeCamera() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.cam.type = mjCAMERA_FREE;
        sim.camera = 0;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

CameraState MujocoQuickItem::cameraState() {
    CameraState s;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        s.type         = sim.cam.type;
        s.fixedcamid   = sim.cam.fixedcamid;
        s.trackbodyid  = sim.cam.trackbodyid;
        s.lookat       = QVector3D(float(sim.cam.lookat[0]),
                                   float(sim.cam.lookat[1]),
                                   float(sim.cam.lookat[2]));
        s.distance     = sim.cam.distance;
        s.azimuth      = sim.cam.azimuth;
        s.elevation    = sim.cam.elevation;
        s.orthographic = (sim.cam.orthographic != 0);
    });
    return s;
}

bool MujocoQuickItem::setCameraState(const CameraState& state) {
    if (!m_sim) return false;

    const int duration = m_cameraTransitionDuration.load();

    if (duration <= 0) {
        // 无过渡：立即跳转（原有行为）
        bool applied = false;
        bool orthoChanged = false;
        withSimulateLocked([&](mujoco::Simulate& sim) {
            const int oldOrtho = sim.cam.orthographic;
            sim.cam.type          = state.type;
            sim.cam.fixedcamid    = state.fixedcamid;
            sim.cam.trackbodyid   = state.trackbodyid;
            sim.cam.lookat[0]     = state.lookat.x();
            sim.cam.lookat[1]     = state.lookat.y();
            sim.cam.lookat[2]     = state.lookat.z();
            sim.cam.distance      = state.distance;
            sim.cam.azimuth       = state.azimuth;
            sim.cam.elevation     = state.elevation;
            sim.cam.orthographic  = state.orthographic ? 1 : 0;
            if (state.type == mjCAMERA_FIXED) {
                sim.camera = state.fixedcamid + 2;
            } else if (state.type == mjCAMERA_TRACKING) {
                sim.camera = 1;
            } else {
                sim.camera = 0;
            }
            sim.pending_.ui_update_rendering = true;
            orthoChanged = (oldOrtho != sim.cam.orthographic);
            applied = true;
        });
        if (orthoChanged) emit orthographicCameraChanged();
        return applied;
    }

    // 有过渡：捕获当前相机状态作为起点，后续由 onFrameRendered 驱动插值
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        // 捕获起点（当前 sim.cam）
        m_camTransition.start.type         = sim.cam.type;
        m_camTransition.start.fixedcamid   = sim.cam.fixedcamid;
        m_camTransition.start.trackbodyid  = sim.cam.trackbodyid;
        m_camTransition.start.lookat       = QVector3D(float(sim.cam.lookat[0]),
                                                        float(sim.cam.lookat[1]),
                                                        float(sim.cam.lookat[2]));
        m_camTransition.start.distance     = sim.cam.distance;
        m_camTransition.start.azimuth      = sim.cam.azimuth;
        m_camTransition.start.elevation    = sim.cam.elevation;
        m_camTransition.start.orthographic = (sim.cam.orthographic != 0);

        m_camTransition.target      = state;
        m_camTransition.startTime   = std::chrono::steady_clock::now();
        m_camTransition.durationMs  = duration;
        m_camTransition.active      = true;

        // 离散字段立即生效（不参与插值）
        sim.cam.type         = state.type;
        sim.cam.fixedcamid   = state.fixedcamid;
        sim.cam.trackbodyid  = state.trackbodyid;
        sim.cam.orthographic = state.orthographic ? 1 : 0;
        if (state.type == mjCAMERA_FIXED) {
            sim.camera = state.fixedcamid + 2;
        } else if (state.type == mjCAMERA_TRACKING) {
            sim.camera = 1;
        } else {
            sim.camera = 0;
        }
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::resetCameraToDefault() {
    // 取消任何活跃的相机过渡
    withSimulateLocked([&](mujoco::Simulate& sim) {
        m_camTransition.active = false;
    });

    bool applied = false;
    bool orthoChanged = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const int oldOrtho = sim.cam.orthographic;
        mjv_defaultCamera(&sim.cam);
        sim.camera = 0;
        sim.pending_.ui_update_rendering = true;
        orthoChanged = (oldOrtho != sim.cam.orthographic);
        applied = true;
    });
    if (orthoChanged) emit orthographicCameraChanged();
    return applied;
}

void MujocoQuickItem::applyCameraTransitionLocked(mujoco::Simulate& sim) {
    if (!m_camTransition.active) return;

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - m_camTransition.startTime).count();

    float t = std::min(1.0f, float(elapsed) / float(m_camTransition.durationMs));
    const float et = easeInOutCubic(t);

    const auto& s = m_camTransition.start;
    const auto& g = m_camTransition.target;

    // 线性插值连续参数
    sim.cam.lookat[0] = s.lookat.x() + (g.lookat.x() - s.lookat.x()) * et;
    sim.cam.lookat[1] = s.lookat.y() + (g.lookat.y() - s.lookat.y()) * et;
    sim.cam.lookat[2] = s.lookat.z() + (g.lookat.z() - s.lookat.z()) * et;
    sim.cam.distance  = s.distance + (g.distance - s.distance) * et;
    sim.cam.azimuth   = s.azimuth + (g.azimuth - s.azimuth) * et;
    sim.cam.elevation = s.elevation + (g.elevation - s.elevation) * et;

    sim.pending_.ui_update_rendering = true;

    if (t >= 1.0f) {
        m_camTransition.active = false;
    }
}

void MujocoQuickItem::setCameraTransitionDuration(int ms) {
    if (ms < 0) ms = 0;
    if (m_cameraTransitionDuration.exchange(ms) == ms) return;
    emit cameraTransitionDurationChanged();
}

bool MujocoQuickItem::orthographicCamera() {
    bool ortho = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        const mjModel* m = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (m) ortho = (m->vis.global.orthographic != 0);
    });
    return ortho;
}

bool MujocoQuickItem::setOrthographicCamera(bool orthographic) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* m = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (!m) return;
        const int val = orthographic ? 1 : 0;
        if (m->vis.global.orthographic == val) return;
        m->vis.global.orthographic = val;
        // 同步更新 cam.orthographic 以保持一致性（虽然 free/tracking 相机不读它，
        // 但固定相机路径会用到）
        sim.cam.orthographic = val;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    if (applied) emit orthographicCameraChanged();
    return applied;
}

bool MujocoQuickItem::setTrackingCamera(int bodyId) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (!model) return;
        int target = bodyId >= 0 ? bodyId : sim.pert.select;
        if (target <= 0 || target >= model->nbody) return;
        sim.cam.type = mjCAMERA_TRACKING;
        sim.cam.trackbodyid = target;
        sim.cam.fixedcamid = -1;
        sim.camera = 1;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFixedCamera(int cameraIndex) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(cameraIndex, sim.ncam_)) return;
        sim.cam.type = mjCAMERA_FIXED;
        sim.cam.fixedcamid = cameraIndex;
        sim.camera = cameraIndex + 2;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::cycleFixedCamera(int direction) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.ncam_ <= 0 || direction == 0) return;
        int current = sim.cam.type == mjCAMERA_FIXED ? sim.cam.fixedcamid : -1;
        int next = current + (direction > 0 ? 1 : -1);
        if (next < 0) next = sim.ncam_ - 1;
        if (next >= sim.ncam_) next = 0;
        sim.cam.type = mjCAMERA_FIXED;
        sim.cam.fixedcamid = next;
        sim.camera = next + 2;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::alignView() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!sim.m_) return;
        sim.pending_.align = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setFrameVisualization(int frame) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(frame, mjNFRAME)) return;
        sim.opt.frame = frame;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::cycleFrameVisualization() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.opt.frame = (sim.opt.frame + 1) % mjNFRAME;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setLabelVisualization(int label) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(label, mjNLABEL)) return;
        sim.opt.label = label;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

// 将标签模式循环切换到下一种（超过 mjNLABEL 后回绕到 0）。
// 无需模型已加载，即使在加载前调用也会更新选项。
bool MujocoQuickItem::cycleLabelVisualization() {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.opt.label = (sim.opt.label + 1) % mjNLABEL;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setVisualizationFlag(int flag, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(flag, mjNVISFLAG)) return;
        sim.opt.flags[flag] = enabled;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setRenderingFlag(int flag, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (!isValidIndex(flag, mjNRNDFLAG)) return;
        sim.scn.flags[flag] = enabled;
        sim.pending_.ui_update_rendering = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setVisualGroupVisible(unsigned char* groups, int group, bool visible) {
    if (!groups || !isValidIndex(group, mjNGROUP)) return false;
    groups[group] = visible;
    return true;
}

bool MujocoQuickItem::setGeomGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.geomgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setSiteGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.sitegroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setJointGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.jointgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setTendonGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.tendongroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setActuatorGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.actuatorgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setFlexGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.flexgroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setSkinGroupVisible(int group, bool visible) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        applied = setVisualGroupVisible(sim.opt.skingroup, group, visible);
        if (applied) sim.pending_.ui_update_rendering = true;
    });
    return applied;
}

bool MujocoQuickItem::setPhysicsDisableFlag(int disableBit, bool disabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        const int index = bitFlagIndex(disableBit, mjNDISABLE);
        if (!model || index < 0) return;
        sim.disable[index] = boolToInt(disabled);
        if (disabled) model->opt.disableflags |= disableBit;
        else model->opt.disableflags &= ~disableBit;
        sim.pending_.ui_update_physics = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setPhysicsEnableFlag(int enableBit, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        const int index = bitFlagIndex(enableBit, mjNENABLE);
        if (!model || index < 0) return;
        sim.enable[index] = boolToInt(enabled);
        if (enabled) model->opt.enableflags |= enableBit;
        else model->opt.enableflags &= ~enableBit;
        sim.pending_.ui_update_physics = true;
        applied = true;
    });
    return applied;
}

bool MujocoQuickItem::setActuatorGroupEnabled(int group, bool enabled) {
    bool applied = false;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        mjModel* model = sim.is_passive_ ? sim.m_passive_ : sim.m_;
        if (!model || !isValidIndex(group, mjNGROUP)) return;
        sim.enableactuator[group] = boolToInt(enabled);
        if (enabled) model->opt.disableactuator &= ~(1 << group);
        else model->opt.disableactuator |= (1 << group);
        sim.pending_.ui_update_physics = true;
        sim.pending_.ui_remake_ctrl = true;
        applied = true;
    });
    return applied;
}

void MujocoQuickItem::setHelpVisible(bool visible) {
    if (m_helpVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.help = boolToInt(visible); });
    emit helpVisibleChanged();
}

void MujocoQuickItem::setInfoVisible(bool visible) {
    if (m_infoVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.info = boolToInt(visible); });
    emit infoVisibleChanged();
}

void MujocoQuickItem::setProfilerVisible(bool visible) {
    if (m_profilerVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.profiler = boolToInt(visible); });
    emit profilerVisibleChanged();
}

void MujocoQuickItem::setSensorVisible(bool visible) {
    if (m_sensorVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.sensor = boolToInt(visible); });
    emit sensorVisibleChanged();
}

void MujocoQuickItem::setPauseUpdateEnabled(bool enabled) {
    if (m_pauseUpdateEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.pause_update = boolToInt(enabled); });
    emit pauseUpdateEnabledChanged();
}

void MujocoQuickItem::setFullscreenRequested(bool enabled) {
    if (m_fullscreenRequested.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        if (sim.fullscreen != boolToInt(enabled) && sim.platform_ui) {
            sim.platform_ui->ToggleFullscreen();
        }
        sim.fullscreen = boolToInt(enabled);
    });
    emit fullscreenRequestedChanged();
}

void MujocoQuickItem::setVSyncEnabled(bool enabled) {
    if (m_vSyncEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) {
        sim.vsync = boolToInt(enabled);
        if (sim.platform_ui) sim.platform_ui->SetVSync(enabled);
    });
    emit vSyncEnabledChanged();
}

void MujocoQuickItem::setBusyWaitEnabled(bool enabled) {
    if (m_busyWaitEnabled.exchange(enabled) == enabled) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.busywait = boolToInt(enabled); });
    emit busyWaitEnabledChanged();
}

void MujocoQuickItem::setLeftUiVisible(bool visible) {
    if (m_leftUiVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.ui0_enable = boolToInt(visible); });
    emit leftUiVisibleChanged();
}

void MujocoQuickItem::setRightUiVisible(bool visible) {
    if (m_rightUiVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.ui1_enable = boolToInt(visible); });
    emit rightUiVisibleChanged();
}

void MujocoQuickItem::setStatusOverlayVisible(bool visible) {
    if (m_statusOverlayVisible.exchange(visible) == visible) return;
    withSimulateLocked([&](mujoco::Simulate& sim) { sim.status_overlay = boolToInt(visible); });
    emit statusOverlayVisibleChanged();
}

// ----------------------------------------------------------------- IMujocoHost
void MujocoQuickItem::onFrameRendered() {
    // 在 mujoco 渲染线程被调用，转发到 GUI 线程触发 update()
    const QString statusText = m_sim ? QString::fromUtf8(m_sim->status_overlay_text) : QString();

    // 在锁内采样当前帧接触信息（sim.mtx 是 recursive_mutex，渲染线程可能已持锁）
    QList<ContactInfo> contacts;
    bool simRunChanged = false;
    int  histScrub = m_historyScrubIndex;
    int  histCap   = m_historyCapacity;
    if (m_sim) {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        if (m_sim->m_ && m_sim->d_) {
            contacts = buildContactSnapshot(m_sim->m_, m_sim->d_);
            sampleTrackedTrajectoriesLocked(m_sim->m_, m_sim->d_);
        }
        if (m_sim->m_) {
            histScrub = m_sim->scrub_index;
            histCap   = m_sim->nhistory_ > 0 ? m_sim->nhistory_ - 1 : 0;
        }
        // stopWhenSettled：等机械臂停稳（或安全超时）后自动停仿真
        if (m_settleStop.active && m_sim->run != 0 && m_sim->m_ && m_sim->d_) {
            const qint64 now = mqiNowMs();
            bool stop = false;
            if (isSettledLocked(m_sim->m_, m_sim->d_,
                                m_settleStop.posTol, m_settleStop.velTol)) {
                if (m_settleStop.settledMs < 0) m_settleStop.settledMs = now;
                if (now - m_settleStop.settledMs >= m_settleStop.stableMs) stop = true;
            } else {
                m_settleStop.settledMs = -1;
            }
            // 被障碍卡住无法到位时兜底，避免一直空转
            if (!stop && m_settleStop.timeoutMs > 0 &&
                now - m_settleStop.armMs >= m_settleStop.timeoutMs) {
                stop = true;
            }
            if (stop) {
                m_settleStop.active = false;
                m_sim->run = 0;
                m_sim->pending_.ui_update_simulation = true;
            }
        }
        // 键盘/UI 可能直接修改了 sim.run（例如空格键），同步回 m_simulationRunning
        const bool simRunning = (m_sim->run != 0);
        if (m_simulationRunning.load() != simRunning) {
            m_simulationRunning.store(simRunning);
            simRunChanged = true;
        }
        // 相机平滑过渡：每帧按流逝时间插值
        applyCameraTransitionLocked(*m_sim);
    }

    QMetaObject::invokeMethod(this, [this, statusText, contacts = std::move(contacts), simRunChanged, histScrub, histCap] {
        if (simRunChanged) emit simulationRunningChanged();
        if (m_historyCapacity != histCap) { m_historyCapacity = histCap; emit historyCapacityChanged(); }
        if (m_historyScrubIndex != histScrub) { m_historyScrubIndex = histScrub; emit historyScrubIndexChanged(); }
        const int depthNow = m_historyDepth.load();
        if (m_historyDepthEmitted != depthNow) { m_historyDepthEmitted = depthNow; emit historyDepthChanged(); }
        if (m_statusOverlayText != statusText) {
            m_statusOverlayText = statusText;
            emit statusOverlayTextChanged();
        }
        if (!sameContactSnapshot(m_contactSnapshot, contacts)) {
            m_contactSnapshot = std::move(contacts);
            emit contactsChanged();
        }
        update();
    }, Qt::QueuedConnection);
}
void MujocoQuickItem::onSetTitle(const QString& t) {
    // 注意：作为嵌入式 QQuickItem，不应擅自修改宿主窗口标题
    // （否则会出现 QML Window 标题被改成 "MuJoCo : <model>" 的问题）。
    // 仅通过信号把标题对外暴露，由上层 QML/C++ 自行决定如何使用。
    QMetaObject::invokeMethod(this, [this, t] {
        if (m_modelTitle == t) return;
        m_modelTitle = t;
        emit modelTitleChanged();
    }, Qt::QueuedConnection);
}
void MujocoQuickItem::onToggleFullscreen() {
    QMetaObject::invokeMethod(this, [this] {
        QQuickWindow* w = window();
        if (!w) return;
        w->setVisibility(w->visibility() == QWindow::FullScreen
                         ? QWindow::Windowed : QWindow::FullScreen);
    }, Qt::QueuedConnection);
}

// ----------------------------------------------------------------- 几何 ----
void MujocoQuickItem::updateGeometryToAdapter() {
    if (!m_adapterRaw) return;
    qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
    int w  = int(width()), h = int(height());
    int fbw = int(width() * dpr), fbh = int(height() * dpr);
    if (fbw <= 0) fbw = 1;
    if (fbh <= 0) fbh = 1;
    QScreen* scr = window() ? window()->screen() : QGuiApplication::primaryScreen();
    double dpi = scr ? scr->logicalDotsPerInch() : 96.0;
    m_adapterRaw->SetWindowGeometry(w, h, fbw, fbh, dpi);
    m_adapterRaw->PostResize(fbw, fbh);
}

#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
void MujocoQuickItem::geometryChange(const QRectF& newGeo, const QRectF& oldGeo) {
    QQuickFramebufferObject::geometryChange(newGeo, oldGeo);
    updateGeometryToAdapter();
}
#else
void MujocoQuickItem::geometryChanged(const QRectF& newGeo, const QRectF& oldGeo) {
    QQuickFramebufferObject::geometryChanged(newGeo, oldGeo);
    updateGeometryToAdapter();
}
#endif

void MujocoQuickItem::itemChange(ItemChange change, const ItemChangeData& data) {
    QQuickFramebufferObject::itemChange(change, data);
    if (change == ItemSceneChange) {
        updateGeometryToAdapter();
    }
}

// ----------------------------------------------------------------- 输入 ----
int MujocoQuickItem::qtMouseButtonToInternal(int btn) const {
    if (btn == Qt::LeftButton)   return 1;
    if (btn == Qt::RightButton)  return 2;
    if (btn == Qt::MiddleButton) return 3;
    return 0;
}
void MujocoQuickItem::updateModifiersFrom(int qtMods) {
    if (!m_adapterRaw) return;
    m_adapterRaw->SetModifiers(qtMods & Qt::ControlModifier,
                               qtMods & Qt::ShiftModifier,
                               qtMods & Qt::AltModifier);
}

void MujocoQuickItem::mousePressEvent(QMouseEvent* e) {
    forceActiveFocus();
    emit mousePressed();
    if (!m_adapterRaw) { QQuickFramebufferObject::mousePressEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 1,
                                  e->pos().x(), e->pos().y());
    e->accept();
}
void MujocoQuickItem::mouseReleaseEvent(QMouseEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::mouseReleaseEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    m_adapterRaw->PostMouseButton(qtMouseButtonToInternal(int(e->button())), 0,
                                  e->pos().x(), e->pos().y());
    e->accept();
}
void MujocoQuickItem::mouseMoveEvent(QMouseEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::mouseMoveEvent(e); return; }
    updateModifiersFrom(int(e->modifiers()));
    m_adapterRaw->PostMouseMove(e->pos().x(), e->pos().y());
    e->accept();
}
void MujocoQuickItem::hoverMoveEvent(QHoverEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::hoverMoveEvent(e); return; }
    m_adapterRaw->PostMouseMove(e->pos().x(), e->pos().y());
    e->accept();
}
void MujocoQuickItem::wheelEvent(QWheelEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::wheelEvent(e); return; }
    QPoint d = e->angleDelta();
    m_adapterRaw->PostScroll(d.x() / 120.0, d.y() / 120.0);
    e->accept();
}
int MujocoQuickItem::qtKeyToMjui(int key) const {
    if (key >= Qt::Key_Space && key <= Qt::Key_AsciiTilde) return key;
    switch (key) {
        case Qt::Key_Escape:    return 256;
        case Qt::Key_Enter:
        case Qt::Key_Return:    return 257;
        case Qt::Key_Tab:       return 258;
        case Qt::Key_Backtab:   return 258; // Shift+Tab → Tab + Shift modifier already set
        case Qt::Key_Backspace: return 259;
        case Qt::Key_Insert:    return 260;
        case Qt::Key_Delete:    return 261;
        case Qt::Key_Right:     return 262;
        case Qt::Key_Left:      return 263;
        case Qt::Key_Down:      return 264;
        case Qt::Key_Up:        return 265;
        case Qt::Key_PageUp:    return 266;
        case Qt::Key_PageDown:  return 267;
        case Qt::Key_Home:      return 268;
        case Qt::Key_End:       return 269;
        case Qt::Key_F1:        return 290;
        case Qt::Key_F2:        return 291;
        case Qt::Key_F3:        return 292;
        case Qt::Key_F4:        return 293;
        case Qt::Key_F5:        return 294;
        case Qt::Key_F6:        return 295;
        case Qt::Key_F7:        return 296;
        case Qt::Key_F8:        return 297;
        case Qt::Key_F9:        return 298;
        case Qt::Key_F10:       return 299;
        case Qt::Key_F11:       return 300;
        case Qt::Key_F12:       return 301;
        default:                return 0;
    }
}
void MujocoQuickItem::keyPressEvent(QKeyEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::keyPressEvent(e); return; }
    int mods = int(e->modifiers());
    if (e->key() == Qt::Key_Backtab) mods |= Qt::ShiftModifier;
    updateModifiersFrom(mods);
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 1);
    e->accept();
}
void MujocoQuickItem::keyReleaseEvent(QKeyEvent* e) {
    if (!m_adapterRaw) { QQuickFramebufferObject::keyReleaseEvent(e); return; }
    int mods = int(e->modifiers());
    if (e->key() == Qt::Key_Backtab) mods |= Qt::ShiftModifier;
    updateModifiersFrom(mods);
    int k = qtKeyToMjui(e->key());
    if (k) m_adapterRaw->PostKey(k, 0);
    e->accept();
}

// ----------------------------------------------------------------- 渲染线程 ----
void MujocoQuickItem::renderThreadMain() {
    m_ctx->moveToThread(QThread::currentThread());
    m_surface->moveToThread(QThread::currentThread());
    if (!m_ctx->makeCurrent(m_surface)) {
        qWarning() << "MujocoQuickItem: makeCurrent failed";
        return;
    }

    if (auto* f = m_ctx->extraFunctions()) {
        f->glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    }

    static mjvCamera  cam;  mjv_defaultCamera(&cam);
    static mjvOption  opt;  mjv_defaultOption(&opt);
    static mjvPerturb pert; mjv_defaultPerturb(&pert);

    auto adapter_unique = std::make_unique<mjqt::QtPlatformUIAdapter>(this);
    m_adapterRaw = adapter_unique.get();

    m_sim = std::make_unique<mujoco::Simulate>(
        std::move(adapter_unique),
        &cam, &opt, &pert, /*is_passive=*/false);

    {
        std::unique_lock<std::recursive_mutex> lk(m_sim->mtx);
        applyBooleanPropertiesTo(*m_sim);
    }

    // adapter 已就绪，把当前几何信息再推一次
    QMetaObject::invokeMethod(this, [this] { updateGeometryToAdapter(); },
                              Qt::QueuedConnection);

    m_sim->RenderLoop();

    // 释放点云 GPU 资源（仍在渲染线程、GL context 当前）。
    if (m_pointRenderer) m_pointRenderer->releaseGL();

    // 释放适配器持有的 GL 资源（共享纹理 / FBO），此时 GL context 仍在当前线程
    if (m_adapterRaw) m_adapterRaw->ReleaseSharedGL();

    m_ctx->doneCurrent();

    // 把 QObject 亲和性交还为 nullptr，主线程会在 stop() 中拾回并销毁。
    // 这是 Qt 跨线程交接 QObject 的合法做法；moveToThread 必须
    // 在当前拥有该对象的线程中调用。
    m_ctx->moveToThread(nullptr);
    m_surface->moveToThread(nullptr);
}

// ----------------------------------------------------------------- 物理线程 ----
namespace {
struct LoadedModel {
    mjModel* model = nullptr;
    mjSpec* spec = nullptr;

    ~LoadedModel() {
        if (model) mj_deleteModel(model);
        if (spec) mj_deleteSpec(spec);
    }

    LoadedModel() = default;
    LoadedModel(const LoadedModel&) = delete;
    LoadedModel& operator=(const LoadedModel&) = delete;

    LoadedModel(LoadedModel&& other) noexcept
        : model(other.model), spec(other.spec) {
        other.model = nullptr;
        other.spec = nullptr;
    }

    LoadedModel& operator=(LoadedModel&& other) noexcept {
        if (this == &other) return *this;
        if (model) mj_deleteModel(model);
        if (spec) mj_deleteSpec(spec);
        model = other.model;
        spec = other.spec;
        other.model = nullptr;
        other.spec = nullptr;
        return *this;
    }
};

LoadedModel loadModelFile(const QString& filename, mujoco::Simulate& sim) {
    char err[kErrorLength] = "";
    QByteArray utf8 = filename.toLocal8Bit();
    LoadedModel result;
    if (filename.endsWith(".mjb", Qt::CaseInsensitive)) {
        result.model = mj_loadModel(utf8.constData(), nullptr);
        if (!result.model) std::strncpy(err, "could not load binary model", sizeof(err) - 1);
    } else {
        result.spec = filename.endsWith(".xml", Qt::CaseInsensitive)
            ? mj_parseXML(utf8.constData(), nullptr, err, sizeof(err))
            : mj_parse(utf8.constData(), nullptr, nullptr, err, sizeof(err));
        if (result.spec) {
            result.model = mj_compile(result.spec, nullptr);
            if (!result.model) {
                const char* specErr = mjs_getError(result.spec);
                if (specErr && specErr[0]) std::strncpy(err, specErr, sizeof(err) - 1);
            }
        }
    }
    if (!result.model) {
        std::strncpy(sim.load_error, err, sizeof(sim.load_error) - 1);
        std::printf("loadModel error: %s\n", err);
    }
    return result;
}
} // namespace

void MujocoQuickItem::physicsThreadMain() {
    while (m_running.load() && !m_sim) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!m_sim) return;

    auto& sim = *m_sim;
    mjModel* m = nullptr;
    mjData*  d = nullptr;

    using Clock = mujoco::Simulate::Clock;
    std::chrono::time_point<Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim.exitrequest.load() && m_running.load()) {
        if (m_hasPendingLoad.exchange(false)) {
            QString file;
            { std::lock_guard<std::mutex> lk(m_pendingMtx); file = m_pendingFile; }
            sim.LoadMessage(file.toLocal8Bit().constData());
            LoadedModel loaded = loadModelFile(file, sim);
            mjModel* mnew = loaded.model;
            mjData*  dnew = mnew ? mj_makeData(mnew) : nullptr;
            if (dnew) {
                sim.Load(mnew, dnew, file.toLocal8Bit().constData());
                m_historyDepth.store(0);   // 新模型：history 缓冲已重建，回退深度归零
                std::unique_lock<std::recursive_mutex> lk(sim.mtx);
                if (d) mj_deleteData(d);
                if (m) mj_deleteModel(m);
                if (m_editSpec) mj_deleteSpec(m_editSpec);
                m_editSpec = loaded.spec;
                loaded.spec = nullptr;
                loaded.model = nullptr;
                m = mnew; d = dnew;
                mj_forward(m, d);
                lk.unlock();
                setLastError(QString());
                const QString src = file;
                QMetaObject::invokeMethod(this, [this, src]{ emit sceneLoaded(src); },
                                          Qt::QueuedConnection);
            } else {
                sim.LoadMessageClear();
                const QString reason = QString::fromLocal8Bit(sim.load_error);
                setLastError(reason);
                QMetaObject::invokeMethod(this, [this, reason]{ emit sceneLoadFailed(reason); },
                                          Qt::QueuedConnection);
            }
        }

        if (sim.uiloadrequest.load()) {
            sim.uiloadrequest.fetch_sub(1);
            sim.LoadMessage(sim.filename);
            LoadedModel loaded = loadModelFile(QString::fromLocal8Bit(sim.filename), sim);
            mjModel* mnew = loaded.model;
            mjData*  dnew = mnew ? mj_makeData(mnew) : nullptr;
            if (dnew) {
                sim.Load(mnew, dnew, sim.filename);
                m_historyDepth.store(0);   // 新模型：history 缓冲已重建，回退深度归零
                std::unique_lock<std::recursive_mutex> lk(sim.mtx);
                if (d) mj_deleteData(d);
                if (m) mj_deleteModel(m);
                if (m_editSpec) mj_deleteSpec(m_editSpec);
                m_editSpec = loaded.spec;
                loaded.spec = nullptr;
                loaded.model = nullptr;
                m = mnew; d = dnew;
                mj_forward(m, d);
                lk.unlock();
                setLastError(QString());
                const QString src = QString::fromLocal8Bit(sim.filename);
                QMetaObject::invokeMethod(this, [this, src]{ emit sceneLoaded(src); },
                                          Qt::QueuedConnection);
            } else {
                sim.LoadMessageClear();
                const QString reason = QString::fromLocal8Bit(sim.load_error);
                setLastError(reason);
                QMetaObject::invokeMethod(this, [this, reason]{ emit sceneLoadFailed(reason); },
                                          Qt::QueuedConnection);
            }
        }

        if (sim.run && sim.busywait) std::this_thread::yield();
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));

        std::unique_lock<std::recursive_mutex> lk(sim.mtx);
        if (!m) continue;

        if (sim.run) {
            const auto startCPU   = Clock::now();
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim     = d->time - syncSim;
            double slowdown       = 100.0 / sim.percentRealTime[sim.real_time_index];
            bool misaligned = std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

            if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
                syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed) {
                syncCPU = startCPU; syncSim = d->time;
                sim.speed_changed = false;
                sim.InjectNoise(sim.key);
                mj_step(m, d);
                sim.AddToHistory();
                bumpHistoryDepth(sim);
            } else {
                mjtNum prevSim = d->time;
                double refreshTime = simRefreshFraction / sim.refresh_rate;
                while (Seconds((d->time - syncSim)*slowdown) < Clock::now() - syncCPU &&
                       Clock::now() - startCPU < Seconds(refreshTime)) {
                    sim.InjectNoise(sim.key);
                    mj_step(m, d);
                    if (d->time < prevSim) break;
                }
                sim.AddToHistory();
                bumpHistoryDepth(sim);
            }
        } else {
            mj_forward(m, d);
            if (sim.pause_update) mju_copy(d->qacc_warmstart, d->qacc, m->nv);
            sim.speed_changed = true;
        }
    }

    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
    if (m_editSpec) {
        mj_deleteSpec(m_editSpec);
        m_editSpec = nullptr;
    }
}
