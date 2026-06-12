#pragma once
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QQuaternion>
#include <QMetaType>
#include <QList>

// ---------------------------------------------------------------------------
// JointInfo — 关节固有属性描述。支持 Q_GADGET，可在 QML 中直接读取属性。
//
// type 对应 mjtJoint：
//   0 = free  (7 qpos：3 位置 + 4 四元数)
//   1 = ball  (4 qpos：四元数)
//   2 = slide (1 qpos：滑移距离，单位米)
//   3 = hinge (1 qpos：旋转角度，单位弧度)
// ---------------------------------------------------------------------------
struct JointInfo {
    Q_GADGET
    Q_PROPERTY(QString name      MEMBER name      CONSTANT)
    Q_PROPERTY(int     type      MEMBER type      CONSTANT)
    Q_PROPERTY(QString typeName  MEMBER typeName  CONSTANT)
    Q_PROPERTY(int     qposDim   MEMBER qposDim   CONSTANT)
    Q_PROPERTY(bool    limited   MEMBER limited   CONSTANT)
    Q_PROPERTY(double  rangeMin  MEMBER rangeMin  CONSTANT)
    Q_PROPERTY(double  rangeMax  MEMBER rangeMax  CONSTANT)
    Q_PROPERTY(double  stiffness MEMBER stiffness CONSTANT)
    Q_PROPERTY(int     qposadr   MEMBER qposadr   CONSTANT)
public:
    QString name;
    int     type      = 0;    // mjtJoint 枚举值
    QString typeName;         // "free" / "ball" / "slide" / "hinge"
    int     qposDim   = 1;    // qpos 维度
    bool    limited   = false;
    double  rangeMin  = 0.0;  // limited=false 时为 0
    double  rangeMax  = 0.0;
    double  stiffness = 0.0;
    int     qposadr   = 0;    // 在全局 qpos 数组中的起始下标
};
Q_DECLARE_METATYPE(JointInfo)

// ---------------------------------------------------------------------------
// ActuatorInfo — 驱动器固有属性描述。支持 Q_GADGET，可在 QML 中直接读取。
//
// trnType 对应 mjtTrn（传输类型）：
//   0 = joint  (作用在关节上，如位置/速度伺服)
//   1 = jointInParent
//   2 = sliderCrank
//   3 = tendon
//   4 = site
//   5 = body   (adhesion)
//
// gainType 对应 mjtGain（增益类型）：
//   0 = fixed    (简单力/力矩输入)
//   1 = affine   (位置/速度相关增益)
//   2 = muscle
//   3 = dcmotor
//   4 = user
//
// biasType 对应 mjtBias（偏置类型）：
//   0 = none     (无偏置)
//   1 = affine   (kp*kv 伺服，如 position/velocity actuator)
//   2 = muscle
//   3 = dcmotor
//   4 = user
//
// ctrlRange：控制值范围（XML ctrlrange）。
// forceRange：力值范围（XML forcerange）。
// gear[0]：传动比第一分量。
// jointId：关联的 joint id（-1 表示无关联 joint）。
// jointName：关联的 joint 名称。伺服类 actuator 通过 trnid[0] 绑定 joint。
// ---------------------------------------------------------------------------
struct ActuatorInfo {
    Q_GADGET
    Q_PROPERTY(QString name         MEMBER name         CONSTANT)
    Q_PROPERTY(int     trnType      MEMBER trnType      CONSTANT)
    Q_PROPERTY(QString trnTypeName  MEMBER trnTypeName  CONSTANT)
    Q_PROPERTY(int     gainType     MEMBER gainType     CONSTANT)
    Q_PROPERTY(QString gainTypeName MEMBER gainTypeName CONSTANT)
    Q_PROPERTY(int     biasType     MEMBER biasType     CONSTANT)
    Q_PROPERTY(QString biasTypeName MEMBER biasTypeName CONSTANT)
    Q_PROPERTY(int     jointId      MEMBER jointId      CONSTANT)
    Q_PROPERTY(QString jointName    MEMBER jointName    CONSTANT)
    Q_PROPERTY(double  ctrlMin      MEMBER ctrlMin      CONSTANT)
    Q_PROPERTY(double  ctrlMax      MEMBER ctrlMax      CONSTANT)
    Q_PROPERTY(double  forceMin     MEMBER forceMin     CONSTANT)
    Q_PROPERTY(double  forceMax     MEMBER forceMax     CONSTANT)
    Q_PROPERTY(double  gear         MEMBER gear         CONSTANT)
public:
    QString name;
    int     trnType      = 0;
    QString trnTypeName;
    int     gainType     = 0;
    QString gainTypeName;
    int     biasType     = 0;
    QString biasTypeName;
    int     jointId      = -1;
    QString jointName;
    double  ctrlMin      = 0.0;
    double  ctrlMax      = 0.0;
    double  forceMin     = 0.0;
    double  forceMax     = 0.0;
    double  gear         = 0.0;
};
Q_DECLARE_METATYPE(ActuatorInfo)

// ---------------------------------------------------------------------------
// SceneObjectInfo — 场景 body 的基础属性快照。objectCount()/objectInfo()
// 使用与 MuJoCo 一致的 body id，包含 world body (body id 0)。
//
// 字段说明：
//   bodyId / name         — body id 和名称；无名时用 "#<id>"。
//   parentBodyId / parentName — 父 body。
//   position             — body 当前世界坐标（来自 mjData::xpos）。
//   localPosition        — body 相对父 body 的模型坐标（来自 mjModel::body_pos）。
//   orientation          — body 当前世界姿态四元数 [w,x,y,z]（来自 mjData::xquat）。
//   localOrientation     — body 相对父 body 的局部姿态四元数（来自 mjModel::body_quat）。
//   mass                 — body 质量。
//   jointCount / geomCount — 当前 body 直属 joint / geom 数量。
//   movable              — 是否有自由度；true 时位置通常由 qpos 控制。
//   firstGeom*           — 第一个直属 geom 的基础信息，便于简单编辑器显示。
// ---------------------------------------------------------------------------
struct SceneObjectInfo {
    Q_GADGET
    Q_PROPERTY(int      bodyId       MEMBER bodyId       CONSTANT)
    Q_PROPERTY(QString  name         MEMBER name         CONSTANT)
    Q_PROPERTY(int      parentBodyId MEMBER parentBodyId CONSTANT)
    Q_PROPERTY(QString  parentName   MEMBER parentName   CONSTANT)
    Q_PROPERTY(QVector3D position    MEMBER position     CONSTANT)
    Q_PROPERTY(QVector3D localPosition MEMBER localPosition CONSTANT)
    Q_PROPERTY(QQuaternion orientation MEMBER orientation CONSTANT)
    Q_PROPERTY(QQuaternion localOrientation MEMBER localOrientation CONSTANT)
    Q_PROPERTY(double   mass         MEMBER mass         CONSTANT)
    Q_PROPERTY(int      jointCount   MEMBER jointCount   CONSTANT)
    Q_PROPERTY(int      geomCount    MEMBER geomCount    CONSTANT)
    Q_PROPERTY(bool     movable      MEMBER movable      CONSTANT)
    Q_PROPERTY(int      firstGeomId  MEMBER firstGeomId  CONSTANT)
    Q_PROPERTY(QString  firstGeomName MEMBER firstGeomName CONSTANT)
    Q_PROPERTY(int      firstGeomType MEMBER firstGeomType CONSTANT)
    Q_PROPERTY(QString  firstGeomTypeName MEMBER firstGeomTypeName CONSTANT)
    Q_PROPERTY(QVector3D firstGeomSize MEMBER firstGeomSize CONSTANT)
    Q_PROPERTY(QVector4D firstGeomRgba MEMBER firstGeomRgba CONSTANT)
    Q_PROPERTY(int      firstGeomContype MEMBER firstGeomContype CONSTANT)
    Q_PROPERTY(int      firstGeomConaffinity MEMBER firstGeomConaffinity CONSTANT)
public:
    int      bodyId       = -1;
    QString  name;
    int      parentBodyId = -1;
    QString  parentName;
    QVector3D position;
    QVector3D localPosition;
    QQuaternion orientation;
    QQuaternion localOrientation;
    double   mass       = 0.0;
    int      jointCount = 0;
    int      geomCount  = 0;
    bool     movable    = false;
    int      firstGeomId = -1;
    QString  firstGeomName;
    int      firstGeomType = -1;
    QString  firstGeomTypeName;
    QVector3D firstGeomSize;
    QVector4D firstGeomRgba {1.0f, 1.0f, 1.0f, 1.0f};
    int       firstGeomContype    = 0;
    int       firstGeomConaffinity = 0;
};
Q_DECLARE_METATYPE(SceneObjectInfo)

// ---------------------------------------------------------------------------
// BodyMeshData — 某个 body 的三角网格快照，用于外部碰撞库（如 coal）做
// 精确网格碰撞检测。顶点位于 body 局部坐标系（已合并该 body 所有 mesh geom，
// 并按各 geom 的 geom_pos/geom_quat 变换到 body 帧），单位为米。
//
// 字段说明：
//   vertices  — body 局部坐标系下的顶点列表。
//   indices   — 三角面索引，长度为 3 * 三角形数，每 3 个为一个三角形。
//   valid     — 是否成功提取到至少一个三角网格 geom。
// ---------------------------------------------------------------------------
struct BodyMeshData {
    Q_GADGET
    Q_PROPERTY(int vertexCount READ vertexCount)
    Q_PROPERTY(int triangleCount READ triangleCount)
    Q_PROPERTY(bool valid MEMBER valid)
public:
    QVector<QVector3D> vertices;
    QVector<int>       indices;
    bool               valid = false;

    int vertexCount() const { return vertices.size(); }
    int triangleCount() const { return indices.size() / 3; }
};
Q_DECLARE_METATYPE(BodyMeshData)

// ---------------------------------------------------------------------------
// ContactInfo — 单次接触（contact）的快照数据。支持 Q_GADGET，可在 QML 中读取。
//
// 由 MujocoQuickItem::contact(int) / contacts() 返回，每帧在物理锁内采样。
//
// 字段说明：
//   geom0Id / geom1Id   — 两个碰撞 geom 的 ID（geom[0]/geom[1]），-1 表示无效。
//   body0Id / body1Id   — 对应 body 的 ID；geom<0 时为 -1。
//   geom0Name / geom1Name — geom 名称；无名时用 "#<id>"。
//   body0Name / body1Name — body 名称；无名时用 "#<id>"。
//   dist                — 接触距离（< 0 表示穿透）。
//   active              — 是否为有效接触：exclude==0 且 efc_address>=0。
//   penetrating         — dist < 0。
//   normalForce         — 法向力大小（由 mj_contactForce 计算，单位 N）。
//   position            — 接触点在世界坐标系中的位置。
//   normal              — 接触法向量（从 geom1 指向 geom0，由接触帧首行给出）。
// ---------------------------------------------------------------------------
struct ContactInfo {
    Q_GADGET
    Q_PROPERTY(int      geom0Id     MEMBER geom0Id     CONSTANT)
    Q_PROPERTY(int      geom1Id     MEMBER geom1Id     CONSTANT)
    Q_PROPERTY(int      body0Id     MEMBER body0Id     CONSTANT)
    Q_PROPERTY(int      body1Id     MEMBER body1Id     CONSTANT)
    Q_PROPERTY(QString  geom0Name   MEMBER geom0Name   CONSTANT)
    Q_PROPERTY(QString  geom1Name   MEMBER geom1Name   CONSTANT)
    Q_PROPERTY(QString  body0Name   MEMBER body0Name   CONSTANT)
    Q_PROPERTY(QString  body1Name   MEMBER body1Name   CONSTANT)
    Q_PROPERTY(double   dist        MEMBER dist        CONSTANT)
    Q_PROPERTY(bool     active      MEMBER active      CONSTANT)
    Q_PROPERTY(bool     penetrating MEMBER penetrating CONSTANT)
    Q_PROPERTY(double   normalForce MEMBER normalForce CONSTANT)
    Q_PROPERTY(QVector3D position   MEMBER position    CONSTANT)
    Q_PROPERTY(QVector3D normal     MEMBER normal      CONSTANT)
public:
    int      geom0Id     = -1;
    int      geom1Id     = -1;
    int      body0Id     = -1;
    int      body1Id     = -1;
    QString  geom0Name;
    QString  geom1Name;
    QString  body0Name;
    QString  body1Name;
    double   dist        = 0.0;   // 接触距离（< 0 = 穿透）
    bool     active      = false; // exclude==0 && efc_address>=0
    bool     penetrating = false; // dist < 0
    double   normalForce = 0.0;   // 法向力大小（N）
    QVector3D position;           // 接触点世界坐标
    QVector3D normal;             // 接触法向量（接触帧首行）
};
Q_DECLARE_METATYPE(ContactInfo)
