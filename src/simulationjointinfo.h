#pragma once
#include <QString>
#include <QMetaType>

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
