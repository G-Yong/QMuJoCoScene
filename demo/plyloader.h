#ifndef PLYLOADER_H
#define PLYLOADER_H

// ---------------------------------------------------------------------------
// 极简 PLY 点云加载器（仅用于 demo）。
// 支持 format ascii 1.0 与 binary_little_endian 1.0；解析 vertex 元素的
// x/y/z（必需）与 red/green/blue 或 r/g/b（可选，uchar 0~255 或 float 0~1）。
// face 等其他元素一律跳过（点云不需要）。
// ---------------------------------------------------------------------------

#include <QByteArray>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QVector>
#include <cstring>

struct PlyCloud {
    QVector<float> positions;   // xyz 扁平
    QVector<float> colors;      // rgba 扁平（0~1）；hasColor=false 时为空
    int            count = 0;
    bool           hasColor = false;
};

namespace plyloader_detail {

inline int typeSize(const QString& t) {
    if (t == "char"  || t == "uchar" || t == "int8" || t == "uint8")   return 1;
    if (t == "short" || t == "ushort"|| t == "int16"|| t == "uint16")  return 2;
    if (t == "int"   || t == "uint"  || t == "int32"|| t == "uint32" ||
        t == "float" || t == "float32")                                return 4;
    if (t == "double"|| t == "float64")                                return 8;
    return 0; // 未知（如 list）
}

inline bool isFloatType(const QString& t) {
    return t == "float" || t == "float32" || t == "double" || t == "float64";
}

// 从二进制缓冲按类型读出一个数值（小端），并返回归一化后的颜色/原始坐标值。
inline double readBinary(const char* p, const QString& t) {
    if (t == "float" || t == "float32") { float v; std::memcpy(&v, p, 4); return v; }
    if (t == "double"|| t == "float64") { double v; std::memcpy(&v, p, 8); return v; }
    if (t == "uchar" || t == "uint8")   { return static_cast<unsigned char>(p[0]); }
    if (t == "char"  || t == "int8")    { return static_cast<signed char>(p[0]); }
    if (t == "ushort"|| t == "uint16")  { quint16 v; std::memcpy(&v, p, 2); return v; }
    if (t == "short" || t == "int16")   { qint16 v;  std::memcpy(&v, p, 2); return v; }
    if (t == "uint"  || t == "uint32")  { quint32 v; std::memcpy(&v, p, 4); return v; }
    if (t == "int"   || t == "int32")   { qint32 v;  std::memcpy(&v, p, 4); return v; }
    return 0.0;
}

} // namespace plyloader_detail

inline bool loadPly(const QString& path, PlyCloud* out, QString* err = nullptr) {
    using namespace plyloader_detail;
    if (!out) return false;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) *err = QStringLiteral("cannot open %1").arg(path);
        return false;
    }
    const QByteArray all = f.readAll();
    f.close();

    // 找到 header 结束位置（"end_header\n"）。
    const QByteArray marker = "end_header";
    int hdrEnd = all.indexOf(marker);
    if (hdrEnd < 0) { if (err) *err = "no end_header"; return false; }
    int dataStart = all.indexOf('\n', hdrEnd);
    if (dataStart < 0) { if (err) *err = "malformed header"; return false; }
    ++dataStart;

    const QString header = QString::fromLatin1(all.left(hdrEnd));
    const QStringList lines = header.split('\n', Qt::SkipEmptyParts);

    bool ascii = true;
    int  vertexCount = 0;
    bool inVertex = false;

    struct Prop { QString name; QString type; };
    QVector<Prop> vprops;

    for (const QString& raw : lines) {
        const QString line = raw.trimmed();
        const QStringList tok = line.split(' ', Qt::SkipEmptyParts);
        if (tok.isEmpty()) continue;
        if (tok[0] == "format" && tok.size() >= 2) {
            ascii = tok[1].startsWith("ascii");
        } else if (tok[0] == "element" && tok.size() >= 3) {
            inVertex = (tok[1] == "vertex");
            if (inVertex) vertexCount = tok[2].toInt();
        } else if (tok[0] == "property" && inVertex) {
            // property <type> <name>  或  property list <a> <b> <name>
            if (tok.size() >= 3 && tok[1] != "list")
                vprops.push_back({tok.last(), tok[1]});
        }
    }
    if (vertexCount <= 0 || vprops.isEmpty()) {
        if (err) *err = "no vertices";
        return false;
    }

    auto findProp = [&](std::initializer_list<const char*> names) -> int {
        for (int i = 0; i < vprops.size(); ++i)
            for (const char* n : names)
                if (vprops[i].name.compare(QLatin1String(n), Qt::CaseInsensitive) == 0)
                    return i;
        return -1;
    };
    const int ix = findProp({"x"});
    const int iy = findProp({"y"});
    const int iz = findProp({"z"});
    const int ir = findProp({"red",   "r", "diffuse_red"});
    const int ig = findProp({"green", "g", "diffuse_green"});
    const int ib = findProp({"blue",  "b", "diffuse_blue"});
    const int ia = findProp({"alpha", "a", "diffuse_alpha"});
    if (ix < 0 || iy < 0 || iz < 0) { if (err) *err = "missing x/y/z"; return false; }
    const bool hasColor = (ir >= 0 && ig >= 0 && ib >= 0);

    out->positions.clear();
    out->colors.clear();
    out->positions.reserve(vertexCount * 3);
    if (hasColor) out->colors.reserve(vertexCount * 4);

    auto colorScale = [&](int idx, double v) -> float {
        return isFloatType(vprops[idx].type) ? static_cast<float>(v)
                                             : static_cast<float>(v / 255.0);
    };

    if (ascii) {
        const QByteArray body = all.mid(dataStart);
        const QList<QByteArray> rows = body.split('\n');
        int got = 0;
        for (const QByteArray& row : rows) {
            if (got >= vertexCount) break;
            const QByteArray t = row.trimmed();
            if (t.isEmpty()) continue;
            const QList<QByteArray> f2 = t.split(' ');
            QVector<double> vals;
            vals.reserve(f2.size());
            for (const QByteArray& s : f2)
                if (!s.isEmpty()) vals.push_back(s.toDouble());
            if (vals.size() < vprops.size()) continue;
            out->positions.push_back(static_cast<float>(vals[ix]));
            out->positions.push_back(static_cast<float>(vals[iy]));
            out->positions.push_back(static_cast<float>(vals[iz]));
            if (hasColor) {
                out->colors.push_back(colorScale(ir, vals[ir]));
                out->colors.push_back(colorScale(ig, vals[ig]));
                out->colors.push_back(colorScale(ib, vals[ib]));
                out->colors.push_back(colorScale(ia, vals[ia]));
            }
            ++got;
        }
        out->count = got;
    } else {
        // binary_little_endian：逐顶点按属性顺序读取。
        QVector<int> off(vprops.size(), 0);
        int stride = 0;
        for (int i = 0; i < vprops.size(); ++i) {
            off[i] = stride;
            const int sz = typeSize(vprops[i].type);
            if (sz == 0) { if (err) *err = "list property in vertex not supported"; return false; }
            stride += sz;
        }
        const char* base = all.constData() + dataStart;
        const int avail = all.size() - dataStart;
        if (avail < stride * vertexCount) { if (err) *err = "binary data truncated"; return false; }
        for (int v = 0; v < vertexCount; ++v) {
            const char* rec = base + static_cast<qsizetype>(v) * stride;
            out->positions.push_back(static_cast<float>(readBinary(rec + off[ix], vprops[ix].type)));
            out->positions.push_back(static_cast<float>(readBinary(rec + off[iy], vprops[iy].type)));
            out->positions.push_back(static_cast<float>(readBinary(rec + off[iz], vprops[iz].type)));
            if (hasColor) {
                out->colors.push_back(colorScale(ir, readBinary(rec + off[ir], vprops[ir].type)));
                out->colors.push_back(colorScale(ig, readBinary(rec + off[ig], vprops[ig].type)));
                out->colors.push_back(colorScale(ib, readBinary(rec + off[ib], vprops[ib].type)));
                out->colors.push_back(colorScale(ia, readBinary(rec + off[ia], vprops[ia].type)));
            }
        }
        out->count = vertexCount;
    }

    out->hasColor = hasColor && !out->colors.isEmpty();
    return out->count > 0;
}

#endif // PLYLOADER_H
