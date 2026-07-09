#ifndef ASSIMPLOADER_H
#define ASSIMPLOADER_H

// ---------------------------------------------------------------------------
// 通用 3D 模型 / 点云顶点提取器。
//
// 直接使用 Assimp 读取模型（.stl .obj .ply .fbx .dae .3ds .gltf 等），
// 只提取顶点位置（以及模型自带的顶点颜色）。相比之前经 coal::internal::Loader
// 的做法，直接用 Assimp 有两个好处：
//   1. coal 的 Loader/buildMesh 依赖三角面，纯点云（PLY 只有 vertex、无 face）
//      会直接抛 "No meshes remaining" / "Invalid face index"，这里不再有此问题。
//   2. 不需要再维护单独的 plyloader.h 兜底。
//
// PCD（PCL 点云格式）不是 Assimp 支持的格式，因此本文件内置了一个自包含的
// PCD 读取器（支持 ascii / binary / binary_compressed 三种 DATA 模式）。
//
// 需要在工程中直接链接 Assimp（头文件 + 导入库），不再依赖 coal 内部的 Assimp。
// ---------------------------------------------------------------------------

#include <QString>
#include <QVector>
#include <QByteArray>
#include <QFile>
#include <QDebug>
#include <QRegularExpression>

#include <cstring>
#include <cstdint>

#include <coal/BVH/BVH_model.h>   // 仅为 coal::Vec3s（scale 参数类型，保持接口兼容）

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// 输出结构（与旧 PlyCloud / 旧 MeshCloud 兼容）
struct MeshCloud {
    QVector<float> positions;   //  xyz 扁平，长度 = count * 3
    QVector<float> colors;      // rgba[0.0-1.0] 假如模型文件本身带颜色就填充颜色,没有则由调用方自己填充
    int            count   = 0;
    bool           hasColor = false; // 模型是否带颜色
};

namespace assimploader_detail {

// -------------------------------------------------------------------------
// LZF 解压（liblzf，public domain）——用于 PCD 的 "binary_compressed"。
// 返回实际写入的字节数；失败返回 0。
// -------------------------------------------------------------------------
inline unsigned lzfDecompress(const void* inData, unsigned inLen,
                              void* outData, unsigned outLen)
{
    const unsigned char* ip = static_cast<const unsigned char*>(inData);
    unsigned char*       op = static_cast<unsigned char*>(outData);
    const unsigned char* const inEnd  = ip + inLen;
    unsigned char*       const outEnd = op + outLen;
    unsigned char*       const outBeg = op;

    while (ip < inEnd) {
        unsigned int ctrl = *ip++;
        if (ctrl < (1 << 5)) {                 // 字面量串
            ctrl++;
            if (op + ctrl > outEnd) return 0;
            do { *op++ = *ip++; } while (--ctrl);
        } else {                               // 回溯引用
            unsigned int len = ctrl >> 5;
            unsigned char* ref = op - ((ctrl & 0x1f) << 8) - 1;
            if (len == 7) len += *ip++;
            ref -= *ip++;
            if (op + len + 2 > outEnd) return 0;
            if (ref < outBeg)          return 0;
            *op++ = *ref++;
            *op++ = *ref++;
            do { *op++ = *ref++; } while (--len);
        }
    }
    return static_cast<unsigned>(op - outBeg);
}

struct PcdField {
    QString name;
    int     size  = 4;    // 每元素字节数
    char    type  = 'F';  // 'F' 浮点 / 'I' 有符号整型 / 'U' 无符号整型
    int     count = 1;    // 分量个数
};

// 从裸内存按 (size,type) 读取一个数值为 double。
inline double pcdReadValue(const char* p, int size, char type)
{
    if (type == 'F') {
        if (size == 4) { float  v; std::memcpy(&v, p, 4); return v; }
        if (size == 8) { double v; std::memcpy(&v, p, 8); return v; }
    } else if (type == 'U') {
        if (size == 1) return static_cast<double>(*reinterpret_cast<const std::uint8_t*>(p));
        if (size == 2) { std::uint16_t v; std::memcpy(&v, p, 2); return v; }
        if (size == 4) { std::uint32_t v; std::memcpy(&v, p, 4); return v; }
        if (size == 8) { std::uint64_t v; std::memcpy(&v, p, 8); return static_cast<double>(v); }
    } else { // 'I'
        if (size == 1) return static_cast<double>(*reinterpret_cast<const std::int8_t*>(p));
        if (size == 2) { std::int16_t v; std::memcpy(&v, p, 2); return v; }
        if (size == 4) { std::int32_t v; std::memcpy(&v, p, 4); return v; }
        if (size == 8) { std::int64_t v; std::memcpy(&v, p, 8); return static_cast<double>(v); }
    }
    return 0.0;
}

// 将 PCL 打包的 rgb/rgba 值（float 位模式或 uint32）拆成 0~1 的 r/g/b/a。
inline void unpackPackedColor(double raw, char type, bool hasAlpha,
                              float& r, float& g, float& b, float& a)
{
    std::uint32_t rgb;
    if (type == 'F') { float f = static_cast<float>(raw); std::memcpy(&rgb, &f, 4); }
    else             { rgb = static_cast<std::uint32_t>(raw); }
    r = ((rgb >> 16) & 0xff) / 255.0f;
    g = ((rgb >>  8) & 0xff) / 255.0f;
    b = ( rgb        & 0xff) / 255.0f;
    a = hasAlpha ? (((rgb >> 24) & 0xff) / 255.0f) : 1.0f;
}

// -------------------------------------------------------------------------
// loadPcd — 读取 PCD 点云（ascii / binary / binary_compressed）。
// -------------------------------------------------------------------------
inline bool loadPcd(const QString& path, MeshCloud* out, QString* err,
                    const coal::Vec3s& scale)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) *err = QStringLiteral("cannot open %1").arg(path);
        return false;
    }
    const QByteArray all = f.readAll();
    f.close();

    QStringList fieldNames;
    QVector<int> sizes;
    QStringList types;
    QVector<int> countList;
    long width = 0, height = 0, pointsHdr = -1;
    QString dataMode;
    int dataStart = -1;

    // 逐行解析头部，直到 DATA 行。
    int pos = 0;
    while (pos < all.size()) {
        int nl = all.indexOf('\n', pos);
        if (nl < 0) nl = all.size();
        const QString line = QString::fromLatin1(all.mid(pos, nl - pos)).simplified();
        pos = nl + 1;
        if (line.isEmpty() || line.startsWith('#')) continue;

        const QStringList tok = line.split(' ', Qt::SkipEmptyParts);
        const QString key = tok[0].toUpper();
        if (key == "FIELDS" || key == "COLUMNS") {
            for (int i = 1; i < tok.size(); ++i) fieldNames << tok[i];
        } else if (key == "SIZE") {
            for (int i = 1; i < tok.size(); ++i) sizes << tok[i].toInt();
        } else if (key == "TYPE") {
            for (int i = 1; i < tok.size(); ++i) types << tok[i];
        } else if (key == "COUNT") {
            for (int i = 1; i < tok.size(); ++i) countList << tok[i].toInt();
        } else if (key == "WIDTH") {
            if (tok.size() >= 2) width = tok[1].toLong();
        } else if (key == "HEIGHT") {
            if (tok.size() >= 2) height = tok[1].toLong();
        } else if (key == "POINTS") {
            if (tok.size() >= 2) pointsHdr = tok[1].toLong();
        } else if (key == "DATA") {
            dataMode  = tok.size() >= 2 ? tok[1].toLower() : QStringLiteral("ascii");
            dataStart = pos;
            break;
        }
    }

    if (fieldNames.isEmpty() || dataStart < 0) {
        if (err) *err = QStringLiteral("invalid PCD header: %1").arg(path);
        return false;
    }

    const int nf = fieldNames.size();
    QVector<PcdField> fields(nf);
    for (int i = 0; i < nf; ++i) {
        fields[i].name  = fieldNames[i];
        fields[i].size  = (i < sizes.size())     ? sizes[i]              : 4;
        fields[i].type  = (i < types.size() && !types[i].isEmpty()) ? types[i][0].toLatin1() : 'F';
        fields[i].count = (i < countList.size()) ? countList[i]          : 1;
        if (fields[i].count <= 0) fields[i].count = 1;
    }

    long pointCount = pointsHdr > 0 ? pointsHdr : (width * (height > 0 ? height : 1));
    if (pointCount <= 0) {
        if (err) *err = QStringLiteral("PCD has zero points: %1").arg(path);
        return false;
    }

    // 定位 x/y/z 与颜色字段。
    int ix = -1, iy = -1, iz = -1, iColor = -1, ir = -1, ig = -1, ib = -1, ia = -1;
    for (int i = 0; i < nf; ++i) {
        const QString n = fields[i].name.toLower();
        if      (n == "x") ix = i;
        else if (n == "y") iy = i;
        else if (n == "z") iz = i;
        else if (n == "rgb" || n == "rgba") iColor = i;
        else if (n == "r" || n == "red")   ir = i;
        else if (n == "g" || n == "green") ig = i;
        else if (n == "b" || n == "blue")  ib = i;
        else if (n == "a" || n == "alpha") ia = i;
    }
    if (ix < 0 || iy < 0 || iz < 0) {
        if (err) *err = QStringLiteral("PCD missing x/y/z fields: %1").arg(path);
        return false;
    }
    const bool hasPacked   = (iColor >= 0);
    const bool hasSeparate = (ir >= 0 && ig >= 0 && ib >= 0);
    const bool hasColor    = hasPacked || hasSeparate;

    // 各字段在一条记录内的字节偏移 / ASCII 分量偏移。
    QVector<int> byteOff(nf, 0), compOff(nf, 0);
    int stride = 0, compTotal = 0;
    for (int i = 0; i < nf; ++i) {
        byteOff[i] = stride;
        compOff[i] = compTotal;
        stride    += fields[i].size * fields[i].count;
        compTotal += fields[i].count;
    }

    out->positions.clear();
    out->colors.clear();
    out->positions.reserve(pointCount * 3);
    if (hasColor) out->colors.reserve(pointCount * 4);

    // 统一的元素取值：给定点 i、字段 f，返回其内存指针。
    // AoS（ascii/binary）与 SoA（binary_compressed）布局分别处理。
    QByteArray decompressed;          // binary_compressed 用
    const char* binBase = nullptr;    // 二进制数据起点
    bool soa = false;
    QVector<int> soaOff(nf, 0);       // SoA 下各字段块起始偏移

    const bool ascii = (dataMode == "ascii");
    if (!ascii) {
        if (dataMode == "binary") {
            binBase = all.constData() + dataStart;
            if (all.size() - dataStart < static_cast<long>(stride) * pointCount) {
                if (err) *err = QStringLiteral("PCD binary data truncated: %1").arg(path);
                return false;
            }
        } else if (dataMode == "binary_compressed") {
            // 头 8 字节：compressedSize, uncompressedSize（uint32 小端）。
            if (all.size() - dataStart < 8) {
                if (err) *err = QStringLiteral("PCD binary_compressed header truncated: %1").arg(path);
                return false;
            }
            std::uint32_t compSize = 0, uncompSize = 0;
            std::memcpy(&compSize,   all.constData() + dataStart,     4);
            std::memcpy(&uncompSize, all.constData() + dataStart + 4, 4);
            if (all.size() - dataStart - 8 < static_cast<long>(compSize)) {
                if (err) *err = QStringLiteral("PCD compressed payload truncated: %1").arg(path);
                return false;
            }
            decompressed.resize(uncompSize);
            const unsigned got = lzfDecompress(all.constData() + dataStart + 8, compSize,
                                               decompressed.data(), uncompSize);
            if (got != uncompSize) {
                if (err) *err = QStringLiteral("PCD LZF decompress failed: %1").arg(path);
                return false;
            }
            binBase = decompressed.constData();
            soa = true;
            int acc = 0;
            for (int i = 0; i < nf; ++i) {
                soaOff[i] = acc;
                acc += fields[i].size * fields[i].count * static_cast<int>(pointCount);
            }
        } else {
            if (err) *err = QStringLiteral("unsupported PCD DATA mode '%1': %2").arg(dataMode, path);
            return false;
        }
    }

    auto binPtr = [&](long i, int fieldIdx) -> const char* {
        if (soa) return binBase + soaOff[fieldIdx] + i * (fields[fieldIdx].size * fields[fieldIdx].count);
        return binBase + i * stride + byteOff[fieldIdx];
    };

    if (ascii) {
        const QByteArray body = all.mid(dataStart);
        const QList<QByteArray> rows = body.split('\n');
        long got = 0;
        for (const QByteArray& row : rows) {
            if (got >= pointCount) break;
            const QByteArray t = row.trimmed();
            if (t.isEmpty()) continue;
            const QList<QByteArray> f2 = t.split(' ');
            QVector<double> vals;
            vals.reserve(f2.size());
            for (const QByteArray& s : f2)
                if (!s.isEmpty()) vals.push_back(s.toDouble());
            if (vals.size() < compTotal) continue;

            const double x = vals[compOff[ix]];
            const double y = vals[compOff[iy]];
            const double z = vals[compOff[iz]];
            out->positions << static_cast<float>(x * scale[0])
                           << static_cast<float>(y * scale[1])
                           << static_cast<float>(z * scale[2]);
            if (hasColor) {
                float r = 1, g = 1, b = 1, a = 1;
                if (hasPacked) {
                    unpackPackedColor(vals[compOff[iColor]], fields[iColor].type,
                                      fields[iColor].name.compare(QLatin1String("rgba"), Qt::CaseInsensitive) == 0,
                                      r, g, b, a);
                } else {
                    r = static_cast<float>(vals[compOff[ir]] / (fields[ir].type == 'F' ? 1.0 : 255.0));
                    g = static_cast<float>(vals[compOff[ig]] / (fields[ig].type == 'F' ? 1.0 : 255.0));
                    b = static_cast<float>(vals[compOff[ib]] / (fields[ib].type == 'F' ? 1.0 : 255.0));
                    if (ia >= 0) a = static_cast<float>(vals[compOff[ia]] / (fields[ia].type == 'F' ? 1.0 : 255.0));
                }
                out->colors << r << g << b << a;
            }
            ++got;
        }
        out->count = static_cast<int>(got);
    } else {
        for (long i = 0; i < pointCount; ++i) {
            const double x = pcdReadValue(binPtr(i, ix), fields[ix].size, fields[ix].type);
            const double y = pcdReadValue(binPtr(i, iy), fields[iy].size, fields[iy].type);
            const double z = pcdReadValue(binPtr(i, iz), fields[iz].size, fields[iz].type);
            out->positions << static_cast<float>(x * scale[0])
                           << static_cast<float>(y * scale[1])
                           << static_cast<float>(z * scale[2]);
            if (hasColor) {
                float r = 1, g = 1, b = 1, a = 1;
                if (hasPacked) {
                    const double raw = pcdReadValue(binPtr(i, iColor), fields[iColor].size, fields[iColor].type);
                    unpackPackedColor(raw, fields[iColor].type,
                                      fields[iColor].name.compare(QLatin1String("rgba"), Qt::CaseInsensitive) == 0,
                                      r, g, b, a);
                } else {
                    const double rr = pcdReadValue(binPtr(i, ir), fields[ir].size, fields[ir].type);
                    const double gg = pcdReadValue(binPtr(i, ig), fields[ig].size, fields[ig].type);
                    const double bb = pcdReadValue(binPtr(i, ib), fields[ib].size, fields[ib].type);
                    r = static_cast<float>(rr / (fields[ir].type == 'F' ? 1.0 : 255.0));
                    g = static_cast<float>(gg / (fields[ig].type == 'F' ? 1.0 : 255.0));
                    b = static_cast<float>(bb / (fields[ib].type == 'F' ? 1.0 : 255.0));
                    if (ia >= 0) {
                        const double aa = pcdReadValue(binPtr(i, ia), fields[ia].size, fields[ia].type);
                        a = static_cast<float>(aa / (fields[ia].type == 'F' ? 1.0 : 255.0));
                    }
                }
                out->colors << r << g << b << a;
            }
        }
        out->count = static_cast<int>(pointCount);
    }

    out->hasColor = hasColor && !out->colors.isEmpty();
    return out->count > 0;
}

// -------------------------------------------------------------------------
// loadPlyFallback — PLY 回退解析器，处理纯点云 PLY（只有 vertex、无 face）。
// Assimp 无法处理这种文件（"all meshes are orphaned"），因此直接读取。
// 支持 ascii / binary_little_endian / binary_big_endian 格式。
// -------------------------------------------------------------------------
inline bool loadPlyFallback(const QString& path, MeshCloud* out, QString* err,
                            const coal::Vec3s& scale)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) *err = QStringLiteral("cannot open %1").arg(path);
        return false;
    }

    // ── 第一遍：逐字节扫描 header，定位 end_header 行尾（二进制数据起点）──
    const QByteArray all = f.readAll();
    f.close();

    // 查找 "end_header" 标记（不区分大小写）
    int ehPos = -1;
    for (int i = 0; i <= all.size() - 10; ++i) {
        if (qstrnicmp(all.constData() + i, "end_header", 10) == 0) {
            ehPos = i;
            break;
        }
    }
    if (ehPos < 0) {
        if (err) *err = QStringLiteral("PLY: no end_header found in %1").arg(path);
        return false;
    }

    // 找到 end_header 后的第一个换行符，其后的字节就是数据起点
    int dataStart = all.indexOf('\n', ehPos + 10);
    if (dataStart < 0) {
        if (err) *err = QStringLiteral("PLY: end_header not followed by newline in %1").arg(path);
        return false;
    }
    ++dataStart; // 跳过换行符

    // ── 解析 header 文本 ──
    const QByteArray headerBytes = all.left(ehPos);
    const QString header = QString::fromLatin1(headerBytes);
    const QStringList lines = header.split('\n');

    enum PlyFmt { PlyAscii, PlyBinLE, PlyBinBE };
    PlyFmt format = PlyAscii;
    bool formatSet = false;

    struct PlyProp {
        QString name;
        int     byteSize = 4;
        int     offset   = 0;
        bool    isFloat  = true;   // true=float/double, false=integer/uchar
    };

    QString curElement;
    long vertexCount = -1;
    QVector<PlyProp> vertexProps;
    int vertexStride = 0;

    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) continue;
        QStringList tok = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (tok.isEmpty()) continue;

        const QString key = tok[0].toLower();

        if (key == "ply") {
            continue;
        } else if (key == "format") {
            if (tok.size() >= 2) {
                const QString fname = tok[1].toLower();
                if (fname == "ascii")            format = PlyAscii;
                else if (fname == "binary_little_endian") format = PlyBinLE;
                else if (fname == "binary_big_endian")   format = PlyBinBE;
                else {
                    if (err) *err = QStringLiteral("PLY: unsupported format '%1' in %2")
                                        .arg(tok[1], path);
                    return false;
                }
                formatSet = true;
            }
        } else if (key == "comment") {
            continue;
        } else if (key == "element") {
            curElement = (tok.size() >= 2) ? tok[1].toLower() : QString();
            long count = (tok.size() >= 3) ? tok[2].toLong() : 0;
            if (curElement == "vertex") {
                vertexCount = count;
                vertexProps.clear();
                vertexStride = 0;
            }
        } else if (key == "property") {
            if (tok.size() < 3) continue;

            // 跳过 list 类型属性（如 property list uchar int vertex_indices）
            if (tok[1].compare("list", Qt::CaseInsensitive) == 0) continue;

            PlyProp prop;
            prop.name = tok[2];

            // 类型映射
            const QString ptype = tok[1].toLower();
            if (ptype == "char" || ptype == "int8")       { prop.byteSize = 1; prop.isFloat = false; }
            else if (ptype == "uchar" || ptype == "uint8") { prop.byteSize = 1; prop.isFloat = false; }
            else if (ptype == "short" || ptype == "int16")  { prop.byteSize = 2; prop.isFloat = false; }
            else if (ptype == "ushort" || ptype == "uint16"){ prop.byteSize = 2; prop.isFloat = false; }
            else if (ptype == "int" || ptype == "int32")    { prop.byteSize = 4; prop.isFloat = false; }
            else if (ptype == "uint" || ptype == "uint32")  { prop.byteSize = 4; prop.isFloat = false; }
            else if (ptype == "float" || ptype == "float32") { prop.byteSize = 4; prop.isFloat = true; }
            else if (ptype == "double" || ptype == "float64"){ prop.byteSize = 8; prop.isFloat = true; }
            else {
                // 未知类型，跳过该 property（不参与 stride 计算）
                continue;
            }

            if (curElement == "vertex") {
                prop.offset = vertexStride;
                vertexStride += prop.byteSize;
                vertexProps.append(prop);
            }
        }
    }

    if (!formatSet) {
        if (err) *err = QStringLiteral("PLY: no format line in %1").arg(path);
        return false;
    }
    if (vertexCount <= 0 || vertexProps.isEmpty()) {
        if (err) *err = QStringLiteral("PLY: no vertices in %1").arg(path);
        return false;
    }

    // ── 定位 x/y/z 和颜色属性 ──
    int ix = -1, iy = -1, iz = -1;
    int ir = -1, ig = -1, ib = -1, ia = -1;
    for (int i = 0; i < vertexProps.size(); ++i) {
        const QString n = vertexProps[i].name.toLower();
        if      (n == "x") ix = i;
        else if (n == "y") iy = i;
        else if (n == "z") iz = i;
        else if (n == "red"   || n == "r") ir = i;
        else if (n == "green" || n == "g") ig = i;
        else if (n == "blue"  || n == "b") ib = i;
        else if (n == "alpha" || n == "a") ia = i;
    }
    if (ix < 0 || iy < 0 || iz < 0) {
        if (err) *err = QStringLiteral("PLY: missing x/y/z properties in %1").arg(path);
        return false;
    }
    const bool hasColor = (ir >= 0 && ig >= 0 && ib >= 0);

    // ── 读取数据 ──
    out->positions.clear();
    out->colors.clear();
    out->positions.reserve(static_cast<int>(vertexCount) * 3);
    if (hasColor) out->colors.reserve(static_cast<int>(vertexCount) * 4);

    if (format == PlyAscii) {
        // ASCII 逐行读取
        const QByteArray body = all.mid(dataStart);
        const QList<QByteArray> dataLines = body.split('\n');
        long got = 0;
        for (const QByteArray& dl : dataLines) {
            if (got >= vertexCount) break;
            const QByteArray t = dl.trimmed();
            if (t.isEmpty()) continue;
            const QList<QByteArray> parts = t.split(' ');
            QVector<double> vals;
            vals.reserve(parts.size());
            for (const QByteArray& s : parts)
                if (!s.isEmpty()) vals.push_back(s.toDouble());
            if (vals.size() < vertexProps.size()) continue;

            out->positions << static_cast<float>(vals[ix] * scale[0])
                           << static_cast<float>(vals[iy] * scale[1])
                           << static_cast<float>(vals[iz] * scale[2]);
            if (hasColor) {
                float r = static_cast<float>(vals[ir] / (vertexProps[ir].isFloat ? 1.0 : 255.0));
                float g = static_cast<float>(vals[ig] / (vertexProps[ig].isFloat ? 1.0 : 255.0));
                float b = static_cast<float>(vals[ib] / (vertexProps[ib].isFloat ? 1.0 : 255.0));
                float a = (ia >= 0) ? static_cast<float>(vals[ia] / (vertexProps[ia].isFloat ? 1.0 : 255.0)) : 1.0f;
                out->colors << r << g << b << a;
            }
            ++got;
        }
        out->count = static_cast<int>(got);
    } else {
        // Binary: 直接用指针算术按 stride 逐条读取，避免 QDataStream 大文件兼容问题
        const char* binPtr = all.constData() + dataStart;
        const char* binEnd = all.constData() + all.size();

        for (long i = 0; i < vertexCount; ++i) {
            if (binPtr + vertexStride > binEnd) break;

            auto readVal = [&](int propIdx) -> double {
                const PlyProp& p = vertexProps[propIdx];
                const char* ptr = binPtr + p.offset;
                if (p.byteSize == 1) {
                    return static_cast<double>(static_cast<unsigned char>(*ptr));
                } else if (p.byteSize == 4) {
                    if (p.isFloat) {
                        float v; std::memcpy(&v, ptr, 4); return v;
                    } else {
                        std::int32_t v; std::memcpy(&v, ptr, 4); return v;
                    }
                } else if (p.byteSize == 8) {
                    double v; std::memcpy(&v, ptr, 8); return v;
                } else { // byteSize == 2
                    std::uint16_t v; std::memcpy(&v, ptr, 2); return v;
                }
            };

            out->positions << static_cast<float>(readVal(ix) * scale[0])
                           << static_cast<float>(readVal(iy) * scale[1])
                           << static_cast<float>(readVal(iz) * scale[2]);
            if (hasColor) {
                const float r = static_cast<float>(readVal(ir) / (vertexProps[ir].isFloat ? 1.0 : 255.0));
                const float g = static_cast<float>(readVal(ig) / (vertexProps[ig].isFloat ? 1.0 : 255.0));
                const float b = static_cast<float>(readVal(ib) / (vertexProps[ib].isFloat ? 1.0 : 255.0));
                const float a = (ia >= 0) ? static_cast<float>(readVal(ia) / (vertexProps[ia].isFloat ? 1.0 : 255.0)) : 1.0f;
                out->colors << r << g << b << a;
            }

            binPtr += vertexStride;
        }
        out->count = static_cast<int>(out->positions.size() / 3);
    }

    out->hasColor = hasColor && !out->colors.isEmpty();
    return out->count > 0;
}

} // namespace assimploader_detail

// ---------------------------------------------------------------------------
// loadMesh — 读取任意 Assimp 支持的模型文件（或 PCD 点云）并提取顶点位置。
//
// 参数：
//   path  - 文件路径
//   out   - 输出 MeshCloud；返回 false 时内容未定义
//   err   - 可选错误信息输出
//   scale - 各轴缩放（默认 1,1,1）
//
// 坐标系说明：
//   直接输出模型原始坐标（Assimp 不统一各格式坐标系），
//   由调用方根据模型实际朝向做轴变换。
// ---------------------------------------------------------------------------
inline bool loadMesh(const QString& path, MeshCloud* out, QString* err = nullptr,
                     const coal::Vec3s& scale = coal::Vec3s::Ones())
{
    if (!out) return false;

    // PCD 不是 Assimp 支持的格式，走内置读取器。
    if (path.toLower().endsWith(QStringLiteral(".pcd")))
        return assimploader_detail::loadPcd(path, out, err, scale);

    Assimp::Importer importer;
    // 只需要顶点位置/颜色：
    //   - aiProcess_PreTransformVertices 把节点变换烘焙进顶点（多网格模型坐标才正确）；
    //   - 不使用 aiProcess_JoinIdenticalVertices，避免点云被去重合并；
    //   - 不强制三角化，纯点云（无面）PLY 也能读出顶点。
    const aiScene* scene = importer.ReadFile(path.toStdString(),
                                             aiProcess_PreTransformVertices);
    if (!scene || scene->mNumMeshes == 0) {
        // Assimp 无法处理纯点云 PLY（只有 vertex、无 face，mesh 会变成 orphaned）。
        // 对这种文件，回退到内置 PLY 解析器直接逐行读取顶点。
        if (path.toLower().endsWith(QStringLiteral(".ply"))) {
            return assimploader_detail::loadPlyFallback(path, out, err, scale);
        }
        const char* aiErr = importer.GetErrorString();
        if (err) *err = (aiErr && aiErr[0]) ? QString::fromLatin1(aiErr)
                                            : QStringLiteral("assimp failed to load: %1").arg(path);
        return false;
    }

    long total = 0;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m)
        total += scene->mMeshes[m]->mNumVertices;
    if (total <= 0) {
        if (err) *err = QStringLiteral("no vertices found in: %1").arg(path);
        return false;
    }

    out->positions.clear();
    out->colors.clear();
    out->positions.reserve(total * 3);

    QVector<float> cols;
    cols.reserve(total * 4);
    bool anyColor = false;

    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        const bool meshHasColor = mesh->HasVertexColors(0);
        anyColor = anyColor || meshHasColor;
        for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
            const aiVector3D& v = mesh->mVertices[i];
            out->positions << static_cast<float>(v.x * scale[0])
                           << static_cast<float>(v.y * scale[1])
                           << static_cast<float>(v.z * scale[2]);
            if (meshHasColor) {
                const aiColor4D& c = mesh->mColors[0][i];
                cols << c.r << c.g << c.b << c.a;
            } else {
                cols << 1.0f << 1.0f << 1.0f << 1.0f;
            }
        }
    }

    out->count = static_cast<int>(total);
    if (anyColor) {
        out->colors   = cols;
        out->hasColor = true;
    } else {
        out->colors.clear();
        out->hasColor = false;
    }
    return true;
}

#endif // ASSIMPLOADER_H
