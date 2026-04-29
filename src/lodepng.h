// ---------------------------------------------------------------------------
// lodepng.h —— 最小 Shim
//
// MuJoCo 官方 simulate.cc 仅在「保存截图」时调用：
//   lodepng::encode(path, rgb, w, h, LCT_RGB)
// 我们没有把官方的 lodepng 第三方库一起编进来，这里用 Qt 的 QImage
// 写一个签名兼容的实现作替身，以满足链接器需求。返回 0 表示成功，
// 与原 lodepng 约定一致。
// ---------------------------------------------------------------------------
#pragma once
#include <string>
#include <QImage>

// LodePNG 颜色类型枚举（C 风格，全局可见，与官方 lodepng 一致）
enum LodePNGColorType {
    LCT_GREY       = 0,
    LCT_RGB        = 2,
    LCT_PALETTE    = 3,
    LCT_GREY_ALPHA = 4,
    LCT_RGBA       = 6,
};

namespace lodepng {

inline unsigned encode(const std::string &filename,
                       const unsigned char *image,
                       unsigned w, unsigned h,
                       LodePNGColorType colortype = LCT_RGB,
                       unsigned /*bitdepth*/ = 8)
{
    if (!image || !w || !h) return 1;

    QImage::Format fmt = QImage::Format_RGB888;
    int bpp = 3;
    switch (colortype) {
    case LCT_RGB:        fmt = QImage::Format_RGB888;     bpp = 3; break;
    case LCT_RGBA:       fmt = QImage::Format_RGBA8888;   bpp = 4; break;
    case LCT_GREY:       fmt = QImage::Format_Grayscale8; bpp = 1; break;
    default:             fmt = QImage::Format_RGB888;     bpp = 3; break;
    }

    QImage img(image, int(w), int(h), int(w) * bpp, fmt);
    return img.copy().save(QString::fromStdString(filename), "PNG") ? 0 : 1;
}

} // namespace lodepng
