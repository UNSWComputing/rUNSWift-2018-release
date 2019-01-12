#include "readImage.hpp"

#include <QRgb>

#include "../../../robot/perception/vision/VisionDefinitions.hpp"
#include "../../../robot/perception/vision/other/YUV.hpp"

QRgb RgbImgRGBtoQRgb(RgbImg::RGB c) {
    return qRgb(c.r, c.g, c.b);
}

void RgbImgToQImage(RgbImg* img, QImage *out) {
    int const ROWS = img->getRows();
    int const COLS = img->getCols();

    for (int row = 0; row < ROWS; row++){
        for (int col = 0; col < COLS; col++){
            out->setPixel(col, row, RgbImgRGBtoQRgb(img->getPixel(col, row)));
        }
    }
}

void frameToQImage(VatnaoFrameInfo frameInfo, bool top, QImage *image){
    RgbImg* img = top ? frameInfo.topFrame : frameInfo.botFrame;
    RgbImgToQImage(img, image);
}
