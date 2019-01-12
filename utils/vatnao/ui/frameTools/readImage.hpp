#include <QtGui/QApplication>
#include <QImage>
#include "../../app/vatnaoFrameInfo.hpp"

void RgbImgToQImage(RgbImg* img, QImage *out);
void frameToQImage(VatnaoFrameInfo frameInfo, bool top, QImage *image);
