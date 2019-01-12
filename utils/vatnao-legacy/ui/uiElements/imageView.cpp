#include "imageView.hpp"

#include <QObject>
#include <iostream>

// The following is "inspired" by:
//     http://stackoverflow.com/a/22618496/1112586
// And ensures the images that make up the "ImageViews" will remain in their correct
//  aspect ratio

ImageView::ImageView(QWidget *parent) : QLabel(parent) {
    setMinimumSize(1,1);
    setScaledContents(false);
}

void ImageView::setPixmap(const QPixmap &p) {
    // std::cout << "setting that pixmap" << std::endl;
    pixmap = p;
    applyPixmap();
}

int ImageView::heightForWidth(int width) const {
    return pixmap.isNull() ? this->height() : ((qreal)pixmap.height()*width)/pixmap.width();
}

QSize ImageView::sizeHint() const {
    int w = this->width();
    return QSize(w, heightForWidth(w));
}

QPixmap ImageView::scaledPixmap() const
{
    int w = this->width();
    int h = this->height();
    return pixmap.scaled(w, h, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void ImageView::resizeEvent(QResizeEvent * e)
{
    applyPixmap();
}

void ImageView::applyPixmap() {
    if (!pixmap.isNull()) {
        QLabel::setPixmap(scaledPixmap());
    }
}
