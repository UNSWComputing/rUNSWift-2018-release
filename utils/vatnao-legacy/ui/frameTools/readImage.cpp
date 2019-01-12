#include "readImage.hpp"

#include <QRgb>

#include "readBoundingBoxes.hpp"

// TODO: this is copied from BALLDETECTOR.cpp

#define NORMALISE_PIXEL(val, c) std::min(std::max(((double)val* c) , 0.0), 255.0)

QRgb yuvToQRgb(int y, int u, int v) {
    y -= 16;
    u -= 128;
    v -= 128;

    int r = static_cast<int>((298.082 * y + 0       * u + 408.583 * v) / 256);
    int g = static_cast<int>((298.082 * y - 100.291 * u - 208.120 * v) / 256);
    int b = static_cast<int>((298.082 * y + 516.411 * u + 0       * v) / 256);

    // bound each r, g and b value between 0 and 255
    r = (r < 0) ? 0 : ((r > 255) ? 255 : r);
    g = (g < 0) ? 0 : ((g > 255) ? 255 : g);
    b = (b < 0) ? 0 : ((b > 255) ? 255 : b);

    return qRgb(r, g, b);
}

QRgb getRGB(uint8_t const* frame, int col, int row, int num_cols){
    uint8_t y = gety(frame, row, col, num_cols);
    uint8_t u = getu(frame, row, col, num_cols);
    uint8_t v = getv(frame, row, col, num_cols);

    return yuvToQRgb(y, u, v);
}

void frameToQImage(VatnaoFrameInfo frameInfo, bool top, QImage *image){
    uint8_t const* frame = top ? frameInfo.topFrame : frameInfo.botFrame;

    if (frame != NULL) {
        int const ROWS = top ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
        int const COLS = top ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

        for (int row = 0; row < ROWS; row++){
            for (int col = 0; col < COLS; col++){
                image->setPixel(col, row, getRGB(frame, col, row, COLS));
            }
        }
    } else {
        // if we can't load the frame, fill with red
        image->fill(qRgb(255, 0, 0));
    }
}

QImage* regionToQImage(VatnaoFrameInfo &frameInfo, FrameRect &r){
    QImage* image = new QImage(r.width, r.height, QImage::Format_RGB32);

    uint8_t const* frame = r.topCamera ? frameInfo.topFrame : frameInfo.botFrame;
    int num_cols = r.topCamera ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    if (frame != NULL) {
        for (int row = 0; row < r.height; row++){
            for (int col = 0; col < r.width; col++){
                image->setPixel(col, row, getRGB(frame, r.x + col, r.y + row, num_cols));
            }
        }
    } else {
        // if we can't load the frame, fill with red
        image->fill(qRgb(255, 0, 0));
    }

    return image;
}

QImage* ballROIToSaliencyQImage(VatnaoFrameInfo &frameInfo, BallDetectorVisionBundle &bdvb, SaliencyType t, bool downSampled,
                                bool candidatePoints, bool circleCentres, bool circleFit, 
                                bool regions, bool regionCentres, bool regionTriangles,
                                bool ATIn, QLineEdit *windowSize, QLineEdit *percentage){
    FrameRect r = rectFromBoundingBox(bdvb.region->getBoundingBoxRaw(), bdvb.region->isTopCamera(), "region");
    return regionToSaliencyQImage(frameInfo, r, bdvb, t, downSampled, candidatePoints, circleCentres, 
                                    circleFit, regions, regionCentres, regionTriangles, ATIn, windowSize, percentage);
}

QImage* regionToSaliencyQImage(VatnaoFrameInfo &frameInfo, FrameRect &r, BallDetectorVisionBundle &bdvb, SaliencyType t, bool downSampled,
                               bool candidatePoints, bool circleCentres, bool circleFit, 
                               bool regions, bool regionCentres, bool regionTriangles,
                               bool ATIn, QLineEdit *windowSize, QLineEdit *percentage){
   
    uint8_t const* frame = r.topCamera ? frameInfo.topFrame : frameInfo.botFrame;
    BallDetector bd = BallDetector();

    int image_rows = r.height;
    int image_cols = r.width;
    int density = 1;

    if (downSampled) {
        density = bdvb.region->getDensity();
        image_rows /= density;
        image_cols /= density;
    }

    QImage* image = new QImage(image_cols, image_rows, QImage::Format_RGB32);

    int num_cols = r.topCamera ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    bd.checkPartialRegion(bdvb);
    bd.getAverageBrightness(bdvb);

    // If region is not selected, ATIn replaces adaptivevaluesforcirclefitting
    if (!regions && ATIn){
        bool ok;
        QString win = windowSize->text();
        QString per = percentage->text();
        int winSize = win.toInt(&ok, 10);
        if (ok && winSize > 0) {
            bdvb.window_size = winSize;
        } else {
            bdvb.window_size = DEFAULT_ADAPTIVE_THRESHOLDING_WINDOW_SIZE;
        }
        int perc = per.toInt(&ok, 10);
        if (ok) {
            bdvb.percentage = perc;
        } else {
            bdvb.percentage = DEFAULT_ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT;
        }
    } else {
        bd.calculateAdaptiveValuesForCircleFitting(bdvb);
    }

    bd.preProcessAdaptive(bdvb);
    bd.getCircleCandidatePoints(*bdvb.region, bdvb, false);
    bd.findCircleKenji(bdvb);

    if (regions){

        // If region is selected, ATIn replaces adaptivevaluesforinternalregions
        if (ATIn){
            bool ok;
            QString win = windowSize->text();
            QString per = percentage->text();
            int winSize = win.toInt(&ok, 10);
            if (ok && winSize > 0) {
                bdvb.window_size = winSize;
            }
            int perc = per.toInt(&ok, 10);
            if (ok) {
                bdvb.percentage = perc;
            }
        } else {
            bd.calculateAdaptiveValuesForInternalRegions(bdvb);
        }

        bd.preProcessAdaptive(bdvb);
        bd.processInternalRegions(*bdvb.region, bdvb, bdvb.circle_fit.result_circle, bdvb.internal_regions);                   
        if (bdvb.internal_regions.groups.size() > 2) {
            bd.findRegionTriangles(bdvb, bdvb.internal_regions, bdvb.region_triangle_features);
        }
    }

    switch (t) {
        case RAW:
            for (int row = 0; row < image_rows; row++){
                for (int col = 0; col < image_cols; col++){
                    image->setPixel(col, row, getRGB(frame, r.x + col * density, r.y + row * density, num_cols));
                }
            }
            break;
        case BINARY:
            int adaptiveColour;

            for (int row = 0; row < image_rows; row++){
                for (int col = 0; col < image_cols; col++){
                    adaptiveColour = bdvb.region->getPixelColour(col, row);
                    if (adaptiveColour == 0) {
                        image->setPixel(col, row, QColor("black").rgb());
                    }
                    else {
                        image->setPixel(col, row, QColor("white").rgb());  
                    }
                }
            }
            break;
    }


    // xDif and yDif are offsets to account for region trimming that occurs in processCircleFit()
    // Candidate points and circle are then drawn correctally relative to vatnao frame
    int xDif = (bdvb.region->getBoundingBoxRaw().a.x() - r.x) / density;
    int yDif = (bdvb.region->getBoundingBoxRaw().a.y() - r.y) / density;

    if (candidatePoints) {
        for (std::vector<Point>::iterator it = bdvb.circle_fit_points.begin(); it != bdvb.circle_fit_points.end(); it++) {
            if (downSampled) {
                image->setPixel(it->x() + xDif, it->y() + yDif, QColor("red").rgb());
            }
            else {
                image->setPixel((it->x() * bdvb.region->getDensity()) + xDif, 
                    (it->y() * bdvb.region->getDensity() + yDif), QColor("red").rgb());
            }
        }
    }
    if (circleFit) {
        QColor circle_pen_colour;
        
        if (bdvb.circle_fit.circle_found) {
            circle_pen_colour = QColor("blue");
        }
        else {
            circle_pen_colour = HIGHLIGHT_PINK;
        }
        
        QPainter painter(image);
        QPoint centre;

        if (downSampled) {
            centre.setX(bdvb.circle_fit.result_circle.centre.x() + xDif);
            centre.setY(bdvb.circle_fit.result_circle.centre.y() + yDif);
            painter.setPen(QPen(circle_pen_colour, 1));
            painter.drawEllipse(centre, (int) bdvb.circle_fit.result_circle.radius, (int) bdvb.circle_fit.result_circle.radius);
        }
        else {
            centre.setX((bdvb.circle_fit.result_circle.centre.x() * bdvb.region->getDensity()) + xDif); 
            centre.setY((bdvb.circle_fit.result_circle.centre.y() * bdvb.region->getDensity()) + yDif); 
            painter.setPen(QPen(circle_pen_colour, 1));
            painter.drawEllipse(centre, 
                (int) bdvb.circle_fit.result_circle.radius * bdvb.region->getDensity(), 
                (int) bdvb.circle_fit.result_circle.radius * bdvb.region->getDensity());
        }
    }
    if (circleCentres) {
        for (std::vector<Point>::iterator it = bdvb.circle_center_points.begin(); it != bdvb.circle_center_points.end(); it++) {
            if (downSampled) {
                image->setPixel(it->x() + xDif, it->y() + yDif, QColor("orange").rgb());
            }
            else {
                image->setPixel((it->x() * bdvb.region->getDensity()) + xDif, 
                    (it->y() * bdvb.region->getDensity() + yDif), QColor("orange").rgb());
            }
        }
    }
    if (regions) {

        QColor completely_internal_region_pen_colour = QColor(138, 43, 226);    // Purple
        QColor internal_region_pen_colour = QColor(51, 255, 51);                // Green
        QPainter painter(image);
        
        for (unsigned int i = 0; i < bdvb.internal_regions.groups.size(); i++) {
            if (bdvb.internal_regions.groups[i].completely_internal) {
                painter.setPen(QPen(completely_internal_region_pen_colour, 1));
            }
            else {
                painter.setPen(QPen(internal_region_pen_colour, 1));
            }
            if (downSampled) {
                QPointF topLeft = QPointF(bdvb.internal_regions.groups[i].min_x + xDif, bdvb.internal_regions.groups[i].min_y + yDif);
                QPointF botRight = QPointF(bdvb.internal_regions.groups[i].max_x + xDif, bdvb.internal_regions.groups[i].max_y + yDif);
                QRectF int_region = QRectF(topLeft, botRight);
                painter.drawRect(int_region);                            
            }
            else {
                QPointF topLeft = QPointF((bdvb.internal_regions.groups[i].min_x * bdvb.region->getDensity()) + xDif,
                                            (bdvb.internal_regions.groups[i].min_y * bdvb.region->getDensity()) + yDif);
                QPointF botRight = QPointF((bdvb.internal_regions.groups[i].max_x * bdvb.region->getDensity()) + xDif,
                                            (bdvb.internal_regions.groups[i].max_y * bdvb.region->getDensity()) + yDif);
                QRectF int_region = QRectF(topLeft, botRight);
                painter.drawRect(int_region);                            
            }
        }
    }
    if (regionCentres) {
        for (std::vector<Point>::iterator it = bdvb.region_triangle_features.region_centres.begin(); it != bdvb.region_triangle_features.region_centres.end(); it++) {
            if (downSampled) {
                image->setPixel(it->x() + xDif, it->y() + yDif, QColor("yellow").rgb());
            }
            else {
                image->setPixel((it->x() * bdvb.region->getDensity()) + xDif, 
                    (it->y() * bdvb.region->getDensity() + yDif), QColor("yellow").rgb());
            }
        }
    }
    if (regionTriangles) {
        QPainter painter(image);                    
        QColor triangle_colour = QColor(255, 0, 128);       // Fuchsia - Thanks Jas
        painter.setPen(QPen(triangle_colour, 1));
        for (std::vector<Triangle>::iterator it = bdvb.region_triangle_features.region_triangles.begin(); it != bdvb.region_triangle_features.region_triangles.end(); it++) {
            std::vector<Point>::iterator next_it_v = it->vertices.begin() + 1;
            for (std::vector<Point>::iterator it_v = it->vertices.begin(); it_v != it->vertices.end(); it_v++, next_it_v++) {
                if (downSampled) {
                    if (next_it_v == it->vertices.end()) {
                        next_it_v = it->vertices.begin();
                    }
                    QPoint pointA(it_v->x() + xDif, it_v->y() + yDif);
                    QPoint pointB(next_it_v->x() + xDif, next_it_v->y() + yDif);
                    painter.drawLine(pointA, pointB);
                }
                else {
                    if (next_it_v == it->vertices.end()) {
                        next_it_v = it->vertices.begin();
                    }
                    QPoint pointA((it_v->x() * bdvb.region->getDensity()) + xDif, (it_v->y() * bdvb.region->getDensity()) + yDif);
                    QPoint pointB((next_it_v->x() * bdvb.region->getDensity()) + xDif, (next_it_v->y() * bdvb.region->getDensity()) + yDif);
                    painter.drawLine(pointA, pointB);                                
                }
            }
        }  
    }
    return image;
}