#include "readBoundingBoxes.hpp"

#include <vector>
#include <QPainter>
#include <QPen>

void drawRectangle(int x, int y, int width, int height, QColor colour, int lineThickness, QImage *image){
    QPainter painter(image);
    QPen pen(colour);
    pen.setWidth(lineThickness);
    painter.setPen(pen);

    painter.drawRect(x, y, width, height);
}

void drawLine(int x1, int y1, int x2, int y2, QColor colour, QImage *image){
    QPainter painter(image);
    QPen pen(colour);
    pen.setWidth(5);
    painter.setPen(pen);

    painter.drawLine(x1, y1, x2, y2);
}

void drawPoint(int x, int y, QColor colour, QImage *image){
    QPainter painter(image);
    QPen pen(colour);
    pen.setWidth(5);
    painter.setPen(pen);

    painter.drawEllipse(x, y, 10, 10);
}

void drawRectangles(std::vector<FrameRect> rects, QColor colour, int lineThickness, QImage *topImage, QImage *botImage) {
    for (std::vector<FrameRect>::iterator it = rects.begin(); it != rects.end(); ++it) {
        FrameRect r = *it;
        QImage *img = (r.topCamera) ? topImage : botImage;

        drawRectangle(r.x, r.y, r.width, r.height, colour, lineThickness, img);
    }
}

void drawLines(std::vector<FrameLine> lines, QColor colour, QImage *topImage, QImage *botImage) {
    for (std::vector<FrameLine>::iterator it = lines.begin(); it != lines.end(); ++it) {
        FrameLine l = *it;
        QImage *img = (l.topCamera) ? topImage : botImage;

        drawLine(l.x1, l.y1, l.x2, l.y2, colour, img);
    }
}

void drawPoints(std::vector<FramePoint> points, QColor colour, QImage *topImage, QImage *botImage) {
    for (std::vector<FramePoint>::iterator it = points.begin(); it != points.end(); ++it) {
        FramePoint f = *it;
        QImage *img = (f.topCamera) ? topImage : botImage;

        drawPoint(f.x, f.y, colour, img);
    }
}

void drawRegionBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawRectangles(frameInfo.regions, COLOUR_ROI, THIN_LINE, topImage, botImage);
}

void drawBallBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawRectangles(frameInfo.balls, COLOUR_BALL, THICK_LINE, topImage, botImage);
}

void drawFieldBoundaries(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawLines(frameInfo.fieldBoundaries, COLOUR_BOUNDARY, topImage, botImage);
}

void drawFieldLines(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawLines(frameInfo.fieldLines, COLOUR_LINE, topImage, botImage);
}

void drawFieldPoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawPoints(frameInfo.fieldPoints, COLOUR_FIELD_POINT, topImage, botImage);
}

void drawFieldLinePoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawPoints(frameInfo.fieldLinePoints, COLOUR_LINE, topImage, botImage);
}

void drawGoalPosts(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawRectangles(frameInfo.goalPosts, COLOUR_GOALS, THICK_LINE, topImage, botImage);
}

void drawRobots(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage) {
    drawRectangles(frameInfo.robots, COLOUR_NAO, THICK_LINE, topImage, botImage);
}
