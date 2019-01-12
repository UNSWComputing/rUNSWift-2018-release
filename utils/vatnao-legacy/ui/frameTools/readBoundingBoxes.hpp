#include <QImage>
#include <QColor>
#include "../../app/vatnaoFrameInfo.hpp"

#define COLOUR_BALL        QColor(255,   0,   0)
#define COLOUR_NAO         QColor(  0,   0, 255)
#define COLOUR_LINE        QColor(255,  50, 255)
#define COLOUR_FIELD_POINT QColor(200, 100, 255)
#define COLOUR_BOUNDARY    QColor(  0, 255, 255)
#define COLOUR_GOALS       QColor(255,   0, 255)
#define COLOUR_ROI         QColor(255, 255,   0)
#define HIGHLIGHT_PINK     QColor(255, 105, 180)

#define THICK_LINE 5
#define THIN_LINE  2

void drawRectangle(int x, int y, int width, int height, QColor colour, int lineThickness, QImage *image);
void drawRegionBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawBallBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldBoundaries(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldLines(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldPoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldLinePoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawGoalPosts(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawRobots(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
