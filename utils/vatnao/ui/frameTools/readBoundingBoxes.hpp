#include <QImage>
#include <QColor>
#include "../../app/vatnaoFrameInfo.hpp"

#define COLOUR_BALL        QColor(255,   0,   0)
#define COLOUR_NAO         QColor(255, 255,   0)
#define COLOUR_LINE        QColor(255,  50, 255)
#define COLOUR_FIELD_POINT QColor(200, 100, 255)
#define COLOUR_BOUNDARY    QColor(  0, 255, 255)
#define COLOUR_GOALS       QColor(255,   0, 255)
#define COLOUR_ROI         QColor(  0,   0, 255)
#define COLOUR_SELECTED    QColor(255, 124, 50)

#define THICK_LINE 5
#define MID_LINE   3
#define THIN_LINE  2

void drawRegionBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawHighlightedBoundingBox(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage, unsigned int index);
void drawBallBoundingBoxes(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldBoundaries(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldLines(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldPoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawFieldLinePoints(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawGoalPosts(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
void drawRobots(VatnaoFrameInfo frameInfo, QImage *topImage, QImage *botImage);
