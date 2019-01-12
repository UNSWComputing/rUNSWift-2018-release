#pragma once

#include <QtGui/QApplication>
#include <QColor>
#include <QImage>
#include <QLineEdit>
#include <QPainter>
#include "../../app/vatnaoFrameInfo.hpp"
#include "../../app/generateFrameInfo.hpp"

#include "../../../robot/perception/vision/colour/GreenYUVClassifier.hpp"
#include "../../../robot/perception/vision/detector/BallDetector.hpp"
#include "../../../robot/perception/vision/VisionDefinitions.hpp"
#include "../../../robot/perception/vision/other/YUV.hpp"

//TODO: get this to link to offnao library
//#include "utils/CPlaneColours.hpp"

const QColor CPLANE_COLOURS[] = {
   //Corresponds to enums in robot/perception/vision/VisionDefinitions.hpp
   "green",
   "white",
   "black",
   "#c09", // Background pink purple
   "#cccccc",    // Body
   "orange"
};

enum SaliencyType {
	RAW = 1,
	BINARY = 2,
};

QRgb getRGB(uint8_t const* frame, int col, int row, int num_cols);
void frameToQImage(VatnaoFrameInfo frameInfo, bool top, QImage *image);
QImage* regionToQImage(VatnaoFrameInfo &frameInfo, FrameRect &r);
QImage* ballROIToSaliencyQImage(VatnaoFrameInfo &frameInfo, 
                                BallDetectorVisionBundle &bdvb, SaliencyType t, bool downSampled,
                                bool candidatePoints, bool circleCenters, bool circleFit, 
                                bool regions, bool regionCentres, bool regionTriangles,
                                bool ATIn, QLineEdit *windowSize, QLineEdit *percentage);
QImage* regionToSaliencyQImage(VatnaoFrameInfo &frameInfo, FrameRect &r, 
                                BallDetectorVisionBundle &bdvb, SaliencyType t, bool downSampled,
                                bool candidatePoints, bool circleCenters, bool circleFit, 
                                bool regions, bool regionCentres, bool regionTriangles,
                                bool ATIn, QLineEdit *windowSize, QLineEdit *percentage);