#ifndef PERCEPTION_VISION_YUV_H_
#define PERCEPTION_VISION_YUV_H_

#include <stdint.h>
#include "perception/vision/VisionDefinitions.hpp"

uint8_t* getpixelpair(const uint8_t* yuv, int row, int col,
                      int num_cols = TOP_IMAGE_COLS);

uint8_t gety(const uint8_t* yuv, int row, int col,
             int num_cols = TOP_IMAGE_COLS);

uint8_t getu(const uint8_t* yuv, int row, int col,
             int num_cols = TOP_IMAGE_COLS);

uint8_t getv(const uint8_t* yuv, int row, int col,
             int num_cols = TOP_IMAGE_COLS);

#endif
