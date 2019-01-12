#ifndef PERCEPTION_VISION_WRITEIMAGE_H_
#define PERCEPTION_VISION_WRITEIMAGE_H_

/**
 * WriteImage.hpp
 * Description: Write images at region density in png format to a given file location
 * Input: Region: RegionI
 * 		  Format: RAW_FORMAT or COLOUR_FORMAT (binary)
 * 		  Filename: Location and filename eg. /home/nao/Pictures/ball.png. Location must exist or will fail.
 * 		  Title to be saved in metadata.
 * Return: 1 if successful, 0 if unsuccessful
 */

#include <iostream>
#include <stdint.h>
#include <png.h>

#include "perception/vision/Region.hpp"

struct RGB {
    uint8_t r, g, b;
};

class WriteImage {
    public:
        int writeImage(const RegionI& region, write_image_format format, const char* filename, char* title);
    private:
        RGB yuvToRGB(int y, int u, int v);
};

#endif
