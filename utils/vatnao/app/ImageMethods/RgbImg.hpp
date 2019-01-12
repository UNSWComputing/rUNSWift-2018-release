#ifndef VATNAO_APP_IMAGEMETHODS_RGBIMG_HPP_
#define VATNAO_APP_IMAGEMETHODS_RGBIMG_HPP_

#include <stdint.h>

#include "../../../../robot/perception/vision/VisionDefinitions.hpp"
#include "../../../../robot/perception/vision/Region.hpp"

#define DEFAULT_MAX_ROWS TOP_IMAGE_ROWS
#define DEFAULT_MAX_COLS TOP_IMAGE_COLS

#define NUM_COLOURS 3
#define R_OFFSET 0
#define G_OFFSET 1
#define B_OFFSET 2

class RgbImg {
    public:
    RgbImg(int max_cols = DEFAULT_MAX_COLS, int max_rows = DEFAULT_MAX_ROWS);

    ~RgbImg();

    // Clear the entire image and reset the curr_cols and curr_rows
    void clear();

    // Take a standard runswift YUV image and paint the image with it
    void fromYUV(uint8_t const* frame, bool top);

    // Take a standard runswift region and paint the image with it
    void fromRegion(uint8_t const* frame, const RegionI& region);
    void fromRegion(const RegionI& region);

    // Apply the given image to this one, ignoring transparency
    void apply(RgbImg& img);

    // Pan a blank image with the given dimensions
    void blank(int cols, int rows);

    // Return cols/rows of the current image
    int getCols();
    int getRows();

    void setDensity(int density);

    struct RGB {
        uint8_t r, g, b;
    };

    RGB rgb(uint8_t r, uint8_t g, uint8_t b) {
        RGB retval = {r, g, b};
        return retval;
    }

    enum Colour {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        ORANGE,
        PURPLE,
        PINK,
        CYAN,
        LIME,
        WHITE,
        BLACK,
        GREY
    };

    RGB ColourToRGB(Colour c) {
        switch (c) {
            case RED:    return rgb(255,   0,   0);
            case GREEN:  return rgb(  0, 255,   0);
            case BLUE:   return rgb(  0,   0, 255);
            case YELLOW: return rgb(255, 255,   0);
            case ORANGE: return rgb(255, 165,   0);
            case PURPLE: return rgb(255,   0, 255);
            case PINK:   return rgb(255,  20, 147);
            case CYAN:   return rgb(  0, 255, 255);
            case LIME:   return rgb(128, 255,   0);
            case WHITE:  return rgb(255, 255, 255);
            case BLACK:  return rgb(  0,   0,   0);
            case GREY:   return rgb(150, 150, 150);

            default:     return rgb(  0,   0,   0);
        }
    }

    RGB getPixel(int x, int y);
    bool getTransparency(int x, int y);

    void setTransparency(int x, int y, bool filled);

    void draw(int x, int y, uint8_t r, uint8_t g, uint8_t b);
    void draw(int x, int y, RGB c) {
        draw(x, y, c.r, c.g, c.b);
    }
    void draw(int x, int y, Colour c) {
        draw(x, y, ColourToRGB(c));
    }

    void drawLine(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b);
    void drawLine(int x1, int y1, int x2, int y2, RGB c) {
        drawLine(x1, y1, x2, y2, c.r, c.g, c.b);
    }
    void drawLine(int x1, int y1, int x2, int y2, Colour c) {
        drawLine(x1, y1, x2, y2, ColourToRGB(c));
    }

    void drawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b);
    void drawRect(int x, int y, int w, int h, RGB c) {
        drawRect(x, y, w, h, c.r, c.g, c.b);
    }
    void drawRect(int x, int y, int w, int h, Colour c) {
        drawRect(x, y, w, h, ColourToRGB(c));
    }

    void drawCircle(int x, int y, int radius, uint8_t r, uint8_t g, uint8_t b);
    void drawCircle(int x, int y, int radius, RGB c) {
        drawCircle(x, y, radius, c.r, c.g, c.b);
    }
    void drawCircle(int x, int y, int radius, Colour c) {
        drawCircle(x, y, radius, ColourToRGB(c));
    }

    void fill(uint8_t r, uint8_t g, uint8_t b);
    void fill(RGB c) {
        fill(c.r, c.g, c.b);
    }
    void fill(Colour c) {
        fill(ColourToRGB(c));
    }

    private:
    uint8_t* img_;
    bool* transparency_;

    int max_cols_;
    int max_rows_;

    int curr_cols_;
    int curr_rows_;

    int density_;

    // Sets the size of the current image, if it's too big, just size
    // it to the maximum
    void setSize(int cols, int rows);

    // Edge cases for really boring lines to draw
    void drawVerticalLine(int col, int y1, int y2, uint8_t r, uint8_t g, uint8_t b) {
        if (y1 > y2) {
            int temp = y1;
            y1 = y2;
            y2 = temp;
        }
        for (int i = y1; i <= y2; i++) {
            draw(col, i, r, g, b);
        }
    }
    void drawHorizontalLine(int row, int x1, int x2, uint8_t r, uint8_t g, uint8_t b) {
        if (x1 > x2) {
            int temp = x1;
            x1 = x2;
            x2 = temp;
        }
        for (int i = x1; i <= x2; i++) {
            draw(i, row, r, g, b);
        }
    }

    // Get the index of the r/g/b pixel for a given col and row
    inline int rIdx(int col, int row) {
        return (col * NUM_COLOURS) + (row * NUM_COLOURS * max_cols_) + R_OFFSET;
    }
    inline int gIdx(int col, int row) {
        return (col * NUM_COLOURS) + (row * NUM_COLOURS * max_cols_) + G_OFFSET;
    }
    inline int bIdx(int col, int row) {
        return (col * NUM_COLOURS) + (row * NUM_COLOURS * max_cols_) + B_OFFSET;
    }
    inline int tIdx(int col, int row) {
        return col + row * max_cols_;
    }

};
#endif
