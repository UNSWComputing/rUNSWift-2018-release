#include "RgbImg.hpp"

#include <math.h>

#include "../../../robot/types/BBox.hpp"
#include "../../../robot/perception/vision/other/YUV.hpp"

RgbImg::RgbImg(int max_cols, int max_rows) {
    img_ = (uint8_t*) malloc(sizeof(uint8_t) * max_cols * max_rows * NUM_COLOURS);
    transparency_ = (bool*) malloc(sizeof(bool) * max_cols * max_rows);
    max_cols_ = max_cols;
    max_rows_ = max_rows;
    curr_cols_ = max_cols;
    curr_rows_ = max_rows;
    density_ = 1;
    clear();
}

RgbImg::~RgbImg() {
    free(img_);
    free(transparency_);
}

void RgbImg::clear() {
    for (int x = 0; x < max_cols_; x++) {
        for (int y = 0; y < max_rows_; y++) {
            draw(x, y, BLACK);
            setTransparency(x, y, false);
        }
    }
    curr_cols_ = max_cols_;
    curr_rows_ = max_rows_;
}

RgbImg::RGB RgbImg::getPixel(int x, int y) {
    RGB retval;
    retval.r = img_[rIdx(x, y)];
    retval.g = img_[gIdx(x, y)];
    retval.b = img_[bIdx(x, y)];

    return retval;
}

bool RgbImg::getTransparency(int x, int y) {
    return transparency_[tIdx(x, y)];
}

void RgbImg::blank(int cols, int rows) {
    clear();
    setSize(cols, rows);
}

void RgbImg::setSize(int cols, int rows) {
    curr_cols_ = std::min(cols, max_cols_);
    curr_rows_ = std::min(rows, max_rows_);
}

int RgbImg::getCols() {
    return curr_cols_;
}

int RgbImg::getRows() {
    return curr_rows_;
}

RgbImg::RGB yuvToRGB(int y, int u, int v) {
    y -= 16;
    u -= 128;
    v -= 128;

    int r = static_cast<int>((298.082 * y + 0       * u + 408.583 * v) / 256);
    int g = static_cast<int>((298.082 * y - 100.291 * u - 208.120 * v) / 256);
    int b = static_cast<int>((298.082 * y + 516.411 * u + 0       * v) / 256);

    // bound each r, g and b value between 0 and 255
    RgbImg::RGB retval;
    retval.r = (r < 0) ? 0 : ((r > 255) ? 255 : r);
    retval.g = (g < 0) ? 0 : ((g > 255) ? 255 : g);
    retval.b = (b < 0) ? 0 : ((b > 255) ? 255 : b);

    return retval;
}

RgbImg::RGB getRgbImgRGB(uint8_t const* frame, int col, int row, int num_cols){
    uint8_t y = gety(frame, row, col, num_cols);
    uint8_t u = getu(frame, row, col, num_cols);
    uint8_t v = getv(frame, row, col, num_cols);

    return yuvToRGB(y, u, v);
}

void RgbImg::fromYUV(uint8_t const* frame, bool top) {
    int const COLS = top ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
    int const ROWS = top ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

    setSize(COLS, ROWS);
    if (frame != NULL) {
        for (int col = 0; col < COLS; col++) {
            for (int row = 0; row < ROWS; row++) {
                draw(col, row, getRgbImgRGB(frame, col, row, COLS));
            }
        }
    } else {
        // if we can't load the frame
        fill(PINK);
    }
}

void RgbImg::fromRegion(uint8_t const* frame, const RegionI& region) {
    int const COLS = region.isTopCamera() ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    BBox bounding_box = region.getBoundingBoxRaw();

    int cols = bounding_box.width();
    int rows = bounding_box.height();
    int x1 = bounding_box.a.x();
    int y1 = bounding_box.a.y();
    int x2 = bounding_box.b.x();
    int y2 = bounding_box.b.y();

    setSize(cols, rows);
    if (frame != NULL) {
        for (int col = x1; col < x2; col++) {
            for (int row = y1; row < y2; row++) {
                draw(col, row, getRgbImgRGB(frame, col, row, COLS));
            }
        }
    } else {
        // if we can't load the frame
        fill(PINK);
    }
}

void RgbImg::fromRegion(const RegionI& region) {
    int const COLS = region.isTopCamera() ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    int cols = region.getCols();
    int rows = region.getRows();

    const uint8_t* raw;

    setSize(cols, rows);
    for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
            raw = region.getPixelRaw(col, row);
            if (raw != NULL) {
                draw(col, row, getRgbImgRGB(raw, col, row, COLS));
            } else {
                // something went wrong retrieving the image, fill with pink and
                // break out
                // If you're here trying to debug what went wrong, it's possible
                // you're calling this on a region without an attached frame, in
                // which case you should call the fromRegion(frame, region) method
                // instead.
                fill(PINK);
                return;
            }
        }
    }
}

void RgbImg::apply(RgbImg& img) {
    int cols = std::min(getCols(), img.getCols());
    int rows = std::min(getRows(), img.getRows());

    for (int col = 0; col < cols; col++) {
        for (int row = 0; row < rows; row++) {
            if (img.getTransparency(col, row)) {
                draw(col, row, img.getPixel(col, row));
            }
        }
    }
}

void RgbImg::setDensity(int density) {
    density_ = density;
}

void RgbImg::setTransparency(int x, int y, bool filled) {
     for (int x_ = x * density_; x_ < x * density_ + density_; x_++) {
        for (int y_ = y * density_; y_ < y * density_ + density_; y_++) {
            if (x_ >= 0 && x_ < max_cols_ && y_ >= 0 && y_ < max_rows_) {
                transparency_[tIdx(x_, y_)] = filled;
            }
        }
    }
}

void RgbImg::draw(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    for (int x_ = x * density_; x_ < x * density_ + density_; x_++) {
        for (int y_ = y * density_; y_ < y * density_ + density_; y_++) {
            // Draw the pixel if we are within the array
            if (x_ >= 0 && x_ < max_cols_ && y_ >= 0 && y_ < max_rows_) {
                img_[rIdx(x_, y_)] = r;
                img_[gIdx(x_, y_)] = g;
                img_[bIdx(x_, y_)] = b;
            }
        }
    }
    setTransparency(x, y, true);
}

void RgbImg::drawLine(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b) {
    // Based on the Bresenham's line algorithm
    // https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
    // http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#C.2B.2B

    bool is_steep = abs(y2 - y1) > abs(x2 - x1);
    if (is_steep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    float error = dx / 2.0f;

    int ystep = y1 < y2 ? 1 : -1;
    int y = y1;

    for (int x = x1; x <= x2; x++) {
        if (is_steep) {
            draw(y, x, r, g, b);
        } else {
            draw(x, y, r, g, b);
        }

        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

void RgbImg::drawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b) {
    drawLine(  x,   y, x+w,   y, r, g, b);
    drawLine(  x,   y,   x, y+h, r, g, b);
    drawLine(x+w,   y, x+w, y+h, r, g, b);
    drawLine(  x, y+h, x+w, y+h, r, g, b);
}

void RgbImg::drawCircle(int x, int y, int radius, uint8_t r, uint8_t g, uint8_t b) {
    // Based on the Bresenham Circle Algorithm (Midpoint circle algorithm)
    // https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    int x_ = radius;
    int y_ = 0;
    int err = 0;

    while (x_ >= y_) {
        // We draw a pixel in each octant of the circle until x and y meet
        draw(x + x_, y + y_, r, g, b);
        draw(x + y_, y + x_, r, g, b);
        draw(x - y_, y + x_, r, g, b);
        draw(x - x_, y + y_, r, g, b);
        draw(x - x_, y - y_, r, g, b);
        draw(x - y_, y - x_, r, g, b);
        draw(x + y_, y - x_, r, g, b);
        draw(x + x_, y - y_, r, g, b);

        y_++;
        if (err <= 0) {
            err += 2*y_ + 1;
        }
        if (err > 0) {
            x_ -= 1;
            err -= 2*x_ + 1;
        }
    }
}

void RgbImg::fill(uint8_t r, uint8_t g, uint8_t b) {
    for (int x = 0; x < max_cols_; x++) {
        for (int y = 0; y < max_rows_; y++) {
            draw(x, y, r, g, b);
        }
    }
}
