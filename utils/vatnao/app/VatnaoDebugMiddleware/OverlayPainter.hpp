#ifndef UTILS_VATNAO_APP_VATNAODEBUGMIDDLEWARE_OVERLAYPAINTER_HPP_
#define UTILS_VATNAO_APP_VATNAODEBUGMIDDLEWARE_OVERLAYPAINTER_HPP_

#include "../ImageMethods/RgbImg.hpp"
#include "../../../../robot/perception/vision/VisionDebuggerInterface.hpp"

class OverlayPainter: public VisionPainter {
    public:
    OverlayPainter(RgbImg& img);

    void draw(int x, int y, uint8_t r, uint8_t g, uint8_t b);
    void draw(int x, int y, Colour c);

    void drawLine(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b);
    void drawLine(int x1, int y1, int x2, int y2, Colour c);

    void drawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b);
    void drawRect(int x, int y, int w, int h, Colour c);

    void drawCircle(int x, int y, int radius, uint8_t r, uint8_t g, uint8_t b); 
    void drawCircle(int x, int y, int radius, Colour c); 

    private:
    RgbImg& img_;

    RgbImg::Colour colourConvert(Colour c) {
        switch (c) {
            case RED:    return RgbImg::RED;
            case GREEN:  return RgbImg::GREEN;
            case BLUE:   return RgbImg::BLUE;
            case YELLOW: return RgbImg::YELLOW;
            case ORANGE: return RgbImg::ORANGE;
            case PURPLE: return RgbImg::PURPLE;
            case PINK:   return RgbImg::PINK;
            case CYAN:   return RgbImg::CYAN;
            case LIME:   return RgbImg::LIME;
            case WHITE:  return RgbImg::WHITE;
            case BLACK:  return RgbImg::BLACK;
            case GREY:   return RgbImg::GREY;

            // Try to give some obviously wrong default return
            // This should only happen if someone adds to the
            // VisionPainter::Colour without adding to RgbImg
            default:     return RgbImg::PINK;
        }
    }
};

#endif
