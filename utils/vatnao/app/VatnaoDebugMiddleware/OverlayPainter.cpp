#include "OverlayPainter.hpp"

OverlayPainter::OverlayPainter(RgbImg& img): img_(img) {}

void OverlayPainter::draw(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    img_.draw(x, y, r, g, b);
}

void OverlayPainter::draw(int x, int y, OverlayPainter::Colour c) {
    img_.draw(x, y, colourConvert(c));
}

void OverlayPainter::drawLine(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b) {
    img_.drawLine(x1, y1, x2, y2, r, g, b);
}

void OverlayPainter::drawLine(int x1, int y1, int x2, int y2, OverlayPainter::Colour c) {
    img_.drawLine(x1, y1, x2, y2, colourConvert(c));
}

void OverlayPainter::drawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b) {
    img_.drawRect(x, y, w, h, r, g, b);
}

void OverlayPainter::drawRect(int x, int y, int w, int h, OverlayPainter::Colour c) {
    img_.drawRect(x, y, w, h, colourConvert(c));
}

void OverlayPainter::drawCircle(int x, int y, int radius, uint8_t r, uint8_t g, uint8_t b) {
    img_.drawCircle(x, y, radius, r, g, b);
}

void OverlayPainter::drawCircle(int x, int y, int radius, OverlayPainter::Colour c) {
    img_.drawCircle(x, y, radius, colourConvert(c));
}
