#pragma once

#include <iostream>
#include <string>
#include <stdint.h>
#include <vector>

#include "../../../robot/perception/vision/Region.hpp"

using namespace std;

struct FrameRect {
    string label;
    bool topCamera;

    int x;
    int y;
    int width;
    int height;

    void print() {
        std::cout << label << ": "
                  << " top: " << topCamera
                  << " point: ("<< x << ", " << y << ")"
                  << " width: " << width
                  << " height: " << height << std::endl;
    }
};

struct FrameLine {
    string label;
    bool topCamera;

    int x1;
    int y1;
    int x2;
    int y2;

    void print() {
        std::cout << label << ": "
                  << " top: " << topCamera
                  << " pointA: ("<< x1 << ", " << y1 << ")"
                  << " pointB: ("<< x2 << ", " << y2 << ")" << std::endl;
    }
};

struct FramePoint{
    string label;
    bool topCamera;

    int x;
    int y;

    void print() {
        std::cout << label << ": "
                  << " top: " << topCamera
                  << " top: " << topCamera
                  << " point: ("<< x << ", " << y << ")" << std::endl;
    }
};

struct VatnaoFrameInfo {
    uint8_t const* topFrame;
    uint8_t const* botFrame;
    std::vector<FrameRect> regions;
    std::vector<FrameRect> balls;
    std::vector<FrameLine> fieldBoundaries;
    std::vector<FrameLine> fieldLines;
    std::vector<FramePoint> fieldLinePoints;
    std::vector<FramePoint> fieldPoints;
    std::vector<FrameRect> goalPosts;
    std::vector<FrameRect> robots;

    std::vector<RegionI> regionData;
    CameraToRR cameraToRR;

    void print() {
        for (vector<FrameRect>::iterator it = regions.begin(); it != regions.end(); ++it) { it->print(); }
        cout << endl;
        for (vector<FrameRect>::iterator it = balls.begin(); it != balls.end(); ++it) { it->print(); }
        cout << endl;
        for (vector<FrameLine>::iterator it = fieldBoundaries.begin(); it != fieldBoundaries.end(); ++it) { it->print(); }
        cout << endl;
        for (vector<FrameLine>::iterator it = fieldLines.begin(); it != fieldLines.end(); ++it) { it->print(); }
        cout << endl;
        //for (vector<FramePoint>::iterator it = fieldLinePoints.begin(); it != fieldLinePoints.end(); ++it) { it->print(); }
        //cout << endl;
        for (vector<FramePoint>::iterator it = fieldPoints.begin(); it != fieldPoints.end(); ++it) { it->print(); }
        cout << endl;
        for (vector<FrameRect>::iterator it = goalPosts.begin(); it != goalPosts.end(); ++it) { it->print(); }
        for (vector<FrameRect>::iterator it = robots.begin(); it != robots.end(); ++it) { it->print(); }
    }
};
