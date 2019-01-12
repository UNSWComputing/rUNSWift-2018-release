#ifndef ROBOT_PERCEPTION_VISION_VISIONDEBUGGERINTERFACE_HPP
#define ROBOT_PERCEPTION_VISION_VISIONDEBUGGERINTERFACE_HPP

#include <stdint.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "Region.hpp"

struct VisionDebugQuery {
    // The index of the region the user wants drawn
    unsigned int region_index;
    // The index of a non-blackboard region the user wants drawn.
    // It's up the the developer how to interpret this.
    unsigned int subregion_index;
    // The settings the user provided for each added option
    // mapping from `option_name` to the value given
    std::map<std::string, std::string> options;
    std::map<std::string, double> numeric_options;
};

// Just some general data that you can write to to use throughout your debugging
struct VisionDebugBlackboard {
    std::map<std::string, int> values;
};

class VisionPainter {
    public:
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

    // Fill in the pixel at coordinates x, y with the given colour
    virtual void draw(int x, int y, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void draw(int x, int y, Colour c) = 0;

    // Draw a line between the two given points with the given colour
    virtual void drawLine(int x1, int y1, int x2, int y2, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void drawLine(int x1, int y1, int x2, int y2, Colour c) = 0;

    // Draw a rectangle starting at the given x, y coordinates, of the given width and height, in the given colour
    virtual void drawRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void drawRect(int x, int y, int w, int h, Colour c) = 0;

    // Draw a circle at the given x, y coordinates with the given radius, in the given colour
    virtual void drawCircle(int x, int y, int radius, uint8_t r, uint8_t g, uint8_t b) = 0; 
    virtual void drawCircle(int x, int y, int radius, Colour c) = 0; 

    void drawColourSaliency(const RegionI &region) {
        for (RegionI::iterator_fovea it = region.begin_fovea(); it != region.end_fovea(); ++it) {
            Colour c = PINK;
            switch (it.colour()) {
                case cGREEN: c = GREEN; break;
                case cWHITE: c = WHITE; break;
                case cBLACK: c = BLACK; break;
                case cBACKGROUND: c = PURPLE; break;
                case cBODY_PART: c = LIME; break;
                default: c = PINK;
            }

            draw(it.x(), it.y(), c);
        }
    }
};

class VisionDebugModule {
    public:
    // The user-set values for each of these options will be returned in the query from
    // the getQuery() method.

    // Add a boolean option (checkbox) with the given name
    virtual void addOption(std::string option_name) = 0;
    // Add a multi-choice option (dropdown) with the given name and values
    virtual void addOption(std::string option_name, std::vector<std::string> options) = 0;
    // Add a numeric-choice with the given name
    virtual void addNumericOption(std::string option_name) = 0;

    // virtual std::string getFilename() = 0;

    // virtual int getCurrentFrame() = 0;

    // Return the options and region indexes
    virtual VisionDebugQuery getQuery() = 0;

    // Set a message to display in vatnao
    // Alternatively use:
    // 
    //     msg << "Something like this";
    //     setDebugMessage()
    //
    virtual void setDebugMessage(std::string msg) = 0;
    std::ostringstream msg;
    void setDebugMessage() {
        setDebugMessage(msg.str());
        msg.str("");
    }

    // Return a painter for drawing over the given region
    virtual VisionPainter* getGivenRegionOverlayPainter(const RegionI& region) = 0;

    // Return a painter for drawing over either the top or bottom camera image
    virtual VisionPainter* getFrameOverlayPainter(int density, bool top) = 0;

    // Space to keep values while you do your debugging
    VisionDebugBlackboard vision_debug_blackboard;
};

#endif
