#ifndef PERCEPTION_VISION_COLOUR_COLOURCLASSIFIERINTERFACE_H_
#define PERCEPTION_VISION_COLOUR_COLOURCLASSIFIERINTERFACE_H_

#include "perception/vision/VisionDefinitions.hpp"
#include "types/CombinedFrame.hpp"
#include "types/VisionInfoIn.hpp"

class ColourClassifier {
public:

    /**
     * Sample abstract function intended for implementation in a subclass
     * Given a pixel YUV value return its colour
     */
    virtual Colour classifyTop(const uint8_t *pixel) const = 0;
    virtual Colour classifyBot(const uint8_t *pixel) const = 0;

    virtual void sampleImageScanLines(const uint8_t* image, bool top, int n_rows, int n_cols,
        const VisionInfoIn& info_in, int stepsize) = 0;
    virtual void loadNnmc(std::string filename) = 0;
    virtual void updateColours() = 0;
    virtual void saveClassification() = 0;
    virtual void saveNnmc(std::string filename) = 0;
    virtual void resetNnmc() = 0;
    virtual void fillPoints(bool isGreen) = 0;

};

#endif
