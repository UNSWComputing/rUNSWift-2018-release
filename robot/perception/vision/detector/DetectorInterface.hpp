#ifndef PERCEPTION_VISION_DETECTOR_DETECTORINTERFACE_H_
#define PERCEPTION_VISION_DETECTOR_DETECTORINTERFACE_H_

#include "types/VisionInfoIn.hpp"
#include "types/VisionInfoMiddle.hpp"
#include "types/VisionInfoOut.hpp"

class Detector {
public:
    /**
     * detect abstract function intended for implementation in a subclass
     */
    virtual void detect(const VisionInfoIn& info_in,
                    VisionInfoMiddle& info_middle, VisionInfoOut& info_out) = 0;
};

#endif
