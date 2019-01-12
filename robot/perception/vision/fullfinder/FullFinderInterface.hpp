#ifndef PERCEPTION_VISION_FULLFINDER_FULLFINDERINTERFACE_H_
#define PERCEPTION_VISION_FULLFINDER_FULLFINDERINTERFACE_H_

#include <vector>
#include "types/VisionInfoOut.hpp"
#include "perception/vision/Region.hpp"
#include "types/VisionInfoIn.hpp"

class FullFinder {
public:
    /**
     * find abstract function intended for implementation in a subclass
     */
    virtual void find(const VisionInfoIn& info_in, const RegionI& region, VisionInfoOut& info_out) = 0;
    
};

#endif
