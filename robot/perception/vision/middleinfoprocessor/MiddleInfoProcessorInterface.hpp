#ifndef PERCEPTION_VISION_MIDDLE_INFO_PROCESSOR_INTERFACE_H_
#define PERCEPTION_VISION_MIDDLE_INFO_PROCESSOR_INTERFACE_H_

#include "types/VisionInfoIn.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/VisionInfoMiddle.hpp"

class MiddleInfoProcessor {
public:
    /**
     * detect abstract function intended for implementation in a subclass
     */
    virtual void find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) = 0;

};

#endif
