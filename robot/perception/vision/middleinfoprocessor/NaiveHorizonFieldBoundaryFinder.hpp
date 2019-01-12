#ifndef PERCEPTION_VISION_MIDDLEINFOPROCESSOR_NAIVEHORIZONFIELDBOUNDARYFINDER
#define PERCEPTION_VISION_MIDDLEINFOPROCESSOR_NAIVEHORIZONFIELDBOUNDARYFINDER

#include "MiddleInfoProcessorInterface.hpp"

class NaiveHorizonFieldBoundaryFinder: public MiddleInfoProcessor {
    public:
		void find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);
    private:
		void detect(const VisionInfoIn& info_in, const RegionI& region, VisionInfoOut& info_out);
        void findStartScanCoords(const RegionI& region, VisionInfoOut& info_out);
};

#endif
