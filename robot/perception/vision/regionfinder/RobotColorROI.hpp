#ifndef ROBOT_ROI_H
#define ROBOT_ROI_H

/*
This is a slightly modified version of the colourROI for usage on robot detection 2018. 
The main modifications are that it may take a variable size region (at the cost of slight efficiency)
and also that it stores the number of white pixels for each region as well. This is a very useful feature
in detection and is expensive to recalculate
*/

#include "perception/vision/regionfinder/RegionFinderInterface.hpp"

// This is how many pixels there should be between each image cut.
#define CUT_SIZE 16

// Groups with this number of pixels or fewer BEFORE merge are ignored.
#define EARLY_IGNORE_THRESHOLD 8

// Groups with this number of pixels or fewer AFTER merge are ignored.
#define LATE_IGNORE_THRESHOLD 20

// The maximum portion of the image that can be taken up by a single region.
#define MAX_REGION_PORTION_TOP 0.03
#define MAX_REGION_PORTION_BOT 0.15

// Ratio thresholds for merging.
#define MERGE_THRESHOLD_1 0.9
#define MERGE_THRESHOLD_2 0.8
#define MERGE_THRESHOLD_3 0.7
#define MERGE_THRESHOLD_4 0.6
#define MERGE_THRESHOLD_5 0.5

// To avoid division the ratio comparison is done by multiplying the number of
// white pixels by a value (below) and comparing that to the size of the new
// bounding box times 10.
#define MERGE_MULT_1 (int)((1.0/MERGE_THRESHOLD_1)*10.0)
#define MERGE_MULT_2 (int)((1.0/MERGE_THRESHOLD_2)*10.0)
#define MERGE_MULT_3 (int)((1.0/MERGE_THRESHOLD_3)*10.0)
#define MERGE_MULT_4 (int)((1.0/MERGE_THRESHOLD_4)*10.0)
#define MERGE_MULT_5 (int)((1.0/MERGE_THRESHOLD_5)*10.0)


// Class for finding the regions of interest
// based on ColorROI

class RobotColorROI : public RegionFinder {
public:
    RobotColorROI();
    void find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);
    
    inline std::vector<int> activatedCounts() {
        return activatedCounts_;
    }

private:

    std::vector<int> activatedCounts_;

    void connectedComponents(const VisionInfoOut& info_out, RegionI& region, std::vector<RegionI>& regions_out);

};

#endif
