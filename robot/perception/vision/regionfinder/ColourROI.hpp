#ifndef COLOUR_ROI_H_
#define COLOUR_ROI_H_

#include <stdint.h>

#include "perception/vision/Fovea.hpp"
#include "perception/vision/Region.hpp"
#include "perception/vision/regionfinder/RegionFinderInterface.hpp"

#include "types/GroupLinks.hpp"

// The base resolution CCA should be run at.
#define RES_TARGET TOP_IMAGE_COLS*TOP_IMAGE_ROWS

// Finds ROI in the frame based on colour alone.
class ColourROI : public RegionFinder
{

public:

    // Initialise basic ColourROI variables on creation.
    ColourROI() : group_counts_(std::vector<int>(0, 0)),
                    group_low_xs_(std::vector<int>(0, 0)),
                    group_high_xs_(std::vector<int>(0, 0)),
                    group_low_ys_(std::vector<int>(0, 0)),
                                      group_high_ys_(std::vector<int>(0, 0)) {};

    // Finds ROI in the top and bottom images and stores them in
    // frame.regionsOfInterest.
    void find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

private:

    // Finds ROI in just the top or bottom image.
    template<int columns, int rows> void findROIImage_(const RegionI& region,
                        std::vector<RegionI>& regions_out, VisionInfoOut& info_out);

    // The set of links between groups generated during findROIImage. Here to
    // avoid reallocation.
    GroupLinks group_links_;

    // The ID of the group for each pixel.
    uint16_t groups_[RES_TARGET];

    // The number of pixels in each group.
    std::vector<int> group_counts_;

    // The smallest x value in each group.
    std::vector<int> group_low_xs_;

    // The largest x value in each group.
    std::vector<int> group_high_xs_;

    // The smallest y value in each group.
    std::vector<int> group_low_ys_;

    // The largest y value in each group.
    std::vector<int> group_high_ys_;
};

#endif /* end of include guard: COLOUR_ROI_H_ */
