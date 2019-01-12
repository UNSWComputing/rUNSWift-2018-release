#include "RobotColorROI.hpp"
#include <vector>
#include <stdint.h>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <string>
#include <ctime>
#include <climits>
#include "perception/vision/VisionDefinitions.hpp"
#include "utils/Timer.hpp"
#include "types/BBox.hpp"
#include "types/GroupLinks.hpp"

// Whether we're optimising.
//#define DEBUG_OPTIMISE
#ifdef DEBUG_OPTIMISE
#include <iostream>
#endif // DEBUG_OPTIMISE




RobotColorROI::RobotColorROI() : activatedCounts_(std::vector<int>()){};

void RobotColorROI::find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
        
    //empty activated counts
    activatedCounts_.clear();
    
    for(unsigned int i = 0; i < info_middle.full_regions.size(); ++i) {
        RegionI& region = info_middle.full_regions[i];

        // no bottom regions
        if (!region.isTopCamera()) continue;
        
        connectedComponents(info_out, region, info_middle.roi);
    }
    return;
}

void RobotColorROI::connectedComponents(const VisionInfoOut& info_out, RegionI& region, std::vector<RegionI>& regions_out) {


    // The set of links between groups generated during findROIImage. Here to
    // avoid reallocation.
    GroupLinks group_links_;

    int rows = region.getRows();
    int cols = region.getCols();

    // The ID of the group for each pixel.
    uint16_t groups_[rows*cols];

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
    // Reset group_links for use.
    group_links_.fullReset();

    // Clear the data vectors (hopefully leaving space allocated).

    // The number of pixels in each group.
    group_counts_.clear();

    // The smallest x value in each group.
    group_low_xs_.clear();

    // The largest x value in each group.
    group_high_xs_.clear();

    // The smallest y value in each group.
    group_low_ys_.clear();

    // The largest y value in each group.
    group_high_ys_.clear();


#ifdef DEBUG_OPTIMISE
    // Timer for optimisation.
    Timer timer;
    timer.restart();
#endif // DEBUG_OPTIMISE

    // Set to USHRT_MAX when there is no neighbour.
    uint16_t top_neighbour = USHRT_MAX;
    uint16_t left_neighbour = USHRT_MAX;
    bool has_neighbour = false;

    // Iterators that move through the region.
    RegionI::iterator_fovea cur_point = region.begin_fovea();

    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;

    // A pointer tracking through the underlying array of groups.
    uint16_t* group = groups_;
    
    int max_region_size;
    if(region.isTopCamera())
        max_region_size = rows*cols*MAX_REGION_PORTION_TOP;
    else
        max_region_size = rows*cols*MAX_REGION_PORTION_BOT;
    
    int fieldBoundary[cols];
    int endPoint = TOP_IMAGE_COLS*region.isTopCamera() +
                                         BOT_IMAGE_COLS*(!region.isTopCamera());
    for(int x_raw=0, x=0; x_raw<endPoint; x_raw+=region.getDensity(), ++x)
    {
        if(region.isTopCamera())
        {
            fieldBoundary[x] = info_out.topStartScanCoords[x_raw] /
                                                            region.getDensity();
        }
        else
        {
            fieldBoundary[x] = (info_out.botStartScanCoords[x_raw]-BOT_IMAGE_ROWS)
                                                          / region.getDensity();
        }
    }

    for(int pixel=0; pixel < cols*rows; ++pixel)
    {
        // If this is a white pixel, group it. TODO: Make this fast.
        if(y > fieldBoundary[x] && cur_point.colour() == cWHITE)
        {
            // Get all neighbours.
            has_neighbour = false;
            if(x % CUT_SIZE != 0 && y > fieldBoundary[x-1] &&
                                               cur_point.colourLeft() == cWHITE)
            {
                left_neighbour = *(group-1);
                has_neighbour = true;
            }
            else
                left_neighbour = USHRT_MAX;
            if(y > fieldBoundary[x]+1 && y % CUT_SIZE != 0 &&
                                              cur_point.colourAbove() == cWHITE)
            {
                top_neighbour = *(group-cols);
                has_neighbour = true;
            }
            else
                top_neighbour = USHRT_MAX;

            // If there are no neighbours create a new label.
            if(!has_neighbour)
            {
                // Try to add the new group. If this fails, the group cap has
                // been hit and CCA should terminate.
                if(!group_links_.newGroup())
                    break;

                *group = group_links_.size()-1;
                group_low_xs_.push_back(x);
                group_high_xs_.push_back(x);
                group_low_ys_.push_back(y);
                group_high_ys_.push_back(y);
                group_counts_.push_back(1);
            }
            // If there is a neighbour build components.
            else
            {
                if(top_neighbour < left_neighbour)
                {
                    // Set the pixel's group.
                    *group = top_neighbour;

                    // Add a parent to the left neighbour if needed.
                    if(left_neighbour != USHRT_MAX)
                    {
                        bool new_link;
                        new_link = group_links_.addLink(left_neighbour,
                                                                 top_neighbour);
                        if(new_link)
                        {
                            push_heap(group_links_.begin(left_neighbour),
                                      group_links_.end(left_neighbour),
                                                           std::greater<int>());
                        }
                    }

                    // Update bounding box.
                    group_counts_[top_neighbour] += 1;
                    if(group_high_xs_[top_neighbour] < x)
                        group_high_xs_[top_neighbour] = x;
                    if(group_high_ys_[top_neighbour] < y)
                        group_high_ys_[top_neighbour] = y;
                }
                else
                {
                    // Set the pixel's group.
                    *group = left_neighbour;

                    // Add a parent to the top neighbour if needed.
                    if(top_neighbour != USHRT_MAX)
                    {
                        bool new_link;
                        new_link = group_links_.addLink(top_neighbour,
                                                                 left_neighbour);
                        if(new_link)
                        {
                            push_heap(group_links_.begin(top_neighbour),
                                      group_links_.end(top_neighbour),
                                                           std::greater<int>());
                        }
                    }

                    // Update bounding box.
                    group_counts_[left_neighbour] += 1;
                    if(group_high_xs_[left_neighbour] < x)
                        group_high_xs_[left_neighbour] = x;
                    if(group_high_ys_[left_neighbour] < y)
                        group_high_ys_[left_neighbour] = y;
                }
            }
        }

        ++cur_point;
        ++x;
        ++group;
        if(x == cols)
        {
            x = 0;
            ++y;
        }
    }

    // Don't need a full second pass as we only need bounding boxes. Combining
    // by grabbing pixel location extremes is sufficient. May be a faster way
    // to implement this.

#ifdef DEBUG_OPTIMISE
    std::cout << "CCA: " << timer.elapsed_us() << "us"<< std::endl;
    std::cout << "Number of groups: " << group_links_.size() << std::endl;
    int white_pix = 0;
    for(int group=0; group<group_links_.size(); ++group)
        white_pix += group_counts_[group];
    int num_groups_combined = 0;
    timer.restart();
#endif // DEBUG_OPTIMISE

    // Find the parent group for each group.
    bool changed = true;
    while(changed)
    {
        changed = false;
        for(int group=0; group<group_links_.size(); group++)
        {
            // Tell all linked groups the lowest linked group.
            for(int owner=1; owner<group_links_.size(group); owner++)
            {
                // Insert in sorted order.
                changed = true;
                if(group_links_.get(group, owner) != group)
                {
                    bool new_link;
                    new_link = group_links_.addLink(group_links_.get(group,
                                            owner), group_links_.get(group, 0));
                    if(new_link)
                    {
                        push_heap(group_links_.begin(
                            group_links_.get(group, owner)),
                            group_links_.end(group_links_.get(group, owner)),
                                                           std::greater<int>());
                    }
                }
            }

            // Delete all but the lowest owner.
            group_links_.clearHigh(group);
        }
    }

    // Apply combinations.
    for(int group=group_links_.size()-1; group>=0; group--)
    {
        int owner = group_links_.get(group, 0);
        if(owner != group)
        {
#ifdef DEBUG_OPTIMISE
            ++num_groups_combined;
#endif // DEBUG_OPTIMISE
            group_counts_[owner] += group_counts_[group];
            group_counts_[group] = 0;
            if(group_low_xs_[group] < group_low_xs_[owner])
                group_low_xs_[owner] = group_low_xs_[group];
            if(group_low_ys_[group] < group_low_ys_[owner])
                group_low_ys_[owner] = group_low_ys_[group];
            if(group_high_xs_[group] > group_high_xs_[owner])
                group_high_xs_[owner] = group_high_xs_[group];
            if(group_high_ys_[group] > group_high_ys_[owner])
                group_high_ys_[owner] = group_high_ys_[group];
        }
    }

#ifdef DEBUG_OPTIMISE
    std::cout << "Group combining: " << timer.elapsed_us() << "us"<< std::endl;
    std::cout << "Num groups: " << group_links_.size()-num_groups_combined <<
                                                                      std::endl;
    timer.restart();
    int num_merges = 0;
#endif // DEBUG_OPTIMISE

    // Merge groups where density remains good.
    changed = true;
    int thresh = MERGE_MULT_1;
    while(changed)
    {
        changed = false;

        // Check through the groups for merging. Groups are implicitly sorted
        // from upper left to bottom right by upper left corner.
        for(int group1=0; group1<group_links_.size(); ++group1)
        {
            if(group_counts_[group1] > EARLY_IGNORE_THRESHOLD)
            {
                int group2=group1+1;
                int group1_width = group_high_xs_[group1]-group_low_xs_[group1];
                int group1_size = group1_width * (group_high_ys_[group1] -
                                                         group_low_ys_[group1]);
                bool continue_check = true;

                // Continue while it is reasonably probable that a below
                // threshold group can be created.
                while(group2 < group_links_.size() && continue_check)
                {
                    if(group_counts_[group2] > EARLY_IGNORE_THRESHOLD)
                    {
                        // If both groups have pixels and density after
                        // combining is good, combine.
                        int xL = std::min(group_low_xs_[group1],
                                                         group_low_xs_[group2]);
                        int xH = std::max(group_high_xs_[group1],
                                                        group_high_xs_[group2]);
                        int yL = std::min(group_low_ys_[group1],
                                                         group_low_ys_[group2]);
                        int yH = std::max(group_high_ys_[group1],
                                                        group_high_ys_[group2]);
                        int size = (xH-xL+1)*(yH-yL+1);

                        // To avoid division the ratio comparison is done by
                        // multiplying the number of white pixels by a value and
                        // comparing that to the size of the new bounding box
                        // times 10 (to allow a decimal place).
                        if((group_counts_[group1]+group_counts_[group2])*thresh
                                            > size*10 && size < max_region_size)
                        {
#ifdef DEBUG_OPTIMISE
                            ++num_merges;
#endif // DEBUG_OPTIMISE
                            changed = true;
                            group_low_xs_[group1] = xL;
                            group_high_xs_[group1] = xH;
                            group_low_ys_[group1] = yL;
                            group_high_ys_[group1] = yH;
                            group_counts_[group1] += group_counts_[group2];
                            group_counts_[group2] = 0;
                            group1_width = group_high_xs_[group1] -
                                                          group_low_xs_[group1];
                            group1_size = group1_width * (group_high_ys_[group1]
                                                       - group_low_ys_[group1]);
                        }
                    }
                    ++group2;

                    // Check if we should continue.
                    int empty_space = group1_width * (group_low_ys_[group2] -
                                                        group_high_ys_[group1]);
                    if(empty_space > 0)
                    {
                        continue_check = group_counts_[group1]*thresh >
                                                   (empty_space+group1_size)*10;
                    }
                }
            }
        }

        // Work from high threshold down to low threshold. Tends to create nicer
        // BBs.
        if(changed == false && thresh < MERGE_MULT_5)
        {
            changed = true;
            switch(thresh)
            {
                case MERGE_MULT_1:
                    thresh = MERGE_MULT_2;
                    break;
                case MERGE_MULT_2:
                    thresh = MERGE_MULT_3;
                    break;
                case MERGE_MULT_3:
                    thresh = MERGE_MULT_4;
                    break;
                case MERGE_MULT_4:
                    thresh = MERGE_MULT_5;
                    break;
            }
        }
    }

#ifdef DEBUG_OPTIMISE
    std::cout << "Density merge: " << timer.elapsed_us() << "us"<< std::endl;
    timer.restart();
    int previous_regions = regions_out.size();
    std::cout << "Num merges: " << num_merges << std::endl;
#endif // DEBUG_OPTIMISE

    // Create ROI from every relevant group.
    // NOTE: contains a number of inactive heuristics, included as we may want
    // them later.
    

    for(int group=0; group<group_links_.size(); group++)
    {
        // Check the group actually has pixels.
        if(group_counts_[group] > LATE_IGNORE_THRESHOLD)
        {
            // The corners of the new ROI.
            Point upper_left;
            Point lower_right;

            // Calculate corners.
            upper_left[0] = group_low_xs_[group];
            upper_left[1] = group_low_ys_[group];
            lower_right[0] = group_high_xs_[group]+1;
            lower_right[1] = group_high_ys_[group]+1;
            
            // Add the white pixel counts
            activatedCounts_.push_back(group_counts_[group]);
            
            // Create a region of interest.
            regions_out.push_back(RegionI(region, BBox(upper_left, lower_right)));
        }
    }

#ifdef DEBUG_OPTIMISE
    std::cout << "ROI creation: " << timer.elapsed_us() << "us" << std::endl;
    std::cout << "ROI created: " << regions_out.size()-previous_regions <<
                                                                      std::endl;
    timer.restart();
#endif // DEBUG_OPTIMISE
    return;    
}
