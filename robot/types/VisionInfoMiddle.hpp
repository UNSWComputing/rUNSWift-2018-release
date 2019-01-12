#ifndef VISION_INFO_MIDDLE_HPP
#define VISION_INFO_MIDDLE_HPP

#include <vector>
#include "perception/vision/Region.hpp"
#include "perception/vision/colour/ColourClassifierInterface.hpp"
#include "types/CombinedFrame.hpp"
#include "types/FieldFeatureRegionData.hpp"

struct VisionInfoMiddle
{
    // Regions covering the full frame.
    std::vector<RegionI> full_regions;

    // Regions covering only key areas of interest in the frame.
    std::vector<RegionI> roi;

    // Data created by field features associated with each ROI.
    std::vector<FieldFeatureRegionData> fieldFeatureRegionData;

    // Field features data on other features.
 
    // All valid penalty cross
    std::vector<Point> valid_penalty_cross;

    // All valid field lines.
    std::vector<RANSACLine> valid_lines;

    // Buffered squared length of each line.
    std::vector<int> line_lengths;

    // All valid centre circles.
    std::vector<PointF, Eigen::aligned_allocator<PointF> > circle_centres;

    // The quality of each centre circle.
    std::vector<float> circle_qualities;

    // The centre line associated with each centre circle.
    std::vector<RANSACLine> circle_centre_lines;

    // The ID of the highest quality centre circle.
    int best_centre_circle;

    // The number of non line features sent to localisation by field features.
    int features_sent;

    // All intersections between near perpendicular lines.
    std::vector<Point, Eigen::aligned_allocator<Point> > intersections;

    // All potential corner intersections
    std::vector<Point, Eigen::aligned_allocator<Point> > potentialCornerIntersections;

    std::vector<float> potentialCornerIntersectionAngles;

    // All T intersections of potential T junctions
    std::vector<Point, Eigen::aligned_allocator<Point> > potentialTIntersections;

    // All angles of potential T junctions
    std::vector<float> potentialTIntersectionAngles;

    // The IDs of the lines that form each intersection.
    std::vector<std::pair<int, int> > intersection_lines;

    // Whether each intersection appears to be on each of its forming lines.
    std::vector<std::pair<bool, bool> > on_lines;

    // The quality of each intersection.
    std::vector<float> intersection_qualities;

    // Extra quality for an intersection to be marked as a corner.
    std::vector<float> intersection_corner_qualities;

    // Extra quality for an intersection to be marked as a T.
    std::vector<float> intersection_T_qualities;

    // Extra quality for an intersection to be marked as a centre circle 4 way
    // junction.
    std::vector<float> intersection_4_way_qualities;

    // Colour classifier for the top and bottom cameras.
    ColourClassifier* colour_classifier_top_;
	ColourClassifier* colour_classifier_bot_;

    // The raw data associated with this frame.
    const CombinedFrame* this_frame;
};

#endif
