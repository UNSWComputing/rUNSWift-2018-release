#pragma once

#include <iostream>
#include <vector>

#include "types/Point.hpp"
#include "perception/vision/Region.hpp"

/*
Stores all the data field features generates on a single region.
*/
class FieldFeatureRegionData
{
public:

    /*
    Constructs a new info struct to store field feature related data on a single
    region.
    */
    FieldFeatureRegionData() : direction(-1), isPenaltyCrossConfidence(0.0f),
            isCurveConfidence(0.0f), isCornerConfidence(0.0f),
            isTConfidence(0.0), isLine(false)  {};

    /*
    Clean up data associated with this FieldFeatureRegionData struct.
    */
    ~FieldFeatureRegionData()
    {
        // Delete any regions created by field features.
        for(unsigned int region=0; region<relatedRegions.size(); ++region)
        {
            delete relatedRegions[region];
        }
    }

    /*
    The direction associated with this region. -1 is no direction, 0 through
    3 is up down, upper right lower left, right left and lower right upper
    left respectively.
    */
    int direction;

    /*
    Whether this region is a penaltycross.
    */
    float isPenaltyCrossConfidence;

    /*
    Whether this region is a curve, as a probability between 0 and 1.
    */
    float isCurveConfidence;

    /*
    Whether this region is a corner, as a probability between 0 and 1.
    */
    float isCornerConfidence;

    /*
    Whether this region is a T intersection, as a probability between 0 and 1.
    */
    float isTConfidence;

    /*
    The border relative positions of the ends of the lines associated with this
    region.
    */
    std::vector<int> regionEndsBorder;

    /*
    The (X,Y) of the start and end pixels of a region end
    */
    std::vector<std::pair<Point, Point> > regionEndXYPairs;

    /*
    The border ID of the start and end pixels of a region end
    */
    std::vector<std::pair<int, int> > borderEndPairs;

    /*
    The region relative positions of the ends of the lines associated with this
    region.
    */
    std::vector<Point, Eigen::aligned_allocator<Point> > regionEnds;

    /*
    The field positions of the ends of lines found in the region.
    */
    std::vector<Point, Eigen::aligned_allocator<Point> > fieldEnds;

    /*
    The number of white pixels associated with each line end point in a region.
    */
    std::vector<int> regionEndSizes;

    /*
    Intersection, if detected as T or corner
    */
    Point intersection;

    /*
    Whether this region is a line.
    */
    bool isLine;

    /*
    If this region is a line, these are its true ends.
    */
    std::pair<Point, Point> lineEnds;

    /*
    Spreaded Region
    */
    RegionI* spreadedRegion;

    /*
    Spreaded Region with padding added
    */
    RegionI* spreadedRegionWithPadding;

    /*
    Neighbor regions that have been combined IDs;
    */
    std::vector<unsigned int> neighborRegionIDs;

    /*
    Zoomed and/or rethresholded regions associated with this region.
    */
    std::vector<RegionI*> relatedRegions;
};
