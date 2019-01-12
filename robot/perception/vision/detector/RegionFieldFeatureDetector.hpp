#ifndef PERCEPTION_VISION_DETECTOR_REGIONFIELDFEATUREDETECTOR_H_
#define PERCEPTION_VISION_DETECTOR_REGIONFIELDFEATUREDETECTOR_H_

#include "DetectorInterface.hpp"

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/other/GMM_classifier.hpp"

#include "utils/Timer.hpp"
#include "types/GroupLinks.hpp"

// Whether to make and output time taken for field features components.
//#define FIELD_FEATURE_TIMINGS

#define MAX_RES TOP_IMAGE_COLS*TOP_IMAGE_ROWS

class RegionFieldFeatureDetector : public Detector
{

public:

    /*
    Constructor
    */
    RegionFieldFeatureDetector();

    /*
    Detects field features by making use of the regions produced by the region
    finder.
     */
    void detect(
        const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out);

private:

    /*
    Analyses regions
    */
    void analyseRegions_(
        const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle);

    /*
    Spread seedRegion to neighbors if regionEnds don't change. This can spread for
    SPREAD_ITERATION times.

    SPREAD_ITERATION = 1 : Combine directly overlapping and neighbor regions
    SPREAD_ITERATION = 2 : Combine with neighbor's neighbors
    ...
    */
    RegionI& spreadRegion_(
        const RegionI& seedRegion,
        FieldFeatureRegionData& seedRegionData,
        const unsigned int seedRegionID,
        const std::vector<RegionI>& regions);

    /*
    Get all regionIDs that are neighbors or overlapping
    */
    std::vector<int> findOverlappingRegionIDs_(
        const RegionI& region,
        const std::vector<RegionI>& regions);

    /*
    Combine two regions, return pointer to new region
    */
    RegionI* combineRegions_(
        const RegionI& region1,
        const RegionI& region2);

    /*
    Pad region to ensure the feature is included and not unnecesarily touching the border
    */
    RegionI* padRegion_(
        const RegionI& region,
        FieldFeatureRegionData& data);

    /*
    Analyses region border. Populates regionEndsBorder, regionEnds,
    regionEndSizes, borderEndPairs, regionEndXYPairs.
    */
    void analyseRegionBorder_(
        const RegionI& region,
        std::vector<int>& regionEndsBorder,
        std::vector<Point, Eigen::aligned_allocator<Point> >& regionEnds,
        std::vector<int>& regionEndSizes,
        std::vector<std::pair<int, int> >& borderEndPairs,
        std::vector<std::pair<Point, Point> >& regionEndXYPairs);

    /*
    Construct a vector of booleans indicating whether or not the pixel along
    the border is white or not.
    */
    const std::vector<bool> constructBorder_(
        const RegionI& region,
        const int numCols,
        const int numRows,
        const int borderLength);

    /*
    Find the "ends" of each region. Populates regionEndsBorder, regionEnds, regionEndSizes,
    borderEndPairs, regionEndXYPairs.
    */
    void findRegionEnds_(
        const RegionI& region,
        const int numCols,
        const int numRows,
        const std::vector<bool>& border,
        const int borderLength,
        std::vector<int>& regionEndsBorder,
        std::vector<Point, Eigen::aligned_allocator<Point> >& regionEnds,
        std::vector<int>& regionEndSizes,
        std::vector<std::pair<int, int> >& borderEndPairs,
        std::vector<std::pair<Point, Point> >& regionEndXYPairs);

    /*
    Check if there is internal black region in a petential penalty cross
    */
    void checkInternalNotWhiteRegion(const RegionI& base_region);

    /*
    Determines and labels a region if it is penalty cross
    */
    void determineIfPenaltyCross(const VisionInfoIn& info_in,
            VisionInfoMiddle& info_middle, const RegionI& pad_region,
            const unsigned int regionID, FieldFeatureRegionData& data);

    /*
    Check if all points on segment AB are white, in region
    */
    bool determineIfAllWhite(
                        const RegionI& region, Point& A, Point& B);

    /*
    Determines and labels a region a line, curve, corner or none.
    */
    void determineIfLineCurveCorner_(const VisionInfoIn& info_in,
            VisionInfoMiddle& info_middle, const RegionI& region,
            FieldFeatureRegionData& data);

    /*
    Checks that a region containing two ends really is a line (straight or
    otherwise).
    */
    bool performLineChecks_(const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle, const RegionI& region,
                                                  FieldFeatureRegionData& data);

    /*
    Determine if the field relative sizes of the region ends is signifcantly
    different.
    */
    bool checkRelEndSize_(const VisionInfoIn& info_in, const RegionI& region,
        const std::pair<Point, Point> end1, const std::pair<Point, Point> end2);

    /*
    Checks whether two border points are actually connected by white pixels.
    */
    bool checkConnected_(const RegionI& region, std::pair<Point, Point>& a,
                                                    std::pair<Point, Point>& b);

    /*
    Determines and labels a region if is T Intersection
    */
    void determineIfTIntersection_(const VisionInfoIn& info_in,
                    VisionInfoMiddle& info_middle, const RegionI& region,
                                        FieldFeatureRegionData& data);

    /*
    Project regionEnds to fieldEnds using cameraToRR
    */
    void projectRegionEndsToFieldEnds_(
        const VisionInfoIn& info_in,
        const RegionI& region,
        FieldFeatureRegionData& data);

    /*
    Determines whether the region is a curve or not
    */
    bool determineIfCurve_(
        VisionInfoMiddle& info_middle,
        const RegionI& region,
        FieldFeatureRegionData& data,
        const std::pair<Point, Point>& regionPoints,
        const std::pair<Point, Point>& fieldPoints);

    /*
    Determines whether the centre point offset from the centre of the line
    is great. Returns True if the offset is large, False if not.
    */
    bool centrePointOffsetAnalysis_(
        const RegionI& region,
        const std::pair<Point, Point>& regionPoints);

    /*
    Doing linear regression of border points near startPoint and endPoint
    respectively.
    */
    bool linearAnalysis_(
        const RegionI& region);

    /*
    Find points that lie on the line where colour changes from white to green.
    */
    const std::vector<Point> findLinearAnalysisPoints_(
        const RegionI& region);

    /*
    Check that linear analysis points don't lie on any lines.
    */
    bool checkInTwoParts_(
        const RegionI& region,
        std::vector<Point, Eigen::aligned_allocator<Point> >& linearAnalysisPoints);

    /*
    Check if the two lines forming the corner intersect at a minimum angle to
    form a corner, and all diagonalScanPoints lie on the two lines
    */
    bool cornerShapeCheck(const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle, const RegionI& region,
        const std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints);

    /*
    Check if the two lines forming the corner intersect at a minimum angle to
    form a corner, and all diagonalScanPoints lie on the two lines
    */
    bool cornerShapeCheck(const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle, const RegionI& region,
        const std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints,
                                                                    Point& tip);

    /*
    Determines whether the region is a corner or not
    */
    bool determineIfCorner_(const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle, const RegionI& region,
        FieldFeatureRegionData& data, const std::pair<Point, Point>& regionPoints,
        const std::pair<Point, Point>& fieldPoints);

    /*
    Creates lines and circles from regions with distinct directions.
    */
    void createFeatures_(const VisionInfoIn& info_in,
                        VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

    /*
    Merges line segments found in regions into longer line segments.
    */
    void mergeLineRegions_(VisionInfoMiddle& info_middle,
                                                  std::vector<int>& curFeature);

    /*
    Construct lines
    */
    void constructLines_(VisionInfoMiddle& info_middle);

    /*
    Merge existing lines into longer lines where possible.
    */
    void mergeLines_(VisionInfoMiddle& info_middle, std::vector<bool>& merged,
                                                  std::vector<int>& curFeature);

    /*
    Locates centre circles using region line segments.
    */
    void detectCentreCircles_(VisionInfoMiddle& info_middle,
                   std::vector<int>& curFeature, std::vector<int>& curFeature2);

    /*
    Adjust centre cirlce quality based on the quality of possible centre lines
    through the centre circle.
    */
    void detectCentreCircleLines_(VisionInfoMiddle& info_middle);

    /*
    Determines the best candidate centre circle and adds it as a field feature,
    if it is of sufficient quality.
    */
    void determineBestCentreCircle_(VisionInfoMiddle& info_middle,
                                                       VisionInfoOut& info_out);

    /*
    Clears lines that are merged or too near the centre circle.
    */
    void cleanUpAndSendLines_(VisionInfoMiddle& info_middle,
                            VisionInfoOut& info_out, std::vector<bool>& merged);

    /*
    Locates corners and T intersections using region line segments.
    */
    void detectIntersections_(VisionInfoMiddle& info_middle);

    /*
    Adds quality to each intersection based on the length and proximity of the
    associated lines. Also adds some T intersection quality if the feature is on
    one of the lines.
    */
    void adjustIntersectionQualityWithLines_(VisionInfoMiddle& info_middle);

    /*
    Adds quality to each intersection based on the all types of confidence of the
    regions that contains the intersections
    */
    void adjustIntersectionQualityWithRegions_(VisionInfoMiddle& info_middle);

    /*
    Add corner quality to the other end of lines with good T quality as it might
    be the goal box.
    */
    void goalBoxCheck_(VisionInfoMiddle& info_middle);

    /*
    Sends Ts and coners of high enough quality to localisation.
    TODO: Select the best features to send, rather than just the first ones to
          pass the threshold. Note: usually wont matter.
    */
    void createTsAndCorners_(const VisionInfoIn& info_in,
                        VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

    /*
    Finally create penalty cross from previous detector
    */
    void createPenaltyCross_(const VisionInfoIn& info_in,
                        VisionInfoMiddle& info_middle, VisionInfoOut& info_out);
    /*
    Interpolates between hyperparameter values based on distance.
    */
    inline int interpolateHyperparameters_(const long long distance,
        const long long min, const long long max, const long long min_distance,
                                                  const long long max_distance);

    /*
    Creates a value by interpolating between minThreshold (0) and maxThreshold,
    (1) then multiplying by maxProbability.
    If maxThreshold is greater than minThreshold the result is 0 if
    value <= minThreshold up to maxValue if value >= maxThreshold.
    If maxThreshold is less than minThreshold the result is 0 if
    value >= minThreshold up to maxValue if value <= maxThreshold.
    In either case there is linear interpolation between the two values.
    */
    inline float createValueFromThresholds_(float minThreshold,
        float midThreshold, float maxThreshold, float minValue, float maxValue,
                                                                    float value)
    {
        float result;
        if(fabs(value-maxThreshold) < fabs(value-minThreshold))
        {
            float threshold = maxThreshold - midThreshold;
            result = maxValue * std::max(0.0f, std::min(1.0f,
                                             (value-midThreshold) / threshold));
        }
        else
        {
            float threshold = minThreshold - midThreshold;
            result = minValue * std::max(0.0f, std::min(1.0f,
                                             (value-midThreshold) / threshold));
        }
        return(result);
    };

    // IMPLEMENTATIONS FROM THE OLD SYSTEM. TODO: Replace with nicer methods.

    /*
    Determines the angle of a T intersection in the form required by
    localisation.
    */
    float findTAngle_(Point &p, RANSACLine &l);

    /*
    Determines the angle of a corner in the form required by localisation.
    */
    float findCAngle_(Point &p, RANSACLine &l1, RANSACLine &l2);

    /*
    Determines the gradient of a line relative to a point.
    */
    float findGradient_(RANSACLine &l, Point &p);

    /*
    Creates an instance of FieldFeatureInfo for a centre circle given its centre
    point and the centre line.
    */
    FieldFeatureInfo constructCenterCircleFeature_(PointF circle_centre,
                                                               RANSACLine line);

    /*
    Calculates the angle between the robot and the given field feature.
    */
    float getRobotCentreAngle_(FieldFeatureInfo feature);

    /*
    Calculates the angle of the centre line given a line.
    */
    float getCentreLineAngle_(RANSACLine line);

    /*
    Builds an edge from startIntersection to endIntersection and stores the
    pixels that compose it in edgePixels.
    */
    bool buildEdge_(const RegionI& region, Point lastIntersection,
        Point startIntersection, Point endIntersection,
              std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints);

    // The set of links between groups generated during determine penalty cross.
    GroupLinks group_links_;

    // The ID of the group for each pixel.
    uint16_t groups_[MAX_RES];

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

    // Number of white pixels Connected Component Analysis
    int num_whites;

    // Whether connected component analysis detected body part
    bool has_body_part;

    // GMM ball classifier
    Estimator estimator;

    // The edges of the current region being considered.
    std::vector<Point, Eigen::aligned_allocator<Point> > firstEdge;
    std::vector<Point, Eigen::aligned_allocator<Point> > secondEdge;

#ifdef FIELD_FEATURE_TIMINGS
    // Count to determine when timings should be output.
    int frameCount;

    // The total time spent on region analysis.
    Timer analyseRegionsTimer;
    int analyseRegionsTime;

    // Analyse region components.

    // The time spent in the region spread function.
    Timer spreadRegionTimer;
    int spreadRegionTime;

    // Spread region components.

    // The time spent in on border analysis within spread region.
    Timer spreadRegionAnalyseRegionBorderTimer;
    int spreadRegionAnalyseRegionBorderTime;

    // The time spent on finding overlapping regions.
    Timer findOverlappingRegionIDsTimer;
    int findOverlappingRegionIDsTime;

    // End of spread region components.

    // The time spent in the project ends function.
    Timer projectRegionEndsTimer;
    int projectRegionEndsTime;

    // The time spent in the penalty cross detector.
    Timer penaltyCrossDetectorTimer;
    int penaltyCrossDetectorTime;

    // The time spent in the line curve corner detector.
    Timer lineCurveCornerTimer;
    int lineCurveCornerTime;

    // Line curve corner components.

    // The time spent in the line check function.
    Timer lineCheckTimer;
    int lineCheckTime;

    // The time spent in the curve detector.
    Timer curveDetectorTimer;
    int curveDetectorTime;

    // Curve detector components.

    // The time spent in the center point offset analysis.
    Timer centrePointOffsetAnalysisCurveTimer;
    int centrePointOffsetAnalysisCurveTime;

    // The time spent in the linear analysis.
    Timer linearAnalysisTimer;
    int linearAnalysisTime;

    // End of curve detector components.

    // The time spent in the corner detector.
    Timer cornerDetectorTimer;
    int cornerDetectorTime;

    // Curve detector components.

    // The time spent in the center point offset analysis.
    Timer centrePointOffsetAnalysisCornerTimer;
    int centrePointOffsetAnalysisCornerTime;

    // The time spent in the black white border transition finder functions.
    Timer findBlackWhiteBorderTransitionsTimer;
    int findBlackWhiteBorderTransitionsTime;

    // The time spent in the corner shape check.
    Timer cornerShapeCheckTimer;
    int cornerShapeCheckTime;

    // End of curve detector components.

    // End of line curve corner components.

    // The time spent in the T-intersection detector.
    Timer tIntersectionTimer;
    int tIntersectionTime;

    // End of analyse region components.

    // The total time spent on region composition.
    Timer createFeaturesTimer;
    int createFeaturesTime;

#endif // FIELD_FEATURE_TIMINGS
};

#endif // PERCEPTION_VISION_DETECTOR_REGIONFIELDFEATUREDETECTOR_H_
