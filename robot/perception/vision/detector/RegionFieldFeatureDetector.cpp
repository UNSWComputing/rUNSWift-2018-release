#include <numeric>

#include "perception/vision/detector/RegionFieldFeatureDetector.hpp"
#include "perception/vision/VisionDefinitions.hpp"

#include "types/FieldFeatureInfo.hpp"
#include "types/RansacTypes.hpp"
#include "types/Point.hpp"
#include "types/IndexSorter.hpp"
#include "types/GroupLinks.hpp"
#include "perception/vision/other/WriteImage.hpp"

#include "utils/SPLDefs.hpp"
#include "utils/speech.hpp"

// The maximum number of features localisation can handle.
#define MAX_LOCALISATION_FEATURES 6

// The minimum length line allowed to be considered a curve.
#define MIN_CURVE_LENGTH (100*100)

// The maximum ratio beteen the sizes of two line ends.
#define MAX_LINE_END_SIZE_RATIO 2.1f

// How wide an area a region may be in to be added to a line.
#define MIN_LINE_CONNECTION_DISTANCE_CLOSE (100)
#define MIN_LINE_CONNECTION_DISTANCE_FAR (200)
#define MIN_LINE_CONNECTION_DISTANCE_DISTANCE_CLOSE (500)
#define MIN_LINE_CONNECTION_DISTANCE_DISTANCE_FAR (5000)
#define MIN_LINE_CONNECTION_DISTANCE_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, MIN_LINE_CONNECTION_DISTANCE_CLOSE, \
    MIN_LINE_CONNECTION_DISTANCE_FAR, \
    MIN_LINE_CONNECTION_DISTANCE_DISTANCE_CLOSE, \
                                      MIN_LINE_CONNECTION_DISTANCE_DISTANCE_FAR)

// Maximum distance between a new and old line end point allowed.
#define MAX_LINE_EXTENSION_DISTANCE_CLOSE (1000*1000)
#define MAX_LINE_EXTENSION_DISTANCE_FAR (2000*2000)
#define MAX_LINE_EXTENSION_DISTANCE_DISTANCE_CLOSE (1000*1000)
#define MAX_LINE_EXTENSION_DISTANCE_DISTANCE_FAR (4000*4000)
#define MAX_LINE_EXTENSION_DISTANCE_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, MAX_LINE_EXTENSION_DISTANCE_CLOSE, \
    MAX_LINE_EXTENSION_DISTANCE_FAR, \
    MAX_LINE_EXTENSION_DISTANCE_DISTANCE_CLOSE, \
                                       MAX_LINE_EXTENSION_DISTANCE_DISTANCE_FAR)

// The minimum allowable line length.
#define MINIMUM_LINE_LENGTH_CLOSE (200*200)
#define MINIMUM_LINE_LENGTH_FAR (200*200)
#define MINIMUM_LINE_LENGTH_CLOSE_DISTANCE (2000*2000)
#define MINIMUM_LINE_LENGTH_FAR_DISTANCE (3000*2000)
#define MINIMUM_LINE_LENGTH_INTERPOLATE(distance) interpolateHyperparameters_( \
    distance, MINIMUM_LINE_LENGTH_CLOSE, MINIMUM_LINE_LENGTH_FAR, \
           MINIMUM_LINE_LENGTH_CLOSE_DISTANCE, MINIMUM_LINE_LENGTH_FAR_DISTANCE)

// The numerator and denominator of the fraction of edges that must be in a
// particular direction for a line to be considered to be in that direction.
#define STRONG_DIRECTION_FRACTION_NUMERATOR 3
#define STRONG_DIRECTION_FRACTION_DENOMINATOR 4

// The numerator and denominator of the ratio of edges between two buckets for
// the strong line direction to be valid.
#define STRONG_DIRECTION_RATIO_FRACTION_NUMERATOR 0
#define STRONG_DIRECTION_RATIO_FRACTION_DENOMINATOR 100

// The maximum error allowed for lines to be considered perpendicular.
#define MAX_PERPENDICULAR_ANGLE_ERROR (M_PI/8)
#define MIN_QUALITY_FROM_PERPENDICULAR_ANGLE_ERROR -100.0f
#define MAX_QUALITY_FROM_PERPENDICULAR_ANGLE_ERROR 1.0f

// The excess distance that must be allowed on either side of a T intersection.
#define T_INTERSECTION_EXCESS (100*100)
#define MIN_QUALITY_FROM_T_INTERSECTION_ON_LINE -100.0f
#define MAX_QUALITY_FROM_T_INTERSECTION_ON_LINE 1.0f

// The maximum error allowed in goal box detection.
#define MAX_GOAL_BOX_ERROR (200)
#define MIN_QUALITY_FROM_GOAL_BOX_PROXIMITY 0.0f
#define MAX_QUALITY_FROM_GOAL_BOX_PROXIMITY 1.0f

// The minimum line length required to form a corner or T.
#define MIN_FEATURE_LINE_LENGTH_CLOSE (1000*1000)
#define MIN_FEATURE_LINE_LENGTH_FAR (1500*1500)
#define MIN_FEATURE_LINE_LENGTH_CLOSE_DISTANCE (1000*1000)
#define MIN_FEATURE_LINE_LENGTH_FAR_DISTANCE (3000*3000)
#define MIN_FEATURE_LINE_LENGTH_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, MIN_FEATURE_LINE_LENGTH_CLOSE, \
    MIN_FEATURE_LINE_LENGTH_FAR, MIN_FEATURE_LINE_LENGTH_CLOSE_DISTANCE, \
                                           MIN_FEATURE_LINE_LENGTH_FAR_DISTANCE)
#define MIN_QUALITY_FROM_LINE_LENGTH_INTERSECTION 0.0f
#define MAX_QUALITY_FROM_LINE_LENGTH_INTERSECTION 1.0f

// The minimum line length required for a goal box corner.
#define MIN_FEATURE_LINE_LENGTH_GOAL_BOX (700*700)

// How far an intersection point can be from the nearest line end.
#define MAX_LINE_END_INTERSECTION_DISTANCE_CLOSE (200*200)
#define MAX_LINE_END_INTERSECTION_DISTANCE_FAR (600*600)
#define MAX_LINE_END_INTERSECTION_DISTANCE_CLOSE_DISTANCE (1000*1000)
#define MAX_LINE_END_INTERSECTION_DISTANCE_FAR_DISTANCE (3000*3000)
#define MAX_LINE_END_INTERSECTION_DISTANCE_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, \
    MAX_LINE_END_INTERSECTION_DISTANCE_CLOSE, \
    MAX_LINE_END_INTERSECTION_DISTANCE_FAR, \
    MAX_LINE_END_INTERSECTION_DISTANCE_CLOSE_DISTANCE, \
                                MAX_LINE_END_INTERSECTION_DISTANCE_FAR_DISTANCE)
#define MIN_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION -100.0f
#define MAX_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION 1.0f

// The multiplier by which regions should be expanded to make sure edges are
// included.
#define REGION_EXPANSION_MULTIPLIER 1.1f

// The minimum quality for a centre circle to be created.
#define MIN_CENTRE_CIRCLE_QUALITY 4.0f

// The portion of a centre circle that must be seen for one to be generated.
#define CENTRE_CIRCLE_PORTION_CLOSE 0.25f
#define CENTRE_CIRCLE_PORTION_FAR 0.35f

// The portion of curve must be detected in all segments of potential circle
#define MIN_CURVE_PORTION_OF_CENTRE_CIRCLE 0.35f
#define MIN_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH -100.0f
#define MAX_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH 1.0f

// The total line length to reach the desired centre circle portion.
#define MIN_CENTRE_CIRCLE_LENGTH_CLOSE ((int)(CENTRE_CIRCLE_PORTION_CLOSE * \
                                          ((float)CENTER_CIRCLE_DIAMETER*M_PI)))
#define MIN_CENTRE_CIRCLE_LENGTH_FAR ((int)(CENTRE_CIRCLE_PORTION_FAR * \
                                          ((float)CENTER_CIRCLE_DIAMETER*M_PI)))
#define MIN_CENTRE_CIRCLE_LENGTH_CLOSE_DISTANCE (500*500)
#define MIN_CENTRE_CIRCLE_LENGTH_FAR_DISTANCE (900*900)
#define MIN_CENTRE_CIRCLE_LENGTH_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, MIN_CENTRE_CIRCLE_LENGTH_CLOSE, \
    MIN_CENTRE_CIRCLE_LENGTH_FAR, MIN_CENTRE_CIRCLE_LENGTH_CLOSE_DISTANCE, \
                                          MIN_CENTRE_CIRCLE_LENGTH_FAR_DISTANCE)
#define MIN_QUALITY_FROM_CENTRE_CIRCLE_LENGTH -100.0f
#define MAX_QUALITY_FROM_CENTRE_CIRCLE_LENGTH 1.0f

// The error in mm allowed for centre circle line ends.
#define CENTRE_CIRCLE_ERROR_CLOSE 300
#define CENTRE_CIRCLE_ERROR_FAR 300
#define CENTRE_CIRCLE_ERROR_CLOSE_DISTANCE (500*500)
#define CENTRE_CIRCLE_ERROR_FAR_DISTANCE (700*700)
#define CENTRE_CIRCLE_ERROR_INTERPOLATE(distance) \
    interpolateHyperparameters_(distance, CENTRE_CIRCLE_ERROR_CLOSE, \
    CENTRE_CIRCLE_ERROR_FAR, CENTRE_CIRCLE_ERROR_CLOSE_DISTANCE, \
                                               CENTRE_CIRCLE_ERROR_FAR_DISTANCE)

// How close a line can pass to the centre of the centre circle before it is
// culled.
#define MIN_CENTRE_CIRCLE_DISTANCE_FROM_CENTRE 600

// How close a line must pass to the centre of a circle to be considered a valid
// centre line.
#define MAX_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CIRCLE_CENTRE 300
#define MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CENTRE_CIRCLE -100.0f
#define MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CENTRE_CIRCLE 1.0f

// The minimum acceptable length of a centre circle centre line.
#define MIN_CENTRE_CIRCLE_LINE_LENGTH (200*200)
#define MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_LENGTH -100.0f
#define MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_LENGTH 1.0f

// The minimum distance a line must be from the centre circle.
#define MIN_DISTANCE_TO_CENTRE_CIRCLE (2000*2000)

// The maximum distance a centre circle line may be from a centre circle.
#define MAX_CENTRE_CIRCLE_LINE_END_DISTANCE (500*500)
#define MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE -100.0f
#define MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE 1.0f

// The overall quality required to form an intersection.
#define INTERSECTION_QUALITY_THRESHOLD 1.0f

// The T intersection specific quality required to test an intersection as a T
// intersection.
#define T_INTERSECTION_QUALITY_THRESHOLD 50.0f

// The maximum T Intersection error
#define MAX_T_INTERSECTION_ERROR (100*100)

// The maximum Corner Intersection error
#define MAX_CORNER_INTERSECTION_ERROR (100*100)

// The corner specific quality required to test an intersection as a corner.
#define CORNER_QUALITY_THRESHOLD 50.0f

// Any regions whose ratio of width and height is larger than 4, cannot be
// corner
#define MAX_CORNER_WIDTH_HEIGHT_RATIO 5

// Minimum angle between two lines in the image to consider as corner
#define MIN_CORNER_ANGLE_IN_IMAGE DEG2RAD(20)

// Maximum distance an edge of the white line pixel can be away from
// one of the corner lines.
#define CORNER_POINT_TO_LINE_DISTANCE_MAX 4

// Maximum resolution
#define MAX_RES TOP_IMAGE_COLS*TOP_IMAGE_ROWS

#define MAX_PENALTY_CROSS_CONSIDERING_DISTANCE (2000*2000)

// The minimum penalty cross region a-b distance
#define MIN_PENALTY_CROSS_DIAGONAL_DISTANCE_SQR (80*80)

// The maximun penalty cross region a-b distance
#define MAX_PENALTY_CROSS_DIAGONAL_DISTANCE_SQR (250*250)

// The maximum and minimum penalty cross width to heigh ratio in image coordinates
#define MIN_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO (0.8)
#define MAX_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO (4.0)

// The minimum number of black pixels in a group to be considered an internal region
#define MIN_GROUP_COUNT_INTERNAL_REGION_TOP 2
#define MIN_GROUP_COUNT_INTERNAL_REGION_BOT 1

// Macro for the squared distance between two Points.
#define DISTANCE_SQR(a, b) (pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2))

// Macro for the distance between two Points.
#define DISTANCE(a, b) sqrt(DISTANCE_SQR(a, b))

// Defines associated with line end finding.

// Acts as a binary enum.
#define ISWHITE true

// The number of blacks allowed before switching from white.
#define SWITCH_RATE 1

// Minimum allowed size for a stretch of white to be considered a point.
#define MIN_LINE_END_SIZE 1

// The maximum difference between the longest and second longest points.
#define PERMITTED_DIFF 0.01

// Padding width and height of region
#define PADDING_WIDTH 4
#define PADDING_HEIGHT 4

// spreadRegion_ check status
#define SPREAD_REGION_NOT_CHECKED 0
#define SPREAD_REGION_REJECTED 1
#define SPREAD_REGION_POTENTIAL_SPREAD 2
#define SPREAD_REGION_ADDED 3

// How many iterations the spread should be
#define SPREAD_ITERATION 1

// #define RFFD_USES_VATNAO
#ifdef RFFD_USES_VATNAO
#include "soccer.hpp"
#include "../VisionDebuggerInterface.hpp"
VisionPainter *p = NULL;
VisionDebugQuery q;
VisionPainter *topPainter = NULL;
VisionPainter *botPainter = NULL;
#endif // RFFD_USES_VATNAO

// #define PENALTY_SPOT_TO_FILE 1
#define PENALTY_SPOT_TO_FILE_DIR "/media/usb/PenaltySpots/"

#ifdef PENALTY_SPOT_TO_FILE
    static int num_images = 1;
#endif //PENALTY_SPOT_TO_FILE

RegionFieldFeatureDetector::RegionFieldFeatureDetector() :
                                    estimator(penaltySpot) {
#ifdef FIELD_FEATURE_TIMINGS
    frameCount = 0;
#endif // FIELD_FEATURE_TIMINGS
#ifdef RFFD_USES_VATNAO
    // Add options in vatnao
    if (vdm != NULL) {
        vdm->addOption("Show Spreaded Region");
        vdm->addOption("Show Spreaded Region With Padding");
        vdm->addOption("Show Color Saliency");
        vdm->addOption("Show Border");
        vdm->addOption("Show RegionEnds");
        vdm->addOption("Show firstEdge Points");
        vdm->addOption("Show secondEdge Points");

        vdm->addOption("Show penalty cross region");
        vdm->addOption("Show penalty cross connected component analysis");

        vdm->addOption("Show Curve Regions");
        vdm->addOption("Show Centre Point Offset Analysis");

        vdm->addOption("Show Linear Analysis Lines");

        vdm->addOption("Show Corner Regions");
        vdm->addOption("Show Corner Shape Check");
        vdm->addOption("Show T Regions");
        vdm->addOption("Show determineIfAllWhite Points");

        vdm->addOption("Show Line Regions");
    }
#endif
}

/*
Detects field features by making use of the regions produced by the region
finder.
 */
void RegionFieldFeatureDetector::detect(
    const VisionInfoIn& info_in,
    VisionInfoMiddle& info_middle,
    VisionInfoOut& info_out)
{
#ifdef FIELD_FEATURE_TIMINGS
    // Reset the timings when the frame count is at 0.
    if(frameCount == 0)
    {
        analyseRegionsTime = 0;

        spreadRegionTime = 0;
        spreadRegionAnalyseRegionBorderTime = 0;
        findOverlappingRegionIDsTime = 0;

        projectRegionEndsTime = 0;
        penaltyCrossDetectorTime = 0;
        lineCurveCornerTime = 0;
        lineCheckTime = 0;

        curveDetectorTime = 0;
        centrePointOffsetAnalysisCurveTime = 0;
        linearAnalysisTime = 0;

        cornerDetectorTime = 0;
        centrePointOffsetAnalysisCornerTime = 0;
        cornerShapeCheckTime = 0;

        tIntersectionTime = 0;

        createFeaturesTime = 0;
    }
#endif // FIELD_FEATURE_TIMINGS

    // Clear and reset existing field feature data.
    info_out.features.clear();
    info_middle.fieldFeatureRegionData.clear();
    info_middle.valid_penalty_cross.clear();
    info_middle.valid_lines.clear();
    info_middle.line_lengths.clear();
    info_middle.circle_centres.clear();
    info_middle.circle_qualities.clear();
    info_middle.circle_centre_lines.clear();
    info_middle.best_centre_circle = -1;
    info_middle.features_sent = 0;
    info_middle.intersections.clear();
    info_middle.potentialCornerIntersections.clear();
    info_middle.potentialTIntersections.clear();
    info_middle.potentialCornerIntersectionAngles.clear();
    info_middle.potentialTIntersectionAngles.clear();
    info_middle.intersection_lines.clear();
    info_middle.on_lines.clear();
    info_middle.intersection_qualities.clear();
    info_middle.intersection_corner_qualities.clear();
    info_middle.intersection_T_qualities.clear();
    info_middle.intersection_4_way_qualities.clear();
    info_middle.on_lines.clear();

    // Make space for the data associated with the ROI.
    info_middle.fieldFeatureRegionData.resize(info_middle.roi.size());


#ifdef RFFD_USES_VATNAO
    // Get query
    if (vdm != NULL)
    {
        q = vdm->getQuery();
    }
#endif

    // Analyse all regions
#ifdef FIELD_FEATURE_TIMINGS
    analyseRegionsTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    analyseRegions_(info_in, info_middle);
#ifdef FIELD_FEATURE_TIMINGS
    analyseRegionsTime += analyseRegionsTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

    // Combine the directed regions into lines, goal boxes and circles.
#ifdef FIELD_FEATURE_TIMINGS
    createFeaturesTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    createFeatures_(info_in, info_middle, info_out);
#ifdef FIELD_FEATURE_TIMINGS
    createFeaturesTime += createFeaturesTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS


#ifdef RFFD_USES_VATNAO
    // Display debug message in vatnao
    if (vdm != NULL)
    {
        vdm->setDebugMessage();
    }
#endif // RFFD_USES_VATNAO

#ifdef FIELD_FEATURE_TIMINGS
    // Display timings every 1000 frames..
    ++frameCount;
    if(frameCount == 1000)
    {
        // Display the timings.
        std::cout << std::endl << "FIELD FEATURE TIMINGS (us)" << std::endl;

        std::cout << "Region Analysis: " << ((float)analyseRegionsTime)/1000.0f
                                                                   << std::endl;

        std::cout << "\tSpread Region: " << ((float)spreadRegionTime)/1000.0f
                                                                   << std::endl;
        std::cout << "\t\tAnalyse Region Border: " <<
              ((float)spreadRegionAnalyseRegionBorderTime)/1000.0f << std::endl;
        std::cout << "\t\tFind Overlapping Region IDs: " <<
                     ((float)findOverlappingRegionIDsTime)/1000.0f << std::endl;
        std::cout << "\t\tRemaining Function: " << ((float)(spreadRegionTime -
            spreadRegionAnalyseRegionBorderTime-findOverlappingRegionIDsTime)) /
                                                           1000.0f << std::endl;

        std::cout << "\tProject Region Ends: " << ((float)projectRegionEndsTime)
                                                         / 1000.0f << std::endl;
        std::cout << "\tPenalty Cross Detector: " <<
                         ((float)penaltyCrossDetectorTime)/1000.0f << std::endl;

        std::cout << "\tLine Curve Corner Detector: " <<
                              ((float)lineCurveCornerTime)/1000.0f << std::endl;
        std::cout << "\t\tLine Checks: " << ((float)lineCheckTime)/1000.0f <<
                                                                      std::endl;

        std::cout << "\t\tCurve Detector: " << ((float)curveDetectorTime) /
                                                           1000.0f << std::endl;
        std::cout << "\t\t\tCentre Point Offset Analysis: " <<
               ((float)centrePointOffsetAnalysisCurveTime)/1000.0f << std::endl;
        std::cout << "\t\t\tLinear Analysis: " << ((float)linearAnalysisTime) /
                                                           1000.0f << std::endl;
        std::cout << "\t\t\tRemaining Function: " << ((float)(curveDetectorTime
            - centrePointOffsetAnalysisCurveTime - linearAnalysisTime)) /
                                                           1000.0f << std::endl;

        std::cout << "\t\tCorner Detector: " << ((float)cornerDetectorTime) /
                                                           1000.0f << std::endl;
        std::cout << "\t\t\tCentre Point Offset Analysis: " <<
              ((float)centrePointOffsetAnalysisCornerTime)/1000.0f << std::endl;
        std::cout << "\t\t\tCorner Shape Check: " <<
                             ((float)cornerShapeCheckTime)/1000.0f << std::endl;
        std::cout << "\t\t\tRemaining Function: " << ((float)(cornerDetectorTime
            - centrePointOffsetAnalysisCornerTime - cornerShapeCheckTime)) /
                                                           1000.0f << std::endl;

        std::cout << "\tT Intersection Detector: " << ((float)tIntersectionTime)
                                                         / 1000.0f << std::endl;

        std::cout << "Feature Creation: " << ((float)createFeaturesTime)/1000.0f
                                                                   << std::endl;

        // Reset the frame count.
        frameCount = 0;
    }
#endif // FIELD_FEATURE_TIMINGS
}

/*
Analyse regions
*/
void RegionFieldFeatureDetector::analyseRegions_(
    const VisionInfoIn& info_in,
    VisionInfoMiddle& info_middle)
{
    // Run through all the regions.
    for(unsigned int regionID=0; regionID<info_middle.roi.size(); ++regionID)
    {
#ifdef RFFD_USES_VATNAO
        // If this is the region, enable a flag to notify all function calls to
        // do their vatnao stuff
        if (vdm != NULL)
        {
            if (q.region_index == regionID){
                vdm->vision_debug_blackboard.values["REQUESTED_REGION"] = 1;
                vdm->vision_debug_blackboard.values["INTERNAL_REGION_COUNT"] = 0;
            } else {
                vdm->vision_debug_blackboard.values["REQUESTED_REGION"] = 0;
            }
        }
#endif // RFFD_USES_VATNAO

        // The region currently being analysed.
        RegionI region = info_middle.roi[regionID];
        FieldFeatureRegionData& regionData =
            info_middle.fieldFeatureRegionData[regionID];

#ifdef RFFD_USES_VATNAO
        // Draw region region by default
        if (vdm != NULL &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            p = vdm->getGivenRegionOverlayPainter(region);
            topPainter = vdm->getFrameOverlayPainter(5, true);
            botPainter = vdm->getFrameOverlayPainter(5, false);

            if (q.options["Show Color Saliency"] == "true")
                p->drawColourSaliency(region);
        }
#endif // RFFD_USES_VATNAO

#ifdef RFFD_USES_VATNAO
        // Otherwise, linear analysis points don't match with lines"
        if (vdm != NULL &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Current regionID: "<< regionID<< std::endl;
        }
#endif // RFFD_USES_VATNAO

#ifdef FIELD_FEATURE_TIMINGS
        spreadRegionTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
        region = spreadRegion_(
            region, regionData, regionID, info_middle.roi);
#ifdef FIELD_FEATURE_TIMINGS
        spreadRegionTime += spreadRegionTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

#ifdef RFFD_USES_VATNAO
        // Draw combined region on frame painter
        if (vdm != NULL && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1 &&
            (q.options["Show Spreaded Region"] == "true" || q.options["Show Spreaded Region With Padding"] == "true"))
        {
            RegionI* region = NULL;
            if (q.options["Show Spreaded Region With Padding"] == "true"){
                region = regionData.spreadedRegionWithPadding;
            } else {
                region = regionData.spreadedRegion;
            }

            BBox boundingBox = region->getBoundingBoxRaw();

            // Prety sure the painter function starts from the top left
            // of the region
            int topLeftX = boundingBox.a.x();
            int topLeftY = boundingBox.a.y();

            VisionPainter* painter = NULL;
            if (region->isTopCamera())
                painter = topPainter;
            else
                painter = botPainter;

            painter->drawRect(
                topLeftX/5,
                topLeftY/5,
                boundingBox.width()/5,
                boundingBox.height()/5,
                VisionPainter::CYAN);
        }
#endif // RFFD_USES_VATNAO

        // Project regionEnds to fieldEnds
#ifdef FIELD_FEATURE_TIMINGS
        projectRegionEndsTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
        projectRegionEndsToFieldEnds_(info_in, region, regionData);
#ifdef FIELD_FEATURE_TIMINGS
        projectRegionEndsTime += projectRegionEndsTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

        // Number of regionEnds
        const int numRegionEnds = regionData.regionEndsBorder.size();

        if (numRegionEnds == 0){

                    #ifdef RFFD_USES_VATNAO
                        if (vdm != NULL &&
                            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                        {
                            vdm->msg << std::endl << "0 region ends"<< std::endl;
                        }
                    #endif // RFFD_USES_VATNAO

            // Only run penalty spot detector if goalie
#ifdef FIELD_FEATURE_TIMINGS
            penaltyCrossDetectorTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
            if (info_in.my_player_number == 1){
                determineIfPenaltyCross(info_in, info_middle, region, regionID, regionData);
            }
#ifdef FIELD_FEATURE_TIMINGS
            penaltyCrossDetectorTime += penaltyCrossDetectorTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
        } else if (numRegionEnds == 2){

                    #ifdef RFFD_USES_VATNAO
                        if (vdm != NULL &&
                            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                        {
                            vdm->msg << std::endl << "2 region ends"<< std::endl;
                        }
                    #endif // RFFD_USES_VATNAO
#ifdef FIELD_FEATURE_TIMINGS
            lineCurveCornerTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
            determineIfLineCurveCorner_(info_in, info_middle, region, regionData);
#ifdef FIELD_FEATURE_TIMINGS
            lineCurveCornerTime += lineCurveCornerTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
        } else if (numRegionEnds == 3){

                    #ifdef RFFD_USES_VATNAO
                        if (vdm != NULL &&
                            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                        {
                            vdm->msg << std::endl << "3 region ends"<< std::endl;
                        }
                    #endif // RFFD_USES_VATNAO

#ifdef FIELD_FEATURE_TIMINGS
            tIntersectionTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
            determineIfTIntersection_(info_in, info_middle, region, regionData);
#ifdef FIELD_FEATURE_TIMINGS
            tIntersectionTime += tIntersectionTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
        } else if (numRegionEnds == 4){

                    #ifdef RFFD_USES_VATNAO
                        if (vdm != NULL &&
                            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                        {
                            vdm->msg << std::endl << "4 region ends"<< std::endl;
                        }
                    #endif // RFFD_USES_VATNAO

            // determineIfFourWayIntersection(region, regionData);
        }

    }

#ifdef RFFD_USES_VATNAO
    for(unsigned int regionID=0; regionID<info_middle.roi.size(); ++regionID)
    {
        // The region currently being analysed.
        RegionI region = info_middle.roi[regionID];
        FieldFeatureRegionData& regionData =
            info_middle.fieldFeatureRegionData[regionID];

        // Indicate which regions were classified as what,
        // on the frame images on vatnao.
        if(vdm != NULL) {
            // get the bounding box of the region
            BBox boundingBox = info_middle.roi[regionID].getBoundingBoxRaw();

            // Prety sure the painter function starts from the top left
            // of the region
            int topLeftX = boundingBox.a.x();
            int topLeftY = boundingBox.a.y();

            VisionPainter* painter = NULL;
            if (region.isTopCamera())
                painter = topPainter;
            else
                painter = botPainter;

            if (q.options["Show penalty cross region"] == "true" &&
                regionData.isPenaltyCrossConfidence == 1.0)
            {
                painter->drawRect(
                    topLeftX/5,
                    topLeftY/5,
                    boundingBox.width()/5,
                    boundingBox.height()/5,
                    VisionPainter::WHITE);

            }

            if (q.options["Show Line Regions"] == "true" &&
                regionData.isLine)
            {
                painter->drawRect(
                    topLeftX/5,
                    topLeftY/5,
                    boundingBox.width()/5,
                    boundingBox.height()/5,
                    VisionPainter::GREY);

            }

            if (q.options["Show Curve Regions"] == "true" &&
                regionData.isCurveConfidence >= 1.0f)
            {
                painter->drawRect(
                    topLeftX/5,
                    topLeftY/5,
                    boundingBox.width()/5,
                    boundingBox.height()/5,
                    VisionPainter::RED);

            }

            if (q.options["Show Corner Regions"] == "true" &&
                regionData.isCornerConfidence == 1.0)
            {
                painter->drawRect(
                    topLeftX/5,
                    topLeftY/5,
                    boundingBox.width()/5,
                    boundingBox.height()/5,
                    VisionPainter::GREEN);

            }

            if (q.options["Show T Regions"] == "true" &&
                regionData.isTConfidence == 1.0)
            {
                painter->drawRect(
                    topLeftX/5,
                    topLeftY/5,
                    boundingBox.width()/5,
                    boundingBox.height()/5,
                    VisionPainter::YELLOW);

            }
        }
    }
#endif
}


enum BorderTraverseStates {
    X_INCREMENT,
    Y_INCREMENT,
    X_DECREMENT,
    Y_DECREMENT
};

/*
Determines the next pixel to read in a traversal around the border of a region.
Not a class function - only exists here. TODO: Replace with region iterator.
*/
std::pair<Point, BorderTraverseStates> nextBorderPoint(
    int x,
    int y,
    BorderTraverseStates state,
    int cols,
    int rows)
{
    if (state == X_INCREMENT) {
        if (x < cols - 1) {
            x++;
        } else {
            state = Y_INCREMENT;
        }
    }
    if (state == Y_INCREMENT) {
        if (y < rows - 1) {
            y++;
        } else {
            state = X_DECREMENT;
        }
    }
    if (state == X_DECREMENT) {
        if (x > 0) {
            x--;
        } else {
            state = Y_DECREMENT;
        }
    }
    if (state == Y_DECREMENT) {
        y--;
    }
    return std::pair<Point, BorderTraverseStates>(Point(x, y), state);
}

/*
Spread seedRegion to neighbors if regionEnds don't change. This can spread for
SPREAD_ITERATION times.

SPREAD_ITERATION = 1 : Combine directly overlapping and neighbor regions
SPREAD_ITERATION = 2 : Combine with neighbor's neighbors
...
*/
RegionI& RegionFieldFeatureDetector::spreadRegion_(
    const RegionI& seedRegion,
    FieldFeatureRegionData& seedRegionData,
    const unsigned int seedRegionID,
    const std::vector<RegionI>& regions)
{
    // Copy original region, and store in data
    RegionI* spreadedRegion = new RegionI(seedRegion);
    seedRegionData.spreadedRegion = spreadedRegion;
    seedRegionData.relatedRegions.push_back(spreadedRegion);

    // Pad original region, and store in data
    RegionI* spreadedRegionWithPadding = padRegion_(seedRegion, seedRegionData);
    seedRegionData.spreadedRegionWithPadding = spreadedRegionWithPadding;
    seedRegionData.relatedRegions.push_back(spreadedRegionWithPadding);


    // Analyse region border
    std::vector<int> regionEndsBorder;
    std::vector<Point, Eigen::aligned_allocator<Point> > regionEnds;
    std::vector<int> regionEndSizes;
    std::vector<std::pair<int, int> > borderEndPairs;
    std::vector<std::pair<Point, Point> > regionEndXYPairs;

#ifdef FIELD_FEATURE_TIMINGS
    spreadRegionAnalyseRegionBorderTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    analyseRegionBorder_(*spreadedRegionWithPadding,
                         regionEndsBorder,
                         regionEnds,
                         regionEndSizes,
                         borderEndPairs,
                         regionEndXYPairs);
#ifdef FIELD_FEATURE_TIMINGS
    spreadRegionAnalyseRegionBorderTime +=
                              spreadRegionAnalyseRegionBorderTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

    // Vector storing whether regions have been checked or not
    std::vector<int> regionsSpreadStatus(regions.size(), SPREAD_REGION_NOT_CHECKED);

    // Start with the seedRegionID
    regionsSpreadStatus[seedRegionID] = SPREAD_REGION_ADDED;

    // Finds ALL overlapping regions
#ifdef FIELD_FEATURE_TIMINGS
    findOverlappingRegionIDsTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    std::vector<int> overLappingRegionIDs =
        findOverlappingRegionIDs_(*spreadedRegion, regions);
#ifdef FIELD_FEATURE_TIMINGS
    findOverlappingRegionIDsTime += findOverlappingRegionIDsTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

    // Looks for overlapping regions that haven't been checked yet, and label them
    // as potential spread regionIDs in regionsSpreadStatus
    for (std::vector<int>::iterator it = overLappingRegionIDs.begin();
            it != overLappingRegionIDs.end();
            ++it)
    {
        if (regionsSpreadStatus[*it] == SPREAD_REGION_NOT_CHECKED)
            regionsSpreadStatus[*it] = SPREAD_REGION_POTENTIAL_SPREAD;
    }

    // Replace following for loop with this while loop to allow spreading until
    // no more spreadable regions.
    // while (std::find(
    //         regionsSpreadStatus.begin(),
    //         regionsSpreadStatus.end(),
    //         SPREAD_REGION_POTENTIAL_SPREAD) != regionsSpreadStatus.end())

    // For SPREAD_ITERATION times
    for (unsigned spreadIteration=0; spreadIteration < SPREAD_ITERATION; ++spreadIteration)
    {

#ifdef RFFD_USES_VATNAO
        // // Print regionSpreadStatus
        // if (vdm != NULL &&
        //         vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        // {
        //     vdm->msg << "regionSpreadStatus: ";
        //     for (std::vector<int>::const_iterator i = regionsSpreadStatus.begin(); i != regionsSpreadStatus.end(); ++i)
        //         vdm->msg << *i << ", ";
        //     vdm->msg << std::endl;
        // }
#endif // RFFD_USES_VATNAO

        // Store all potential Spread RegionIDs in vector
        std::vector<int> potentialSpreadRegionIDs;
        for (unsigned i=0; i<regionsSpreadStatus.size(); ++i){
            if (regionsSpreadStatus[i] == SPREAD_REGION_POTENTIAL_SPREAD){
                potentialSpreadRegionIDs.push_back(i);
            }
        }

        // For all potential spread regions, try expanding
        for (std::vector<int>::iterator it = potentialSpreadRegionIDs.begin();
                it != potentialSpreadRegionIDs.end();
                ++it)
        {
            // Experimental spread
            std::vector<int> regionEndsBorderExperiment;
            std::vector<Point, Eigen::aligned_allocator<Point> > regionEndsExperiment;
            std::vector<int> regionEndSizesExperiment;
            std::vector<std::pair<int, int> > borderEndPairsExperiment;
            std::vector<std::pair<Point, Point> > regionEndXYPairsExperiment;

            // Experimentally combine and pad region
            RegionI* experimentalSpreadRegion =
                combineRegions_(*spreadedRegion, regions[*it]);
            RegionI* experimentalSpreadRegionPadded =
                padRegion_(*experimentalSpreadRegion, seedRegionData);

            // add the regions to relatedRegions (so they get deleted at the end)
            seedRegionData.relatedRegions.push_back(experimentalSpreadRegion);
            seedRegionData.relatedRegions.push_back(experimentalSpreadRegionPadded);

#ifdef FIELD_FEATURE_TIMINGS
            spreadRegionAnalyseRegionBorderTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
            analyseRegionBorder_(*experimentalSpreadRegionPadded,
                                regionEndsBorderExperiment,
                                regionEndsExperiment,
                                regionEndSizesExperiment,
                                borderEndPairsExperiment,
                                regionEndXYPairsExperiment);
#ifdef FIELD_FEATURE_TIMINGS
            spreadRegionAnalyseRegionBorderTime +=
                              spreadRegionAnalyseRegionBorderTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

            // If number of region ends didn't change, then spreading succeeded! Update
            // spreadedRegion and spreadedRegionWithPadding. Store these in seedRegionData.
            // Also,update all "end" information
            if (regionEndsExperiment.size() == regionEnds.size()){
                // Update information
                spreadedRegion = experimentalSpreadRegion;
                spreadedRegionWithPadding = experimentalSpreadRegionPadded;
                regionEndsBorder = regionEndsBorderExperiment;
                regionEnds = regionEndsExperiment;
                regionEndSizes = regionEndSizesExperiment;
                borderEndPairs = borderEndPairsExperiment;
                regionEndXYPairs = regionEndXYPairsExperiment;
                // Update seedRegionData
                seedRegionData.neighborRegionIDs.push_back(*it);
                seedRegionData.spreadedRegion = spreadedRegion;
                seedRegionData.spreadedRegionWithPadding = spreadedRegionWithPadding;

                // Mark region as SPREAD_REGION_ADDED
                regionsSpreadStatus[*it] = SPREAD_REGION_ADDED;
            } else {
                // Mark region as SPREAD_REGION_REJECTED
                regionsSpreadStatus[*it] = SPREAD_REGION_REJECTED;
            }
        }

        // Finds ALL overlapping regions, including ones already added and rejected
#ifdef FIELD_FEATURE_TIMINGS
        findOverlappingRegionIDsTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
        std::vector<int> overLappingRegionIDs =
            findOverlappingRegionIDs_(*spreadedRegion, regions);
#ifdef FIELD_FEATURE_TIMINGS
        findOverlappingRegionIDsTime +=
                                     findOverlappingRegionIDsTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

        // Looks for overlapping regions that haven't been checked yet, and label them
        // as potential spread regionIDs in regionsSpreadStatus
        for (std::vector<int>::iterator it = overLappingRegionIDs.begin();
             it != overLappingRegionIDs.end();
             ++it)
        {
            if (regionsSpreadStatus[*it] == SPREAD_REGION_NOT_CHECKED)
                regionsSpreadStatus[*it] = SPREAD_REGION_POTENTIAL_SPREAD;
        }
    }

    // Store all information
    seedRegionData.regionEndsBorder = regionEndsBorder;
    seedRegionData.regionEnds = regionEnds;
    seedRegionData.regionEndSizes = regionEndSizes;
    seedRegionData.borderEndPairs = borderEndPairs;
    seedRegionData.regionEndXYPairs = regionEndXYPairs;

#ifdef RFFD_USES_VATNAO
    // Draw Region
    if (vdm != NULL &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        if (q.options["Show Spreaded Region With Padding"] == "true"){
            p = vdm->getGivenRegionOverlayPainter(*seedRegionData.spreadedRegionWithPadding);
            if (q.options["Show Color Saliency"] == "true")
                p->drawColourSaliency(*seedRegionData.spreadedRegionWithPadding);
        } else if (q.options["Show Spreaded Region"] == "true"){
            p = vdm->getGivenRegionOverlayPainter(*seedRegionData.spreadedRegion);
            if (q.options["Show Color Saliency"] == "true")
                p->drawColourSaliency(*seedRegionData.spreadedRegion);
        }
    }

    // Redo the border.
    analyseRegionBorder_(*spreadedRegionWithPadding, regionEndsBorder,
                  regionEnds, regionEndSizes, borderEndPairs, regionEndXYPairs);

    // Show regionEnds
    if (vdm != NULL && q.options["Show RegionEnds"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        for (std::vector<std::pair<Point, Point> >::iterator it = regionEndXYPairs.begin();
             it != regionEndXYPairs.end();
             ++it)
        {
            p->draw((*it).first.x(), (*it).first.y(), VisionPainter::BLACK);
            p->draw((*it).second.x(), (*it).second.y(), VisionPainter::BLACK);
        }
    }
#endif // RFFD_USES_VATNAO

    return *spreadedRegionWithPadding;
}

/*
Get all regionIDs that are neighbors or overlapping
*/
std::vector<int> RegionFieldFeatureDetector::findOverlappingRegionIDs_(
    const RegionI& region,
    const std::vector<RegionI>& regions)
{
    std::vector<int> overlappingRegionIDs;

    BBox regionBBoxRaw = region.getBoundingBoxRaw();
    int regionAX = regionBBoxRaw.a.x();
    int regionAY = regionBBoxRaw.a.y();
    int regionBX = regionBBoxRaw.b.x();
    int regionBY = regionBBoxRaw.b.y();
    int regionDensity = region.getDensity();
    bool regionIsTopCamera = region.isTopCamera();

    for(unsigned int i=0; i<regions.size(); ++i)
    {
        if (regions[i].isTopCamera() != regionIsTopCamera)
            // region is in different camera
            continue;

        int testRegionAX = regions[i].getBoundingBoxRaw().a.x();
        int testRegionAY = regions[i].getBoundingBoxRaw().a.y();
        int testRegionBX = regions[i].getBoundingBoxRaw().b.x();
        int testRegionBY = regions[i].getBoundingBoxRaw().b.y();

        if (regionAX - testRegionBX > regionDensity)
            // testRegion is left of region
            continue;
        if (regionAY - testRegionBY > regionDensity)
            // testRegion is above region
            continue;
        if (testRegionAX - regionBX > regionDensity)
            // testRegion is right of region
            continue;
        if (testRegionAY - regionBY > regionDensity)
            // testRegion is below region
            continue;

        // Regions are overlapping
        overlappingRegionIDs.push_back(i);
    }

    return overlappingRegionIDs;
}

/*
Combine two regions, return pointer to new region
*/
RegionI* RegionFieldFeatureDetector::combineRegions_(
    const RegionI& region1,
    const RegionI& region2)
{

    // Construct bounding box, RELATIVE TO REGION1
    const BBox region1BBox = region1.getBoundingBoxRaw();
    const BBox region2BBox = region2.getBoundingBoxRaw();

    const int density = region1.getDensity();

    const int combinedBBoxAX =
        (std::min(region1BBox.a.x(), region2BBox.a.x()) - region1BBox.a.x()) / density;
    const int combinedBBoxAY =
        (std::min(region1BBox.a.y(), region2BBox.a.y()) - region1BBox.a.y()) / density;
    const int combinedBBoxBX =
        (std::max(region1BBox.b.x(), region2BBox.b.x()) - region1BBox.a.x()) / density;
    const int combinedBBoxBY =
        (std::max(region1BBox.b.y(), region2BBox.b.y()) - region1BBox.a.y()) / density;

    const BBox combinedBBox(Point(combinedBBoxAX, combinedBBoxAY),
                            Point(combinedBBoxBX, combinedBBoxBY));

    RegionI* combinedRegion = new RegionI(region1, combinedBBox);

    return combinedRegion;
}

/*
Pad region to ensure the feature is included and not unnecesarily touching the border
*/
RegionI* RegionFieldFeatureDetector::padRegion_(
    const RegionI& region,
    FieldFeatureRegionData& data)
{
    const BBox originalBoundsRaw = region.getBoundingBoxRaw();
    BBox newBoundsRaw = region.getBoundingBoxRaw();

    int density = region.getDensity();

    newBoundsRaw.a.x() -= PADDING_WIDTH * density;
    newBoundsRaw.b.x() += PADDING_WIDTH * density;
    newBoundsRaw.a.y() -= PADDING_HEIGHT * density;
    newBoundsRaw.b.y() += PADDING_HEIGHT * density;

    if (region.isTopCamera()){
        newBoundsRaw.a.x() = std::max(0, newBoundsRaw.a.x());
        newBoundsRaw.a.y() = std::max(0, newBoundsRaw.a.y());
        newBoundsRaw.b.x() = std::min(TOP_IMAGE_COLS-1, newBoundsRaw.b.x());
        newBoundsRaw.b.y() = std::min(TOP_IMAGE_ROWS-1, newBoundsRaw.b.y());
    } else {
        newBoundsRaw.a.x() = std::max(0, newBoundsRaw.a.x());
        newBoundsRaw.a.y() = std::max(0, newBoundsRaw.a.y());
        newBoundsRaw.b.x() = std::min(BOT_IMAGE_COLS-1, newBoundsRaw.b.x());
        newBoundsRaw.b.y() = std::min(BOT_IMAGE_ROWS-1, newBoundsRaw.b.y());
    }

    BBox newBoundsFovea(
        Point((newBoundsRaw.a.x() - originalBoundsRaw.a.x()) / density,
              (newBoundsRaw.a.y() - originalBoundsRaw.a.y()) / density),
        Point((newBoundsRaw.b.x() - originalBoundsRaw.a.x()) / density,
              (newBoundsRaw.b.y() - originalBoundsRaw.a.y()) / density)
    );

    RegionI *paddedRegion = new RegionI(region, newBoundsFovea, 1, 0, true);

    return paddedRegion;
}

/*
Analyses border. Populates regionEndsBorder, regionEnds, regionEndSizes,
borderEndPairs, regionEndXYPairs.
*/
void RegionFieldFeatureDetector::analyseRegionBorder_(
    const RegionI& region,
    std::vector<int>& regionEndsBorder,
    std::vector<Point, Eigen::aligned_allocator<Point> >& regionEnds,
    std::vector<int>& regionEndSizes,
    std::vector<std::pair<int, int> >& borderEndPairs,
    std::vector<std::pair<Point, Point> >& regionEndXYPairs)
{

    // Length of the border around the region. Don't count the corners
    // twice.
    const int numCols = region.getCols();
    const int numRows = region.getRows();
    const int borderLength = numCols * 2 + numRows * 2 - 4;

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        vdm->msg << "Region dimension: ("<<numCols<<", "<<numRows<<")" << std::endl;
#endif // RFFD_USES_VATNAO

    // Construct a vector of booleans indicating whether or not the pixel along
    // the border is white or not.
    const std::vector<bool> border =
        constructBorder_(region, numCols, numRows, borderLength);

    findRegionEnds_(region,
                    numCols,
                    numRows,
                    border,
                    borderLength,
                    regionEndsBorder,
                    regionEnds,
                    regionEndSizes,
                    borderEndPairs,
                    regionEndXYPairs);

}


/*
Construct a vector of booleans indicating whether or not the pixel along
the border is white or not.
*/
const std::vector<bool> RegionFieldFeatureDetector::constructBorder_(
    const RegionI& region,
    const int numCols,
    const int numRows,
    const int borderLength)
{
    // Make space for the border points.
    std::vector<bool> border(borderLength);

    // Get an iterator to the upper left of the region.
    RegionI::iterator_fovea curPixel = region.begin_fovea();

    // Determine the segment sizes.
    int topEnd = numCols;
    int rightEnd = numCols+numRows-1;
    int botEnd = numCols+numRows+numCols-2;

    // Run through the top row.
    --curPixel;
    for(int i=0; i<topEnd; ++i)
    {
        ++curPixel;
        border[i] = curPixel.colour() == cWHITE;
    }

    // Run through the right column.
    for(int i=topEnd; i<rightEnd; ++i)
    {
        curPixel.next_row();
        border[i] = curPixel.colour() == cWHITE;
    }

    // Run through the bottom row.
    for(int i=rightEnd; i<botEnd; ++i)
    {
        --curPixel;
        border[i] = curPixel.colour() == cWHITE;
    }

    // Run through the left column.
    for(int i=botEnd; i<borderLength; ++i)
    {
        curPixel.last_row();
        border[i] = curPixel.colour() == cWHITE;
    }

#ifdef RFFD_USES_VATNAO
    // Show border, with yellow and blue pixels
    if (vdm != NULL && q.options["Show Border"] == "true" &&
           vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        for(int i=0; i<topEnd; ++i)
        {
            if (border[i]) {
                p->draw(i, 0, VisionPainter::YELLOW);
            } else {
                p->draw(i, 0, VisionPainter::BLUE);
            }
        }
        for(int i=topEnd; i<rightEnd; ++i)
        {
            if (border[i]) {
                p->draw(numCols-1, i-topEnd+1, VisionPainter::YELLOW);
            } else {
                p->draw(numCols-1, i-topEnd+1, VisionPainter::BLUE);
            }
        }
        for(int i=rightEnd; i<botEnd; ++i)
        {
            if (border[i]) {
                p->draw(numCols-(i-rightEnd)-2, numRows-1,
                                                         VisionPainter::YELLOW);
            } else {
                p->draw(numCols-(i-rightEnd)-2, numRows-1, VisionPainter::BLUE);
            }
        }
        for(int i=botEnd; i<borderLength; ++i)
        {
            if (border[i]) {
                p->draw(0, numRows-(i-botEnd)-2, VisionPainter::YELLOW);
            } else {
                p->draw(0, numRows-(i-botEnd)-2, VisionPainter::BLUE);
            }
        }
    }
#endif // RFFD_USES_VATNAO

    return border;
}

/*
Find the first point where we have a definite black along the border
*/
int findFirstBlack(
    const std::vector<bool>& border,
    const int length, const int switchRate)
{
    int numBlacks = 0;

    for (int i = 0; i < length; i++)
    {
        if (border[i] == ISWHITE)
            numBlacks = 0;
        else
            numBlacks++;

        // If we've found enough blacks to break a white streak
        if (numBlacks == switchRate + 1)
            return i-switchRate;
    }

    // If we haven't found enough blacks to break white
    return -1;
}

/*
Returns true if this pixel is white or we saw a white pixel SWITCH_RATE pixels
ago. Prevents creating multiple end points due to noise.
*/
bool stillWhite_(
    const std::vector<bool>& border,
    int i,
    bool was_white,
    int num_whites,
    int num_blacks,
    int switchRate,
    int startPoint)
{
    if(i==startPoint)
        return false;

    if (was_white)
    {
        if (border[i] == ISWHITE || num_blacks < switchRate)
            return true;
    }
    else
    {
        if (border[i] == ISWHITE)
            return true;
    }
    return false;
}


/*
Converts the ID of a point along a region border to (x, y) coordinates in the
region.
*/
Point borderPointToXY_(
    int numCols,
    int numRows,
    int point)
{
    if (point < numCols)
        return Point(point, 0);
    else if (point < numCols + numRows - 1)
        return Point(numCols - 1, point - numCols + 1);
    else if (point < numCols * 2 + numRows - 2)
        return Point(numCols - (point - numCols - numRows + 3), numRows-1);
    else
        return Point(0, numRows - (point - numCols * 2 - numRows + 4));
}

int normaliseBorderPixel_(
    int borderLength,
    int borderPixel)
{
    while (borderPixel < 0){
        borderPixel += borderLength;
    }
    while (borderPixel >= borderLength){
        borderPixel -= borderLength;
    }
    return borderPixel;
}

/*
Find the "ends" of each region. Populates regionEndsBorder, regionEnds, regionEndSizes,
borderEndPairs, regionEndXYPairs.
*/
void RegionFieldFeatureDetector::findRegionEnds_(
    const RegionI& region,
    const int numCols,
    const int numRows,
    const std::vector<bool>& border,
    const int borderLength,
    std::vector<int>& regionEndsBorder,
    std::vector<Point, Eigen::aligned_allocator<Point> >& regionEnds,
    std::vector<int>& regionEndSizes,
    std::vector<std::pair<int, int> >& borderEndPairs,
    std::vector<std::pair<Point, Point> >& regionEndXYPairs)
{
    // Dynamic change switch rate based on each region
    int switchRate = std::max(SWITCH_RATE, (borderLength/100));

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Border"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        vdm->msg << "Switch-rate: " << switchRate << std::endl;
#endif // RFFD_USES_VATNAO

    // find the first point where we have a definite black along the border
    int borderPixel = findFirstBlack(border, borderLength, switchRate);

    // Whole border is white, return
    if (borderPixel == -1)
        return;

    bool wasWhite = false;
    int numWhites = 0;
    int numBlacks = 1;
    int startPoint = borderPixel;
    bool passedZero = false;
    std::pair<int, int> borderEndPair;
    int endFirst = 0;
    int endLast = 0;

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Border"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        vdm->msg << "Start At: " << startPoint << std::endl;
#endif // RFFD_USES_VATNAO

    // Loop around the frame until we reach just past where we started
    while(borderPixel != startPoint || !passedZero)
    {
        // Move to the next border pixel.
        ++borderPixel;

        // If we've run back to the first pixel continue till we reach the first
        // black pixel (startPoint).

        if (borderPixel == borderLength)
        {
            borderPixel = 0;
            passedZero = true;
        }

        // Add to a new end point while travelling along a white section.
        if (stillWhite_(border, borderPixel, wasWhite, numWhites, numBlacks, switchRate, startPoint))
        {

#ifdef RFFD_USES_VATNAO
            if (vdm != NULL && q.options["Show Border"] == "true" &&
                vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1 &&
                wasWhite == false)
            {
                vdm->msg << "White start At: " << borderPixel << "after: " <<
                                           numBlacks << " blacks" << std::endl;
            }
#endif // RFFD_USES_VATNAO

            numWhites++;
            wasWhite = true;
            if (border[borderPixel] != ISWHITE)
                numBlacks++;
            else
                numBlacks = 0;
        }

        // While in a black section do nothing unless a new end point has just
        // been completed.
        else
        {
            // A new end point has just been completed.
            if (wasWhite)
            {
#ifdef RFFD_USES_VATNAO
                if (vdm != NULL && q.options["Show Border"] == "true" &&
                    vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                {
                    vdm->msg << "Break At: " << borderPixel << " after: " <<
                                           numWhites << " whites" << std::endl;
                }
#endif // RFFD_USES_VATNAO

                // Account for extra black pixels added due to the noise
                // management system.
                int actualNumWhites;
                if(borderPixel==0)
                {
                    actualNumWhites = numWhites-switchRate;
                    int centrePoint = borderPixel - numWhites + int(actualNumWhites/2);
                    if (centrePoint<0) centrePoint+=borderLength;
                    if(actualNumWhites >= MIN_LINE_END_SIZE)
                    {
                        Point centrePointXY = borderPointToXY_(numCols,
                                                           numRows, centrePoint);
                        endLast = borderPixel-switchRate-1;
                        endLast = normaliseBorderPixel_(borderLength, endLast);
                        endFirst = borderPixel-actualNumWhites-switchRate;
                        endFirst = normaliseBorderPixel_(borderLength, endFirst);
                        borderEndPair=std::make_pair(endFirst, endLast);
                        std::pair<Point, Point> regionEndXYPair =
                            std::make_pair(
                                borderPointToXY_(numCols,numRows, borderEndPair.first),
                                borderPointToXY_(numCols,numRows, borderEndPair.second));
                        // Construct a new line end.
                        regionEndsBorder.push_back(centrePoint);
                        regionEnds.push_back(centrePointXY);
                        regionEndSizes.push_back(actualNumWhites);
                        borderEndPairs.push_back(borderEndPair);
                        regionEndXYPairs.push_back(regionEndXYPair);                        
                    }                    
                }
                else if(borderPixel==startPoint)
                {
                    actualNumWhites = numWhites;
                    // Find the centre point of the line end.
                    int centrePoint = borderPixel - int(actualNumWhites/2);
                    if (centrePoint<0) centrePoint+=borderLength;

                    if(actualNumWhites >= MIN_LINE_END_SIZE)
                    {

                        Point centrePointXY = borderPointToXY_(numCols,
                                                           numRows, centrePoint);
                        endLast = borderPixel-1;
                        endLast = normaliseBorderPixel_(borderLength, endLast);
                        endFirst = borderPixel-actualNumWhites;
                        endFirst = normaliseBorderPixel_(borderLength, endFirst);
                        borderEndPair=std::make_pair(endFirst, endLast);
                        std::pair<Point, Point> regionEndXYPair =
                            std::make_pair(
                                borderPointToXY_(numCols,numRows, borderEndPair.first),
                                borderPointToXY_(numCols,numRows, borderEndPair.second));
                        // Construct a new line end.
                        regionEndsBorder.push_back(centrePoint);
                        regionEnds.push_back(centrePointXY);
                        regionEndSizes.push_back(actualNumWhites);
                        borderEndPairs.push_back(borderEndPair);
                        regionEndXYPairs.push_back(regionEndXYPair);

#ifdef RFFD_USES_VATNAO
                        // Show regionEnds
                        if (vdm != NULL && q.options["Show RegionEnds"] == "true" &&
                            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
                        {
                            p->draw(regionEndXYPair.first.x(), regionEndXYPair.first.y(), VisionPainter::BLACK);
                            p->draw(regionEndXYPair.second.x(), regionEndXYPair.second.y(), VisionPainter::BLACK);
                            p->draw(centrePointXY.x(), centrePointXY.y(), VisionPainter::BLACK);
                        }
#endif // RFFD_USES_VATNAO
                    }
                }
                else
                {
                    if (!passedZero)
                        actualNumWhites = (border[borderPixel - switchRate] ==
                                        ISWHITE) ? numWhites : numWhites - switchRate;
                    else
                        actualNumWhites = (border[borderLength - switchRate] ==
                                        ISWHITE) ? numWhites : numWhites - switchRate;

                    // Find the centre point of the line end.
                    int centrePoint = borderPixel - numWhites +
                                                           int(actualNumWhites/2);
                    if (centrePoint<0) centrePoint+=borderLength;


                    if(actualNumWhites >= MIN_LINE_END_SIZE)
                    {

                        Point centrePointXY = borderPointToXY_(numCols,
                                                           numRows, centrePoint);
                        endLast = borderPixel-switchRate-1;
                        endLast = normaliseBorderPixel_(borderLength, endLast);
                        endFirst = borderPixel-actualNumWhites-switchRate;
                        endFirst = normaliseBorderPixel_(borderLength, endFirst);
                        borderEndPair=std::make_pair(endFirst, endLast);
                        std::pair<Point, Point> regionEndXYPair =
                            std::make_pair(
                                borderPointToXY_(numCols,numRows, borderEndPair.first),
                                borderPointToXY_(numCols,numRows, borderEndPair.second));
                        // Construct a new line end.
                        regionEndsBorder.push_back(centrePoint);
                        regionEnds.push_back(centrePointXY);
                        regionEndSizes.push_back(actualNumWhites);
                        borderEndPairs.push_back(borderEndPair);
                        regionEndXYPairs.push_back(regionEndXYPair);
                    }
                }
            }

            // This is not a white section.
            wasWhite = false;
            numWhites = 0;
            ++numBlacks;
        }

        
        
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Border"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << borderPixel << " ";
        }
#endif // RFFD_USES_VATNAO
    }

}

/*
Check if there is internal black region in a petential penalty cross
*/
void RegionFieldFeatureDetector::checkInternalNotWhiteRegion(const RegionI& base_region)
{
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

    // Number of white pixels Connected Component Analysis
    num_whites = 0;

    // Whether connected component analysis detected body part
    has_body_part = false;

    // Set to USHRT_MAX when there is no neighbour.
    uint16_t top_neighbour = USHRT_MAX;
    uint16_t left_neighbour = USHRT_MAX;
    bool has_neighbour = false;

    // Iterator that move through the region.
    RegionI::iterator_fovea cur_point = base_region.begin_fovea();

    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;

    // A pointer tracking through the underlying array of groups.
    uint16_t* group = groups_;

    // The number of rows and columns in the region.
    int rows = base_region.getRows();
    int cols = base_region.getCols();
    // Connected component analysis.

    for(int pixel=0; pixel < cols*rows; ++pixel)
    {
        // If this is not a white pixel, group it.
        if (cur_point.colour() != cWHITE)
        {
            if (cur_point.colour() == cBODY_PART)
                has_body_part = true;

            // Get all neighbours.
            has_neighbour = false;
            if (x != 0 && (cur_point.colourLeft() != cWHITE))
            {
                left_neighbour = *(group-1);
                has_neighbour = true;
            }
            else
                left_neighbour = USHRT_MAX;

            if (y != 0 && (cur_point.colourAbove() != cWHITE))
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
                    if(group_high_xs_[top_neighbour] < x) {
                        group_high_xs_[top_neighbour] = x;
                    }
                    if(group_high_ys_[top_neighbour] < y) {
                        group_high_ys_[top_neighbour] = y;
                    }
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
        } else {
            num_whites++;
        }

        ++x;
        ++group;
        ++cur_point;
        if(x == cols)
        {
            x = 0;
            ++y;
        }
    }

    // Don't need a full second pass as we only need bounding boxes. Combining
    // by grabbing pixel location extremes is sufficient. May be a faster way
    // to implement this.

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
                    new_link = group_links_.addLink(group_links_.get(group, owner),
                                                    group_links_.get(group, 0));
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
}

/*
Determines and labels a region if it is penalty cross.
Checks are done in inexpensive order.
*/
void RegionFieldFeatureDetector::determineIfPenaltyCross(const VisionInfoIn& info_in,
                VisionInfoMiddle& info_middle, const RegionI& pad_region,
                const unsigned int regionID, FieldFeatureRegionData& data)
{
    // Analyse spreadedRegion for most checks
    const RegionI& spreadedRegion = *(data.spreadedRegion);
    int spreadedRegionWidth = spreadedRegion.getCols();
    int spreadedRegionHeight = spreadedRegion.getRows();

    // 1.1. Test minimum Width-to-height ratio in image
    float widthToHeightRatio = (float) spreadedRegionWidth / (float) spreadedRegionHeight;

    if (widthToHeightRatio < MIN_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO) {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as width to height ratio is below MIN_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 1.2. Test maximum Width-to-height ratio in image
    if (widthToHeightRatio > MAX_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO) {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as width to height ratio is above MAX_PENALTY_CROSS_WIDTH_TO_HEIGHT_RATIO"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }



    // 2. Check middle pixel is white
    if (spreadedRegion.getPixelColour(spreadedRegionWidth/2, spreadedRegionHeight/2) != cWHITE) {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as middle pixel not white"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 3. Check region doesnt have neighbor regions
    if (data.neighborRegionIDs.size() != 0)
    {
        data.isPenaltyCrossConfidence = 0.0f;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as there is region nearby"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return;
    }

    // 4. Check robot-relative points

    // get top-left bottom-right two points
    Point A = spreadedRegion.getBoundingBoxRaw().a;
    Point B = spreadedRegion.getBoundingBoxRaw().b;

    if (!spreadedRegion.isTopCamera())
    {
        A.y() += TOP_IMAGE_ROWS;
        B.y() += TOP_IMAGE_ROWS;
    }

    Point globalA = info_in.cameraToRR.pose.imageToRobotXY(A);
    Point globalB = info_in.cameraToRR.pose.imageToRobotXY(B);

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg <<std::endl<<"globalA:("<<globalA.x()<<", "<<globalA.y()<<")"<< std::endl
            <<"globalB:("<<globalB.x()<<", "<<globalB.y()<<")"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

    // 4.1. Check distance to penalty spot in rr
    if(DISTANCE_SQR(globalA, Point(0,0)) > MAX_PENALTY_CROSS_CONSIDERING_DISTANCE
        || DISTANCE_SQR(globalB, Point(0,0)) > MAX_PENALTY_CROSS_CONSIDERING_DISTANCE)
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as too far"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 4.2. Check minimum diagonal distance of penalty cross in rr
    if(DISTANCE_SQR(globalA, globalB) < MIN_PENALTY_CROSS_DIAGONAL_DISTANCE_SQR)
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as size too small"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 4.3. Check maximum diagonal distance of penalty cross in rr
    if(DISTANCE_SQR(globalA, globalB) > MAX_PENALTY_CROSS_DIAGONAL_DISTANCE_SQR)
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as size too large"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // @ijnek: Commencted out following check as tends to fail
    // 4.4. Check width to height ratio of penalty cross in rr
    // The width and height.
    // int width = std::abs(globalB.x()-globalA.x());
    // int height = std::abs(globalB.y()-globalA.y());

//     if(width > 2.0 * height || height > 2.0 * width)
//     {
//         data.isPenaltyCrossConfidence = 0.0f;
// #ifdef RFFD_USES_VATNAO
//         if (vdm != NULL && q.options["Show penalty cross region"] == "true"
//             && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
//         {
//             vdm->msg << "Penalty cross detection failed as shape not correct"<< std::endl;
//         }
// #endif // RFFD_USES_VATNAO
//         return;
//     }

    // 5. Iterate over region to do
    //      - Connected Component Analysis
    //      - Count white pixels
    //      - Check for robot body parts
    checkInternalNotWhiteRegion(pad_region);

    // 5.1. Check there are no internal regions

    int min_group_count = MIN_GROUP_COUNT_INTERNAL_REGION_BOT;

    if (spreadedRegion.isTopCamera())
        min_group_count = MIN_GROUP_COUNT_INTERNAL_REGION_TOP;


    int numBlackRegion = 0;
    for(int group=0; group<group_links_.size(); group++)
    {
        // Check the group actually has pixels.
        if(group_counts_[group] >= min_group_count)
        {
            ++numBlackRegion;

#ifdef RFFD_USES_VATNAO
            if (vdm != NULL
                && q.options["Show Spreaded Region"] == "true"
                && q.options["Show penalty cross connected component analysis"] == "true"
                && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
            {
                p->drawRect(
                    group_low_xs_[group],
                    group_low_ys_[group],
                    group_high_xs_[group] - group_low_xs_[group],
                    group_high_ys_[group] - group_low_ys_[group],
                    VisionPainter::BLUE);
            }
#endif // RFFD_USES_VATNAO

        }
    }

    if(numBlackRegion>=2)
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Number of black regions: "<< numBlackRegion<<std::endl;
            vdm->msg << "Penalty cross detection failed as there are internal region"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 5.2. Check for minimum white pixels
    if((spreadedRegion.isTopCamera() && num_whites < 30) ||
         (!spreadedRegion.isTopCamera() && num_whites < 10))
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as total_whites below minimum"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 5.3. Check for robot parts
    if(has_body_part)
    {
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed as body part included"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

    // 6. Use GMM classifier check
    if (estimator.predict(spreadedRegion) == CLASSIFIER_FALSE)
    {
        // std::cout << "rejected by gmm!" << std::endl;
        data.isPenaltyCrossConfidence = 0.0f;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection failed at GMM"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

        return;
    }

#ifdef PENALTY_SPOT_TO_FILE
    char name[] = "a";
    char location[] = PENALTY_SPOT_TO_FILE_DIR;
    std::stringstream dir;
    dir << location << num_images << ".png";
    num_images++;
    std::cout << "Writing binary penalty spot image to: " << dir.str().c_str() << "\n";
    WriteImage w;
    if(w.writeImage(spreadedRegion, COLOUR_FORMAT, dir.str().c_str(), name)) {
        std::cout << "Success\n";
    }
    else {
        std::cout << "Failed\n";
    }
#endif  // PENALTY_SPOT_TO_FILE


    // else is penalty cross
    data.isPenaltyCrossConfidence = 1.0f;

    info_middle.valid_penalty_cross.push_back((globalA + globalB) / 2);

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show penalty cross region"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Penalty cross detection success"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
}

/*
Check if all points on segment AB are white, in region
*/
bool RegionFieldFeatureDetector::determineIfAllWhite(const RegionI& region, Point& A, Point& B)
{
    // length of the segment AB
    float length = std::sqrt((float)( (A.x()-B.x())*(A.x()-B.x()) + (A.y()-B.y())*(A.y()-B.y()) ));

    Point curPoint = A;
    float distance = 1.0f;

    // The width and height.
    int width = region.getCols();
    int height = region.getRows();

    do{
        curPoint.x() = A.x()*(length-distance)/length + B.x()*distance/length;
        curPoint.y() = A.y()*(length-distance)/length + B.y()*distance/length;
        if(curPoint.x()<0 || curPoint.y()<0 || curPoint.x()> width-1 || curPoint.y()> height-1) break;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show determineIfAllWhite Points"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            p->draw(curPoint.x(), curPoint.y(), VisionPainter::PURPLE);
        }
#endif // RFFD_USES_VATNAO
        if(region.getPixelColour(curPoint.x(), curPoint.y()) == cWHITE) ++distance;
        else return false;
    }while(distance + 1 < length);

    return true;
}

/*
Determines and labels a region if is T Intersectionn
*/
void RegionFieldFeatureDetector::determineIfTIntersection_(const VisionInfoIn& info_in,
        VisionInfoMiddle& info_middle, const RegionI& region, FieldFeatureRegionData& data)
{
    if(data.regionEnds.size() != 3) return;
    Point point1 = data.regionEnds[0];
    Point point2 = data.regionEnds[1];
    Point point3 = data.regionEnds[2];

    bool whiteOneTwo = determineIfAllWhite(region, point1, point2);
    bool whiteOneThree = determineIfAllWhite(region, point1, point3);
    bool whiteTwoThree = determineIfAllWhite(region, point2, point3);

    Point shoulder1;
    Point shoulder2;
    Point tail;

    if(whiteOneTwo && !whiteOneThree && !whiteTwoThree)
    {
        shoulder1 = point1;
        shoulder2 = point2;
        tail = point3;
    }
    else if (!whiteOneTwo && whiteOneThree && !whiteTwoThree)
    {
        shoulder1 = point1;
        shoulder2 = point3;
        tail = point2;
    }
    else if (!whiteOneTwo && !whiteOneThree && whiteTwoThree)
    {
        shoulder1 = point3;
        shoulder2 = point2;
        tail = point1;
    }
    else
    {
        data.isTConfidence = 0.0f;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection failed1"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return;
    }

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection:"<< std::endl;
            vdm->msg << "Shoulder1: ("<<shoulder1.x()<<", "<<shoulder1.y()<<")"<<std::endl;
            vdm->msg << "Shoulder2: ("<<shoulder2.x()<<", "<<shoulder2.y()<<")"<<std::endl;
            vdm->msg << "Tail: ("<<tail.x()<<", "<<tail.y()<<")"<<std::endl;

        }
#endif // RFFD_USES_VATNAO

    // length of the segment shoulder
    float length = std::sqrt((float)( (shoulder1.x()-shoulder2.x())*(shoulder1.x()-shoulder2.x())
                                + (shoulder1.y()-shoulder2.y())*(shoulder1.y()-shoulder2.y()) ));
    float distance = 1.0f;
    Point curPoint;
    // The width and height.
    int width = region.getCols();
    int height = region.getRows();

    bool isT = false;
    std::vector<Point> availablePoints;
    availablePoints.reserve(length);

    while(distance + 1 < length)
    {
        curPoint.x() = shoulder1.x()*(length-distance)/length + shoulder2.x()*distance/length;
        curPoint.y() = shoulder1.y()*(length-distance)/length + shoulder2.y()*distance/length;
        if(curPoint.x()<0 || curPoint.y()<0 || curPoint.x()> width-1 || curPoint.y()> height-1) break;
        if(determineIfAllWhite(region, curPoint, tail))
        {
            data.isTConfidence = 1.0f;
            isT = true;
            availablePoints.push_back(curPoint);
        }
        ++distance;
    }

    if(!isT)
    {
        data.isTConfidence = 0.0f;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection failed2"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return;
    }

    Point intersection = Point(0, 0);

    for (unsigned int i = 0; i < availablePoints.size(); ++i)
    {
        intersection = availablePoints[i] + intersection;
    }
    intersection = intersection/availablePoints.size();

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show T Regions"] == "true"
        && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << "TIntersection: ("<<intersection.x()<<", "<<intersection.y()<<")"<<std::endl;
        p->draw(intersection.x(), intersection.y(), VisionPainter::ORANGE);
    }
#endif // RFFD_USES_VATNAO

    std::vector<Point, Eigen::aligned_allocator<Point> > three_points_1;
    three_points_1.push_back(shoulder1);
    three_points_1.push_back(intersection);
    three_points_1.push_back(tail);
    if(!cornerShapeCheck(info_in, info_middle, region, three_points_1))
    {
        data.isTConfidence = 0.0f;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection failed 3"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return;
    }

    std::vector<Point, Eigen::aligned_allocator<Point> > three_points_2;
    three_points_2.push_back(shoulder2);
    three_points_2.push_back(intersection);
    three_points_2.push_back(tail);
    if(!cornerShapeCheck(info_in, info_middle, region, three_points_2))
    {
        data.isTConfidence = 0.0f;
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection failed 4"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return;
    }

    // Get the relative density of the raw image.
    int density = region.isTopCamera() ? TOP_SALIENCY_DENSITY : BOT_SALIENCY_DENSITY;

    intersection = intersection * density + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera()) intersection.y() += TOP_IMAGE_ROWS;

    tail = tail * density + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera()) tail.y() += TOP_IMAGE_ROWS;

    intersection = info_in.cameraToRR.pose.imageToRobotXY(intersection);
    tail = info_in.cameraToRR.pose.imageToRobotXY(tail);

    RANSACLine rl(tail, intersection);

    info_middle.potentialTIntersections.push_back(intersection);
    info_middle.potentialTIntersectionAngles.push_back(findTAngle_(intersection, rl));
    data.intersection = intersection;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show T Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "T detection success"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

}

/*
Determines and labels a region a line, curve, corner or none.
*/
void RegionFieldFeatureDetector::determineIfLineCurveCorner_(
        const VisionInfoIn& info_in, VisionInfoMiddle& info_middle,
        const RegionI& region, FieldFeatureRegionData& data)
{
    // Lines, curves and corners are all lines
#ifdef FIELD_FEATURE_TIMINGS
    lineCheckTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    data.isLine = performLineChecks_(info_in, info_middle, region, data);
#ifdef FIELD_FEATURE_TIMINGS
    lineCheckTime += lineCheckTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS

    // If this isn't a line, don't bother with the other checks.
    if(!data.isLine)
        return;

    std::pair<Point, Point> regionPoints(
        data.regionEnds[0],
        data.regionEnds[1]);

    std::pair<Point, Point> fieldPoints(
        data.fieldEnds[0],
        data.fieldEnds[1]);

    data.lineEnds = fieldPoints;

#ifdef FIELD_FEATURE_TIMINGS
    curveDetectorTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    bool isCurve = determineIfCurve_(info_middle, region, data, regionPoints,
                                                                   fieldPoints);
#ifdef FIELD_FEATURE_TIMINGS
    curveDetectorTime += curveDetectorTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if(isCurve)
        return;

#ifdef FIELD_FEATURE_TIMINGS
    cornerDetectorTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    bool isCorner = determineIfCorner_(info_in, info_middle, region, data,
                                                     regionPoints, fieldPoints);
#ifdef FIELD_FEATURE_TIMINGS
    cornerDetectorTime += cornerDetectorTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if(isCorner)
        return;

#ifdef RFFD_USES_VATNAO
    // Otherwise, its a line. Paint lines in topPainter and botPainter
    if (vdm != NULL && q.options["Show Line Regions"] == "true")
    {
        BBox boundingBox = region.getBoundingBoxRaw();
        int density = region.getDensity();

        Point regionEndFrameSaliencyA = density * data.regionEnds[0] +
                                                                  boundingBox.a;
        Point regionEndFrameSaliencyB = density * data.regionEnds[1] +
                                                                  boundingBox.a;

        if (region.isTopCamera())
            topPainter->drawLine(regionEndFrameSaliencyA.x()/5,
                regionEndFrameSaliencyA.y()/5, regionEndFrameSaliencyB.x()/5,
                           regionEndFrameSaliencyB.y()/5, VisionPainter::WHITE);
        else
            botPainter->drawLine(regionEndFrameSaliencyA.x()/5,
                regionEndFrameSaliencyA.y()/5, regionEndFrameSaliencyB.x()/5,
                           regionEndFrameSaliencyB.y()/5, VisionPainter::WHITE);
    }
#endif // RFFD_USES_VATNAO
}

/*
Checks that a region containing two ends really is a line (straight or
otherwise).
*/
bool RegionFieldFeatureDetector::performLineChecks_(
        const VisionInfoIn& info_in, VisionInfoMiddle& info_middle,
        const RegionI& region, FieldFeatureRegionData& data)
{
    // Whether the checks have been passed.
    bool isLine = true;

    // Determine if the field relative sizes of the region ends is signifcantly
    // different.
    //isLine = checkRelEndSize_(info_in, region, data.regionEndXYPairs[0],
    //                                                  data.regionEndXYPairs[1]);

    // Early exit.
    if(!isLine)
        return(isLine);

    // Check the two line ends are connected.
    isLine = checkConnected_(region, data.regionEndXYPairs[0],
                                                      data.regionEndXYPairs[1]);

    // Early exit.
    if(!isLine)
        return(isLine);

    // Return whether this region has passed the line checks.
    return(isLine);
}

/*
Determine if the field relative sizes of the region ends is signifcantly
different.
*/
bool RegionFieldFeatureDetector::checkRelEndSize_(const VisionInfoIn& info_in,
    const RegionI& region, const std::pair<Point, Point> end1,
                                             const std::pair<Point, Point> end2)
{
    // The end points in field coordinates.
    Point end1A, end1B, end2A, end2B;

    // The distances between the end points.
    float end1Size, end2Size;

    // The ratio largerEnd / smallerEnd.
    float sizeRatio;

    // Convert from image to field coordinates.
    end1A = end1.first * region.getDensity() + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera())
        end1A.y() += TOP_IMAGE_ROWS;
    end1A = info_in.cameraToRR.pose.imageToRobotXY(end1A);
    end1B = end1.second * region.getDensity() + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera())
        end1B.y() += TOP_IMAGE_ROWS;
    end1B = info_in.cameraToRR.pose.imageToRobotXY(end1B);
    end2A = end2.first * region.getDensity() + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera())
        end2A.y() += TOP_IMAGE_ROWS;
    end2A = info_in.cameraToRR.pose.imageToRobotXY(end2A);
    end2B = end2.second * region.getDensity() + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera())
        end2B.y() += TOP_IMAGE_ROWS;
    end2B = info_in.cameraToRR.pose.imageToRobotXY(end2B);

    // Calculate the field relative size of each end.
    end1Size = DISTANCE(end1A, end1B);
    end2Size = DISTANCE(end2A, end2B);

    // Calculate the relative end sizes.
    if(end1Size > end2Size)
        sizeRatio = end1Size/end2Size;
    else
        sizeRatio = end2Size/end1Size;

    // Return whether the ratio is too large.
    return(sizeRatio < MAX_LINE_END_SIZE_RATIO);
}

/*
Checks if two edge points are connected by white pixels. Also records the pixels
along the edge.
*/
bool RegionFieldFeatureDetector::checkConnected_(const RegionI& region,
                         std::pair<Point, Point>& a, std::pair<Point, Point>& b)
{
    // Whether the points are connected.
    bool connected = false;

    // The region width and height.
    int width = region.getCols();
    int height = region.getRows();

    // The last pixel intersection.
    Point lastIntersection = a.first;

    // The start and finish intersection for the search.
    Point startIntersection = a.first;
    Point endIntersection = b.second;

    // Check that both the start and end points are actually on white pixels.
    if(region.getPixelColour(startIntersection[0], startIntersection[1]) !=
            cWHITE || region.getPixelColour(endIntersection[0],
                                                  endIntersection[1]) != cWHITE)
        return(false);

    // Adjust the starting point to account for the difference between
    // intersection and pixel coordinates and place the last intersection at the
    // counterclockwise intersection from the current intersection, with both on
    // the region boundary.
    if(a.first.x() == 0)
        ++lastIntersection[1];
    else if(a.first.y() == 0)
        ++startIntersection[0];
    else if(a.first.x() == width-1)
    {
        ++startIntersection[0];
        ++startIntersection[1];
        ++lastIntersection[0];
    }
    else
    {
        ++startIntersection[0];
        ++startIntersection[1];
        ++lastIntersection[1];
    }

    // Ensure the end point is on a border.
    if(b.second.x() == width-1)
        ++endIntersection[0];
    if(b.second.y() == height-1)
        ++endIntersection[1];


    // Try to find a connection between the start and end of the first edge.
    connected = buildEdge_(region, lastIntersection, startIntersection,
                                                    endIntersection, firstEdge);

    // If the ends are connected, build the other edge.
    if(connected)
    {
        // Get the new last, start and end intersections.
        lastIntersection = a.first;
        startIntersection = a.first;
        endIntersection = b.second;

        // Check that both the start and end are on white pixels.
        // TODO: COME UP WITH SOMETHING TO DEAL WITH THIS.
        if(region.getPixelColour(startIntersection[0], startIntersection[1]) !=
                cWHITE || region.getPixelColour(endIntersection[0],
                                                  endIntersection[1]) != cWHITE)
            return(false);

        // Adjust the starting point to account for the difference between
        // intersection and pixel coordinates and place the last intersection at
        // the clockwise intersection from the current intersection, with both
        // on the region boundary.
        if(a.first.x() == 0)
            ++startIntersection[1];
        else if(a.first.y() == 0)
            ++lastIntersection[0];
        else if(a.first.x() == width-1)
        {
            ++lastIntersection[0];
            ++lastIntersection[1];
            ++startIntersection[0];
        }
        else
        {
            ++lastIntersection[0];
            ++lastIntersection[1];
            ++startIntersection[1];
        }

        // Build the edge.
        buildEdge_(region, lastIntersection, startIntersection, endIntersection,
                                                                    secondEdge);
    }

#ifdef RFFD_USES_VATNAO
    // Show linearAnalysisPoints for linearAnalysis
    if(connected && vdm != NULL && q.options["Show firstEdge Points"] ==
         "true" && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        for (unsigned int i = 0; i < firstEdge.size(); ++i){
            p->draw(firstEdge[i].x(), firstEdge[i].y(), VisionPainter::PURPLE);
        }
    }
    if(connected && vdm != NULL && q.options["Show secondEdge Points"] ==
         "true" && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        for (unsigned int i = 0; i < secondEdge.size(); ++i){
            p->draw(secondEdge[i].x(), secondEdge[i].y(), VisionPainter::PURPLE);
        }
    }    
#endif // RFFD_USES_VATNAO

    // Return whether the edge was connected.
    return(connected);
}

/*
Builds an edge from startIntersection to endIntersection and stores the pixels
that compose it in edgePixels.
*/
bool RegionFieldFeatureDetector::buildEdge_(const RegionI& region,
    Point lastIntersection, Point startIntersection, Point endIntersection,
               std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints)
{
    // Whether the points are connected.
    bool connected = false;

    // The region width and height.
    int width = region.getCols();
    int height = region.getRows();

    // The current pixel intersection.
    Point curIntersection = startIntersection;;

    // Make space for the edge points.
    edgePoints.clear();

    // Search until we reach the end or return to the start.
    do
    {
        // Whether the candidate has been selected.
        bool candidateSelected = false;

        // The next candidate to consider.
        Point candidate = lastIntersection;

        // Check which of the three candidates is the next point along the edge.
        while(!candidateSelected)
        {
            // Rotate the candidate clockwise.
            do
            {
                // Up to right.
                if(candidate[1] < curIntersection[1])
                {
                    ++candidate[0];
                    ++candidate[1];
                }

                // Right to down.
                else if(candidate[0] > curIntersection[0])
                {
                    --candidate[0];
                    ++candidate[1];
                }

                // Down to left.
                else if(candidate[1] > curIntersection[1])
                {
                    --candidate[0];
                    --candidate[1];
                }

                // Left to up.
                else
                {
                    ++candidate[0];
                    --candidate[1];
                }
            }
            while(candidate[0] < 0 || candidate[1] < 0 || candidate[0] > width
                                                      || candidate[1] > height);

            // Check if this candidate is along an edge.

            // Horizontal edge.
            if(candidate[0] != curIntersection[0])
            {
                // The x value of the pixels to be checked.
                int x;

                // Whether each of the relevant pixels is white.
                bool upperWhite, lowerWhite;

                // Determine the x value of the pixels to be checked.
                if(curIntersection[0] < candidate[0])
                    x = curIntersection[0];
                else
                    x = curIntersection[0] - 1;

                // Check the pixels.
                if(curIntersection[1] == 0)
                    upperWhite = false;
                else
                {
                    upperWhite = region.getPixelColour(x, curIntersection[1]-1)
                                                                      == cWHITE;
                }
                if(curIntersection[1] == height)
                    lowerWhite = false;
                else
                {
                    lowerWhite = region.getPixelColour(x, curIntersection[1]) ==
                                                                         cWHITE;
                }

                // If appropriate, record the pixel and move to the next
                // intersection.
                if((upperWhite || lowerWhite) && lowerWhite != upperWhite)
                {
                    // Move to the next intersection.
                    lastIntersection = curIntersection;
                    curIntersection = candidate;
                    candidateSelected = true;

                    // Add the pixel. Lazy, will create duplicates.
                    if(curIntersection[1] != 0 && curIntersection[1] != height)
                    {
                        if(lowerWhite)
                            edgePoints.push_back(Point(x, curIntersection[1]));
                        else
                            edgePoints.push_back(Point(x, curIntersection[1]-1));
                    }
                }
            }
            // Vertical edge.
            else
            {
                // The y value of the pixels to be checked.
                int y;

                // Whether each of the relevant pixels is white.
                bool leftWhite, rightWhite;

                // Determine the y value of the pixels to be checked.
                if(curIntersection[1] < candidate[1])
                    y = curIntersection[1];
                else
                    y = curIntersection[1] - 1;

                // Check the pixels.
                if(curIntersection[0] == 0)
                    leftWhite = false;
                else
                {
                    leftWhite = region.getPixelColour(curIntersection[0]-1, y)
                                                                      == cWHITE;
                }
                if(curIntersection[0] == width)
                    rightWhite = false;
                else
                {
                    rightWhite = region.getPixelColour(curIntersection[0], y) ==
                                                                         cWHITE;
                }

                // If appropriate, record the pixel and move to the next
                // intersection.
                if((rightWhite || leftWhite) && rightWhite != leftWhite)
                {
                    // Move to the next intersection.
                    lastIntersection = curIntersection;
                    curIntersection = candidate;
                    candidateSelected = true;

                    // Add the pixel. Lazy, will create duplicates.
                    if(curIntersection[0] != 0 && curIntersection[0] != width)
                    {
                        if(rightWhite)
                            edgePoints.push_back(Point(curIntersection[0], y));
                        else
                            edgePoints.push_back(Point(curIntersection[0]-1, y));
                    }
                }
            }
        }
    }
    while((curIntersection[0] != startIntersection[0] || curIntersection[1] !=
        startIntersection[1]) && (curIntersection[0] != endIntersection[0] ||
                                     curIntersection[1] != endIntersection[1]));

    // Check if the end was reached.
    if(curIntersection[0] == endIntersection[0] && curIntersection[1] ==
                                                             endIntersection[1])
        connected = true;

    // Return whether the border points were connected.
    return(connected);
}

/*
Project regionEnds to fieldEnds using cameraToRR
*/
void RegionFieldFeatureDetector::projectRegionEndsToFieldEnds_(
    const VisionInfoIn& info_in,
    const RegionI& region,
    FieldFeatureRegionData& data)
{
    // Get the relative density of the raw image.
    int density = region.isTopCamera() ? TOP_SALIENCY_DENSITY :
                                                    BOT_SALIENCY_DENSITY;

    for (unsigned int i = 0; i < data.regionEnds.size(); ++i)
    {
        Point regionEndGlobal =
            data.regionEnds[i] * density + region.getBoundingBoxRaw().a;
        if (!region.isTopCamera())
            regionEndGlobal.y() += TOP_IMAGE_ROWS;
        data.fieldEnds.push_back(
            info_in.cameraToRR.pose.imageToRobotXY(regionEndGlobal));
    }
}

/*
Determines whether the region is a curve or not
*/
bool RegionFieldFeatureDetector::determineIfCurve_(
    VisionInfoMiddle& info_middle,
    const RegionI& region,
    FieldFeatureRegionData& data,
    const std::pair<Point, Point>& regionPoints,
    const std::pair<Point, Point>& fieldPoints)
{
    // Whether each check was passed.
    bool passedCheck;

    // If a line is too short, can't tell if its a curve.
    if (DISTANCE_SQR(data.lineEnds.first, data.lineEnds.second) < MIN_CURVE_LENGTH)
    {

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Curve Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not curve: length < MIN_CURVE_LENGTH" <<std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }


    if (region.getRows() < 8){

#ifdef RFFD_USES_VATNAO
        // Region too short to analyse, so assume it isn't a curve.
        if (vdm != NULL && q.options["Show Curve Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not curve: region too short" <<std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }


    if (region.getCols() < 8){

#ifdef RFFD_USES_VATNAO
        // Region too narrow to analyse, so assume it isn't a curve.
        if (vdm != NULL && q.options["Show Curve Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not curve: region too narrow" <<std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }

    // Check if line passes the Centre Point Offset Analysis
#ifdef FIELD_FEATURE_TIMINGS
    centrePointOffsetAnalysisCurveTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    passedCheck = !centrePointOffsetAnalysis_(region, regionPoints);
#ifdef FIELD_FEATURE_TIMINGS
    centrePointOffsetAnalysisCurveTime +=
                               centrePointOffsetAnalysisCurveTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if (passedCheck){

#ifdef RFFD_USES_VATNAO
        // Notify offset analysis fail
        if (vdm != NULL && q.options["Show Curve Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not curve: failed centrePointOffsetAnalysis" <<std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }

#ifdef FIELD_FEATURE_TIMINGS
    linearAnalysisTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    passedCheck = !linearAnalysis_(region);
#ifdef FIELD_FEATURE_TIMINGS
    linearAnalysisTime += linearAnalysisTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if (passedCheck){

#ifdef RFFD_USES_VATNAO
        // Notify linear analysis fail
        if (vdm != NULL && q.options["Show Curve Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not curve: failed linearAnalysis" <<std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }

    data.isCurveConfidence = 1.0f;

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Curve Regions"] == "true"
        && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << data.neighborRegionIDs.size()<< " neighbors all curves"<< std::endl;
    }
#endif // RFFD_USES_VATNAO

    // for(unsigned int i=0; i<data.neighborRegionIDs.size(); ++i)
    // {
    //     info_middle.fieldFeatureRegionData[data.neighborRegionIDs[i]].isCurveConfidence = 1.0f;
    // }

    return true;
}


/*
Determines whether the centre point offset from the centre of the line
is great. Returns True if the offset is large, False if not.
*/
bool RegionFieldFeatureDetector::centrePointOffsetAnalysis_(
    const RegionI& region,
    const std::pair<Point, Point>& regionPoints)
{
    const Point startPoint = regionPoints.first;
    const Point endPoint = regionPoints.second;

    // Calculate the line equation.
    RANSACLine baseLine(startPoint, endPoint);
    RANSACLine perpendicularLine;

    // The width and height.
    int width = region.getCols();
    int height = region.getRows();

    // Calculate the line centre point.
    Point centre = (endPoint + startPoint)/2;

    // Sanity check that centre is inside the region
    if (centre.x() < 0 || centre.x() >= width || centre.y() < 0 || centre.y() >= height) {
        return false;
    }
    // The current point being considered.
    Point curPoint = centre;

    // The extremes of the lines.
    int leftOffset = -1, rightOffset = -1;

    // Create the perpendicular line equation.
    perpendicularLine.t1 = baseLine.t2;
    perpendicularLine.t2 = -baseLine.t1;
    perpendicularLine.sqrtt1t2 = baseLine.sqrtt1t2;

    // Check if the centre point of the line is accurate or offset.

    // Calculate t3 of the perpendicular line at the centre point.
    perpendicularLine.t3 = - perpendicularLine.t1*centre.x() -
                                                perpendicularLine.t2*centre.y();

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true"
        && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << std::endl;
        vdm->msg << "StartPoint:("<<startPoint.x()<<","<<startPoint.y()
                    <<") EndPoint:("<<endPoint.x()<<","<<endPoint.y()<<")"<<std::endl;
        vdm->msg << baseLine.t1<<"x + "<<baseLine.t2<<"y + "<<baseLine.t3<<" = 0"<<std::endl;
        vdm->msg << perpendicularLine.t1<<"x + "<<perpendicularLine.t2<<"y + "<<perpendicularLine.t3<<" = 0" << std::endl;
    }
#endif // RFFD_USES_VATNAO

    // Starting from centre move out looking for the line edges.

    // Moving left.
    bool goingUp = (perpendicularLine.t2 == 0) ||
                             (-perpendicularLine.t1 / perpendicularLine.t2) > 0;
    int offset = 0;
    while(leftOffset == -1)
    {
        // The y value for this column.
        int colY = curPoint.y();
        if(perpendicularLine.t2 != 0)
        {
            colY = (-perpendicularLine.t1 * curPoint.x() - perpendicularLine.t3)
                                                         / perpendicularLine.t2;
        }

        // If this pixel isn't white the edge has been found.
        if(region.getPixelColour(curPoint.x(), curPoint.y()) != cWHITE)
            leftOffset = offset;

        // Otherwise determine the next pixel.
        else
        {
            if(goingUp)
            {
                ++offset;
                if(curPoint.y() > colY)
                    --curPoint.y();
                else
                    --curPoint.x();

                // Check if the edge of the region has been reached.
                if(curPoint.x() < 0 || curPoint.y() < 0)
                    break;
            }
            else
            {
                ++offset;
                if(curPoint.y() < colY)
                    ++curPoint.y();
                else
                    --curPoint.x();

                // Check if the edge of the region has been reached.
                if(curPoint.x() < 0 || curPoint.y() >= height)
                    break;
            }
        }

    }

#ifdef RFFD_USES_VATNAO
    // Record LeftPoint
    Point leftPoint = curPoint;
#endif // RFFD_USES_VATNAO


    // Moving right.
    curPoint = centre;
    offset = 0;
    while(rightOffset == -1)
    {
        // The y value for this column.
        int colY = curPoint.y();
        if(perpendicularLine.t2 != 0)
        {
            colY = (-perpendicularLine.t1 * curPoint.x() - perpendicularLine.t3)
                                                         / perpendicularLine.t2;
        }

        // If this pixel isn't white the edge has been found.
        if(region.getPixelColour(curPoint.x(), curPoint.y()) != cWHITE)
            rightOffset = offset;

        // Otherwise determine the next pixel.
        else
        {
            if(goingUp)
            {
                ++offset;
                if(curPoint.y() > colY)
                    ++curPoint.x();
                else
                    ++curPoint.y();

                // Check if the edge of the region has been reached.
                if(curPoint.x() >= width || curPoint.y() >= height)
                    break;
            }
            else
            {
                ++offset;
                if(curPoint.y() < colY)
                    ++curPoint.x();
                else
                    --curPoint.y();

                // Check if the edge of the region has been reached.
                if(curPoint.x() >= width || curPoint.y() < 0)
                    break;
            }
        }
    }


#ifdef RFFD_USES_VATNAO
    // Record RightPoint
    Point rightPoint = curPoint;
#endif // RFFD_USES_VATNAO


#ifdef RFFD_USES_VATNAO
    // Show Centre Point Offset Analysis
    if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        p->draw(leftPoint.x(), leftPoint.y(), VisionPainter::GREY);
        p->draw(rightPoint.x(), rightPoint.y(), VisionPainter::GREY);
        p->draw(centre.x(), centre.y(), VisionPainter::ORANGE);
    }
#endif // RFFD_USES_VATNAO

    // Determine if the line is curved.

    // First pixel isn't white or both sides go over the edge.
    int diff = abs(leftOffset - rightOffset);
    diff += diff/2;

#ifdef RFFD_USES_VATNAO
    // Print Left, Right, leftOffset and rightOffset
    if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << "Left: " << (float)diff/(float)leftOffset << " Right: "
                                << (float)diff/(float)rightOffset << std::endl;
        vdm->msg << "leftOffset: " << leftOffset << " rightOffset: "
                                << rightOffset << std::endl;
    }
#endif // RFFD_USES_VATNAO

    if(leftOffset == 0 || rightOffset == 0)
    {

#ifdef RFFD_USES_VATNAO
        // centrePoint doesn't lie on line
        if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "centrePointOffsetAnalysis_(TRUE): "
                     << "centrePoint doesn't lie on line"
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return true;
    }
    else if((leftOffset == -1 || rightOffset == -1) &&
                                 !(width>=MAX_CORNER_WIDTH_HEIGHT_RATIO*height))
    {
#ifdef RFFD_USES_VATNAO
        // Reached end of region, couldn't calculate offsets
        if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "centrePointOffsetAnalysis_(FALSE): "
                     << "Reached end of region, couldn't calculate offsets"
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }
    else if(diff / leftOffset > 1 || diff / rightOffset > 1)
    {

#ifdef RFFD_USES_VATNAO
        // The left and right offsets differ greatly
        if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "centrePointOffsetAnalysis_(TRUE): "
                     << "The left and right offsets differ greatly"
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return true;
    }
    else
    {

#ifdef RFFD_USES_VATNAO
        // Otherwise this is a line.
        if (vdm != NULL && q.options["Show Centre Point Offset Analysis"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "centrePointOffsetAnalysis_(FALSE): "
                     << "else - didn't meet conditions for offset analysis"
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }
}

/*
Doing linear regression of border points near startPoint and endPoint
respectively.
*/
bool RegionFieldFeatureDetector::linearAnalysis_(const RegionI& region)
{
    return checkInTwoParts_(region, firstEdge);
}

/*
Check that linear analysis points don't lie on any lines.
*/
bool RegionFieldFeatureDetector::checkInTwoParts_(
    const RegionI& region,
    std::vector<Point, Eigen::aligned_allocator<Point> >& linearAnalysisPoints)
{
    int borderLength = 2*region.getCols() + 2*region.getRows() -4;

    float straightLineError = std::max(1.5f, (float)borderLength/140);

    if(linearAnalysisPoints.size()<7){

#ifdef RFFD_USES_VATNAO
        // Not enough linear analysis points
        if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "checkInTwoParts_(FALSE): Not enough linearAnalysisPoints - "
                     << linearAnalysisPoints.size()
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }

    Point head = linearAnalysisPoints.front();
    Point tail = linearAnalysisPoints.back();
    int midPoint = linearAnalysisPoints.size()/2;
    Point mid = linearAnalysisPoints[midPoint];

#ifdef RFFD_USES_VATNAO
    // Print HeadPoint, MidPoint and TailPoint
    if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << "straightLineError: " << straightLineError
                 << " HeadPoint:("<<head.x()<<","<<head.y()
                 <<") MidPoint:("<<mid.x()<<","<<mid.y()<<")"
                 <<") TailPoint:("<<tail.x()<<","<<tail.y()<<")"<<std::endl;
    }
#endif // RFFD_USES_VATNAO

#ifdef RFFD_USES_VATNAO
    // Show lines for linearAnalysis
    if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        p->drawLine(head.x(), head.y(), mid.x(), mid.y(), VisionPainter::RED);
        p->drawLine(tail.x(), tail.y(), mid.x(), mid.y(), VisionPainter::RED);
    }
#endif // RFFD_USES_VATNAO

    int t1 = mid.y() - head.y();
    int t2 = head.x() - mid.x();
    int t3 = mid.x()*head.y()- head.x()*mid.y();

    bool upStraight = true;

    for(int pointID=1; pointID < midPoint; ++pointID)
    {
        if(((double)(std::abs(linearAnalysisPoints[pointID].x()*t1 +
            linearAnalysisPoints[pointID].y()*t2 +t3))/(double)(std::sqrt(t1*t1 + t2*t2)))
                                                                        > straightLineError)
        {
            upStraight = false;

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << "Point: ("<<linearAnalysisPoints[pointID].x()<<", "
                <<linearAnalysisPoints[pointID].y()<<") is not on the line"<<std::endl;
        p->draw(linearAnalysisPoints[pointID].x(), linearAnalysisPoints[pointID].y(), VisionPainter::BLACK);
    }
#endif
            break;
        }
    }

    t1 = mid.y() - tail.y();
    t2 = tail.x() - mid.x();
    t3 = mid.x()*tail.y()- tail.x()*mid.y();

    bool downStraight = true;

    for(unsigned int pointID=midPoint+1; pointID<linearAnalysisPoints.size(); ++pointID)
    {
        if((double)(std::abs(linearAnalysisPoints[pointID].x()*t1 +
            linearAnalysisPoints[pointID].y()*t2 + t3))/(double)(std::sqrt(t1*t1 + t2*t2))
                                                                        > straightLineError)
        {
            downStraight = false;
#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
            vdm->msg << "Point: ("<<linearAnalysisPoints[pointID].x()<<", "
                    <<linearAnalysisPoints[pointID].y()<<") is not on the line"<<std::endl;
            p->draw(linearAnalysisPoints[pointID].x(), linearAnalysisPoints[pointID].y(), VisionPainter::BLACK);
    }        
#endif
            break;
        }
    }

    if(upStraight || downStraight) {

#ifdef RFFD_USES_VATNAO
        // Linear analysis points match up with line.
        if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "checkInTwoParts_(FALSE): Linear analysis points match up with line"
                     << std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }


#ifdef RFFD_USES_VATNAO
    // Otherwise, linear analysis points don't match with lines"
    if (vdm != NULL && q.options["Show Linear Analysis Lines"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        vdm->msg << "checkInTwoParts_(TRUE): Linear analysis points don't match with lines"
                    << std::endl;
    }
#endif // RFFD_USES_VATNAO

    return true;
}

/*
Check if the two lines forming the corner intersect at a minimum angle to form a
corner, and all edgePoints lie on the two lines
*/
bool RegionFieldFeatureDetector::cornerShapeCheck(const VisionInfoIn& info_in,
    VisionInfoMiddle& info_middle,const RegionI& region,
         const std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints)
{
    // Placeholder for unused return.
    Point tip;

    // Call the real function.
    return(cornerShapeCheck(info_in, info_middle, region, edgePoints, tip));
}

/*
Check if the two lines forming the corner intersect at a minimum angle to form a
corner, and all edgePoints lie on the two lines
*/
bool RegionFieldFeatureDetector::cornerShapeCheck(const VisionInfoIn& info_in,
    VisionInfoMiddle& info_middle,const RegionI& region,
    const std::vector<Point, Eigen::aligned_allocator<Point> >& edgePoints,
                                                                     Point& tip)
{
    // The first and last point along the edge.
    Point head = edgePoints[0];
    Point tail = edgePoints[edgePoints.size()-1];

    // The ID of the tip point.
    int tipID = -1;

    // The distance score of the current tip point.
    int maxTipDistanceScore = 0;

    // The weighting factors for the distance score equation.
    int xWeight = tail.y()-head.y();
    int yWeight = tail.x()-head.x();
    int constant = tail.x()*head.y() - tail.y()*head.x();

    // Locate the tip of the corner. It is the point farthest from the line
    // between head and tail, maximising abs(xWeight*point.x() -
    // yWeight*point.y() + constant).
    for(unsigned int pointID=0; pointID<edgePoints.size(); ++pointID)
    {
        // Calculate the "distance score" of this point (i.e. the component of
        // the point line distance equation that changes when the point
        // changes).
        int tipDistanceScore = abs(xWeight*edgePoints[pointID].x() -
                                    yWeight*edgePoints[pointID].y() + constant);

        // If this point is more distant than the current tip, replace the
        // current tip.
        if(tipDistanceScore > maxTipDistanceScore)
        {
            maxTipDistanceScore = tipDistanceScore;
            tip = edgePoints[pointID];
            tipID = pointID;
        }
    }

#ifdef RFFD_USES_VATNAO
    if (vdm != NULL && q.options["Show Corner Shape Check"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        p->draw(head.x(), head.y(), VisionPainter::PINK);
        p->draw(tail.x(), tail.y(), VisionPainter::BLUE);
        p->draw(tip.x(), tip.y(), VisionPainter::PINK);
    }
#endif // RFFD_USES_VATNAO

    /*
    Check that angle between two lines are greater than minimum angle
    */
    RANSACLine line1(head, tip);
    RANSACLine line2(tip, tail);

    float angle1 = line1.getAngle();
    float angle2 = line2.getAngle();

    if(angle1 < 0.0f) angle1 += M_PI;
    if(angle2 < 0.0f) angle2 += M_PI;

    float angleBetween;

    if(angle1 > angle2)
        angleBetween = angle1 - angle2;
    else
        angleBetween = angle2 - angle1;

    if (angleBetween > M_PI/2)
        angleBetween = M_PI - angleBetween;

    if(angleBetween <= MIN_CORNER_ANGLE_IN_IMAGE)
    {
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not corner: failed cornerShapeCheck - angle of corner below MIN_CORNER_ANGLE_IN_IMAGE"
                     << "\n angleBetween(deg): " << RAD2DEG(angleBetween) << std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }

#ifdef RFFD_USES_VATNAO
    // Show lines for linearAnalysis
    if (vdm != NULL && q.options["Show Corner Shape Check"] == "true" &&
        vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
    {
        p->drawLine(head.x(), head.y(), tip.x(), tip.y(), VisionPainter::YELLOW);
        p->drawLine(tail.x(), tail.y(), tip.x(), tip.y(), VisionPainter::YELLOW);

    }
#endif // RFFD_USES_VATNAO


    /*
    Check that edgePoints are close to head->tip->tail
    */

    // Check points between head and tip
    int t1 = tip.y() - head.y();
    int t2 = head.x() - tip.x();
    int t3 = tip.x()*head.y()- head.x()*tip.y();

    for(int i=0; i < tipID; ++i)
    {
        if(((double)(std::abs(edgePoints[i].x()*t1 +
            edgePoints[i].y()*t2 +t3))/(double)(std::sqrt(t1*t1 + t2*t2)))
                                                        > CORNER_POINT_TO_LINE_DISTANCE_MAX)
        {
#ifdef RFFD_USES_VATNAO
            if (vdm != NULL && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
            {
                vdm->msg << "Not corner: failed cornerShapeCheck - pointToLineDistance above limit between head and tip" <<std::endl;
            }
#endif // RFFD_USES_VATNAO
            return false;
        }
    }

    // Check points between tail and tip
    t1 = tip.y() - tail.y();
    t2 = tail.x() - tip.x();
    t3 = tip.x()*tail.y()- tail.x()*tip.y();

    for(unsigned int i=tipID+1; i<edgePoints.size(); ++i)
    {
        if((double)(std::abs(edgePoints[i].x()*t1 +
            edgePoints[i].y()*t2 + t3))/(double)(std::sqrt(t1*t1 + t2*t2))
                                                        > CORNER_POINT_TO_LINE_DISTANCE_MAX)
        {
#ifdef RFFD_USES_VATNAO
            if (vdm != NULL && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
            {
                vdm->msg << "Not corner: failed cornerShapeCheck - pointToLineDistance above limit between tail and tip" <<std::endl;
            }
#endif // RFFD_USES_VATNAO
            return false;
        }
    }

    return true;
}

/*
Determines whether the region is a corner or not
*/
bool RegionFieldFeatureDetector::determineIfCorner_(const VisionInfoIn& info_in,
    VisionInfoMiddle& info_middle, const RegionI& region, FieldFeatureRegionData& data,
    const std::pair<Point, Point>& regionPoints, const std::pair<Point, Point>& fieldPoints)
{
    // Whether each check was passed.
    bool passedCheck;

    // Check if line passes the Centre Point Offset Analysis
#ifdef FIELD_FEATURE_TIMINGS
    centrePointOffsetAnalysisCornerTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    passedCheck = !centrePointOffsetAnalysis_(region, regionPoints);
#ifdef FIELD_FEATURE_TIMINGS
    centrePointOffsetAnalysisCornerTime +=
                              centrePointOffsetAnalysisCornerTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if (passedCheck){

#ifdef RFFD_USES_VATNAO
        // Notify offset analysis fail
        if (vdm != NULL && q.options["Show Corner Regions"] == "true" &&
            vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Not corner: failed centrePointOffsetAnalysis" <<std::endl;
        }
#endif // RFFD_USES_VATNAO

        return false;
    }

    if(firstEdge.size()==0)
    {
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner detection failed as firstEdge size 0"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }

    if(secondEdge.size()==0)
    {
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner detection failed as secondEdge size 0"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }

    // The tip points of the edges.
    Point tip1, tip2;

#ifdef FIELD_FEATURE_TIMINGS
    cornerShapeCheckTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    passedCheck = !cornerShapeCheck(info_in, info_middle, region, firstEdge,
                                                                          tip1);
#ifdef FIELD_FEATURE_TIMINGS
    cornerShapeCheckTime += cornerShapeCheckTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if(passedCheck)
    {
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner detection failed as cornerShapeCheck 1 failed"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }
#ifdef FIELD_FEATURE_TIMINGS
    cornerShapeCheckTimer.restart();
#endif // FIELD_FEATURE_TIMINGS
    passedCheck = !cornerShapeCheck(info_in, info_middle, region, secondEdge,
                                                                          tip2);
#ifdef FIELD_FEATURE_TIMINGS
    cornerShapeCheckTime += cornerShapeCheckTimer.elapsed_us();
#endif // FIELD_FEATURE_TIMINGS
    if(passedCheck)
    {
#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner detection failed as cornerShapeCheck 2 failed"<< std::endl;
        }
#endif // RFFD_USES_VATNAO
        return false;
    }

    Point intersection = (tip1 + tip2)/2;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner Intersection: ("<<intersection.x()<<", "<<intersection.y()<<")"<<std::endl;
            p->draw(intersection.x(), intersection.y(), VisionPainter::ORANGE);
        }
#endif // RFFD_USES_VATNAO

    // Get the relative density of the raw image.
    int density = region.isTopCamera() ? TOP_SALIENCY_DENSITY : BOT_SALIENCY_DENSITY;

    intersection = intersection * density + region.getBoundingBoxRaw().a;
    if (!region.isTopCamera()) intersection.y() += TOP_IMAGE_ROWS;

    intersection = info_in.cameraToRR.pose.imageToRobotXY(intersection);

    info_middle.potentialCornerIntersections.push_back(intersection);

    RANSACLine l1(intersection, data.fieldEnds[0]);
    RANSACLine l2(intersection, data.fieldEnds[1]);

    info_middle.potentialCornerIntersectionAngles.push_back(findCAngle_(intersection, l1, l2));

    data.intersection = intersection;

#ifdef RFFD_USES_VATNAO
        if (vdm != NULL && q.options["Show Corner Regions"] == "true"
            && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
        {
            vdm->msg << "Corner detection success"<< std::endl;
        }
#endif // RFFD_USES_VATNAO

    data.isCornerConfidence = 1.0f;

// #ifdef RFFD_USES_VATNAO
//         if (vdm != NULL && q.options["Show Corner Regions"] == "true"
//             && vdm->vision_debug_blackboard.values["REQUESTED_REGION"] == 1)
//         {
//             vdm->msg << data.neighborRegionIDs.size()<< "Neighbor all corners"<< std::endl;
//         }
// #endif // RFFD_USES_VATNAO

    // for(unsigned int i=0; i<data.neighborRegionIDs.size(); ++i)
    // {
    //     info_middle.fieldFeatureRegionData[data.neighborRegionIDs[i]].isCornerConfidence = 1.0f;
    // }

    return true;
}

/*
Creates lines and circles from regions with distinct directions.
*/
void RegionFieldFeatureDetector::createFeatures_(const VisionInfoIn& info_in,
                         VisionInfoMiddle& info_middle, VisionInfoOut& info_out)
{
    // Storage for the current features being built. Created here to avoid
    // memory reallocations.
    std::vector<int> curFeature;
    std::vector<int> curFeature2;

    // Merge completed lines as possible.
    std::vector<bool> merged;

    // Line building.

    // Merge region line segments into full lines.
    // mergeLineRegions_(info_middle, curFeature);

    constructLines_(info_middle);

    // Merge lines into still longer lines.
    mergeLines_(info_middle, merged, curFeature);

    // Centre circle finding.

    // Find potential centre circles using region line segments.
    detectCentreCircles_(info_middle, curFeature, curFeature2);

    // Checks for centre lines through centre circles, increasing circle
    // probability if a good line is found.
    detectCentreCircleLines_(info_middle);

    // Determines which of the centre circles found is the true centre circle.
    determineBestCentreCircle_(info_middle, info_out);

    // Clean up and send valid lines.

    // Removes any lines that have been merged or are too close to the centre
    // circle.
    cleanUpAndSendLines_(info_middle, info_out, merged);

    // Corner and T finding.

    // Find line intersections so that they can be checked to see if they are
    // corners or T intersections.
    detectIntersections_(info_middle);

    // Improve intersection quality by checking the attributes of the
    // surrounding lines.
    adjustIntersectionQualityWithLines_(info_middle);

    // Improve intersection quality by checking the all-types-confidence of
    // regions containing intersections.
    adjustIntersectionQualityWithRegions_(info_middle);

    // Locate pairs of intersections that might be goal boxes and increase their
    // quality.
    goalBoxCheck_(info_middle);

    // Create Ts and corners from the intersections.
    createTsAndCorners_(info_in, info_middle, info_out);

    // Finally create penalty cross from previous detector
    createPenaltyCross_(info_in, info_middle, info_out);
}


/*
Merges line segments found in regions into longer line segments.
*/
void RegionFieldFeatureDetector::mergeLineRegions_(
                    VisionInfoMiddle& info_middle, std::vector<int>& curFeature)
{
    // Get shortcuts to the region data.
    std::vector<FieldFeatureRegionData>& regionDataVec =
                                             info_middle.fieldFeatureRegionData;
    std::vector<RANSACLine>& tempLines = info_middle.valid_lines;

    // Whether each region is already part of a feature.
    std::vector<bool> taken(regionDataVec.size(), false);

    // Make space for lines.
    tempLines.reserve(regionDataVec.size());

    // Try each region as a starting point for a line.
    for(unsigned int sourceRegionID=0; sourceRegionID<regionDataVec.size();
                                                               ++sourceRegionID)
    {
        // The source region data.
        FieldFeatureRegionData& sourceRegionData =
                                                  regionDataVec[sourceRegionID];

        // Check that this region is a line and is not already part of a
        // feature.
        if(sourceRegionData.isLine && !taken[sourceRegionID] &&
                                     sourceRegionData.isCurveConfidence <= 0.0f)
        {
            // Whether a new region was added to the line this cycle.
            bool newRegionAdded = true;

            // The start and end points of the line.
            Point start = sourceRegionData.lineEnds.first;
            Point end = sourceRegionData.lineEnds.second;

            // Calculate the line equation of the current line.
            RANSACLine* lineEquation = new RANSACLine(start, end);

            // The current line length.
            int lineLength = DISTANCE_SQR(start, end);

            // Create a line from this region.
            curFeature.push_back(sourceRegionID);
            taken[sourceRegionID] = true;

            // Add other regions to the line as possible.
            while(newRegionAdded)
            {
                // No new region has been added yet.
                newRegionAdded = false;

                // Search for new regions to add.
                for(unsigned int regionID=0; regionID<regionDataVec.size();
                                                                     ++regionID)
                {
                    // The region data.
                    FieldFeatureRegionData& regionData =
                                                        regionDataVec[regionID];

// #ifdef RFFD_USES_VATNAO
//                     if (vdm != NULL)
//                     {
//                         VisionDebugQuery q = vdm->getQuery();
//                         if (q.region_index == sourceRegionID)
//                         {
//                             vdm->msg << "Distance to region " << regionID
//                                 << ": " << lineEquation->distance(
//                                 regionData.lineEnds.first) << " / " <<
//                                 lineEquation->distance(
//                                        regionData.lineEnds.second) << std::endl;
//                         }
//                     }
// #endif // RFFD_USES_VATNAO

                    if(regionData.isLine && !taken[regionID] &&
                        regionData.isCurveConfidence <= 0.0f &&
                        lineEquation->distance(regionData.lineEnds.first) <
                        MIN_LINE_CONNECTION_DISTANCE_INTERPOLATE(
                        regionData.lineEnds.first.x()) &&
                        lineEquation->distance(regionData.lineEnds.second) <
                        MIN_LINE_CONNECTION_DISTANCE_INTERPOLATE(
                                                regionData.lineEnds.second.x()))
                    {
                        // Record the distances between the new points and the
                        // current line start/end.
                        int firstToStart =
                                 DISTANCE_SQR(regionData.lineEnds.first, start);
                        int firstToEnd =
                                   DISTANCE_SQR(regionData.lineEnds.first, end);
                        int secondToStart =
                                DISTANCE_SQR(regionData.lineEnds.second, start);
                        int secondToEnd =
                                  DISTANCE_SQR(regionData.lineEnds.second, end);

                        // The maximum distance away this new region can be from
                        // the current region while still being connected.
                        int firstMaxLineExtensionDistance =
                            MAX_LINE_EXTENSION_DISTANCE_INTERPOLATE(
                                       regionData.lineEnds.first.squaredNorm());
                        int secondMaxLineExtensionDistance =
                            MAX_LINE_EXTENSION_DISTANCE_INTERPOLATE(
                                      regionData.lineEnds.second.squaredNorm());

#ifdef RFFD_USES_VATNAO
                        if (vdm != NULL)
                        {
                            VisionDebugQuery q = vdm->getQuery();
                            if (q.region_index == sourceRegionID)
                            {
                                vdm->msg << "Distances to segments:" <<
                                                                      std::endl;
                                vdm->msg << "firstToStart: " << firstToStart <<
                                                                      std::endl;
                                vdm->msg << "firstToEnd: " << firstToEnd <<
                                                                      std::endl;
                                vdm->msg << "secondToStart: " << secondToStart
                                                                   << std::endl;
                                vdm->msg << "secondToEnd: " << secondToEnd <<
                                                                      std::endl;
                            }
                        }
#endif // RFFD_USES_VATNAO

                        // If all the points on the new line are far from the
                        // existing line don't connect it.
                        if(firstToStart > firstMaxLineExtensionDistance &&
                                firstToEnd > firstMaxLineExtensionDistance &&
                                secondToStart > secondMaxLineExtensionDistance
                                && secondToEnd > secondMaxLineExtensionDistance)
                            continue;

#ifdef RFFD_USES_VATNAO
                        if (vdm != NULL)
                        {
                            VisionDebugQuery q = vdm->getQuery();
                            if (q.region_index == sourceRegionID)
                            {
                                vdm->msg << "JOINED" << std::endl;
                            }
                        }
#endif // RFFD_USES_VATNAO

                        // Add the new line segment.
                        taken[regionID] = true;
                        newRegionAdded = true;
                        curFeature.push_back(regionID);

                        // If this is farther from the start than the old end,
                        // and is on the same side of the line as the start, it
                        // should be the new start.
                        if(firstToEnd > lineLength && firstToEnd > firstToStart)
                        {
                            // Extend the line.
                            start = regionData.lineEnds.first;
                            lineLength = firstToEnd;

                            // Update the line equation.
                            delete lineEquation;
                            lineEquation = new RANSACLine(start, end);
                        }

                        // If this is farther from the start than the old end,
                        // and is on the same side of the line as the start, it
                        // should be the new start.
                        if(secondToEnd > lineLength && secondToEnd >
                                                                  secondToStart)
                        {
                            // Extend the line.
                            start = regionData.lineEnds.second;
                            lineLength = secondToEnd;

                            // Update the line equation.
                            delete lineEquation;
                            lineEquation = new RANSACLine(start, end);
                        }

                        // If this is farther from the end than the old start,
                        // and is on the same side of the line as the end, it
                        // should be the new end.
                        if(firstToStart > lineLength && firstToStart >
                                                                     firstToEnd)
                        {
                            // Extend the line.
                            end = regionData.lineEnds.first;
                            lineLength = firstToStart;

                            // Update the line equation.
                            delete lineEquation;
                            lineEquation = new RANSACLine(start, end);
                        }

                        // If this is farther from the start than the old end,
                        // and is on the same side of the line as the start, it
                        // should be the new start.
                        if(secondToStart > lineLength && secondToStart >
                                                                    secondToEnd)
                        {
                            // Extend the line.
                            end = regionData.lineEnds.second;
                            lineLength = secondToStart;

                            // Update the line equation.
                            delete lineEquation;
                            lineEquation = new RANSACLine(start, end);
                        }
                    }
                }
            }

            // All valid regions have been added, check if the resulting line is
            // long enough.
            if(lineLength > MINIMUM_LINE_LENGTH_INTERPOLATE(
                              std::min(start.squaredNorm(), end.squaredNorm())))
            {
                // Note that this is making a copy.
                tempLines.push_back(*lineEquation);
            }
            else
            {
                // The potential line is invalid, so free up the regions.
                for(unsigned int regionID=0; regionID<curFeature.size();
                                                                     ++regionID)
                    taken[curFeature[regionID]] = false;
            }

            // Clean up variables.
            delete lineEquation;
            curFeature.clear();
        }
    }
}


/*
Construct lines
*/
void RegionFieldFeatureDetector::constructLines_(VisionInfoMiddle& info_middle)
{
    std::vector<FieldFeatureRegionData>& regionDataVec =
                                             info_middle.fieldFeatureRegionData;
    std::vector<RANSACLine>& tempLines = info_middle.valid_lines;

    // Try each region as a starting point for a line.
    for(unsigned int sourceRegionID=0; sourceRegionID<regionDataVec.size();
                                                               ++sourceRegionID)
    {
        // The source region data.
        FieldFeatureRegionData& sourceRegionData =
                                                  regionDataVec[sourceRegionID];

        // Check that this region is a line and is not already part of a
        // feature.
        if(sourceRegionData.isLine &&
           sourceRegionData.isCurveConfidence <= 0.0f &&
           sourceRegionData.isCornerConfidence <= 0.0f &&
           sourceRegionData.isTConfidence <= 0.0f)
        {
            // The start and end points of the line.
            Point start = sourceRegionData.lineEnds.first;
            Point end = sourceRegionData.lineEnds.second;

            tempLines.push_back(RANSACLine(start, end));
        } else if (sourceRegionData.isLine &&
                   (sourceRegionData.isCornerConfidence > 0.0f ||
                    sourceRegionData.isTConfidence > 0.0f))
        {
            Point intersection = sourceRegionData.intersection;
            for (unsigned i=0; i < sourceRegionData.fieldEnds.size(); ++i)
            {
                Point fieldEnd = sourceRegionData.fieldEnds[i];
                tempLines.push_back(RANSACLine(intersection, fieldEnd));
            }
        }
    }
}



/*
Merge existing lines into longer lines where possible.
*/
void RegionFieldFeatureDetector::mergeLines_(VisionInfoMiddle& info_middle,
                        std::vector<bool>& merged, std::vector<int>& curFeature)
{
    // Get the current line set.
    std::vector<RANSACLine>& tempLines = info_middle.valid_lines;

    // Mark any lines that are merged to other lines as merged.
    merged.resize(tempLines.size(), false);

    /*
    // This section merges the lines built above if possible, as there are
    // certain circumstances in which two sections can be build seperately and
    // not connected. As far as I am aware it is functional, but there was not
    // time to test it before comp.
    for(unsigned int sourceLine=0; sourceLine<tempLines.size(); ++sourceLine)
    {
        // Whether a new region was added to the line this cycle.
        bool newLineAdded = true;

        // The start and end points of the line.
        Point start = validLines[sourceLine].p1;
        Point end = validLines[sourceLine].p2;

        // Calculate the line equation of the current line.
        RANSACLine* lineEquation = new RANSACLine(start, end);

        // The current line length.
        int lineLength = DISTANCE_SQR(start, end);

        // Create a line from this region.
        curFeature.push_back(sourceLine);
        merged[sourceLine] = true;

        // Add other regions to the line as possible.
        while(newLineAdded)
        {
            // No new line has been added yet.
            newLineAdded = false;

            // Search for new lines to add.
            for(unsigned int line=0; line<tempLines.size(); ++line)
            {
                if(!merged[line] &&
                    lineEquation->distance(tempLines[line].p1) <
                    MIN_LINE_CONNECTION_DISTANCE_INTERPOLATE(
                    tempLines[line].p1.x()) &&
                    lineEquation->distance(tempLines[line].p2) <
                    MIN_LINE_CONNECTION_DISTANCE_INTERPOLATE(
                                                    tempLines[line].p2.x()))
                {
                    // Record the distances between the new points and the
                    // current line start/end.
                    int firstToStart =
                                    DISTANCE_SQR(tempLines[line].p1, start);
                    int firstToEnd = DISTANCE_SQR(tempLines[line].p1, end);
                    int secondToStart =
                                    DISTANCE_SQR(tempLines[line].p2, start);
                    int secondToEnd = DISTANCE_SQR(tempLines[line].p2, end);

                    // The maximum distance away this new region can be from
                    // the current region while still being connected.
                    int firstMaxLineExtensionDistance =
                        MAX_LINE_EXTENSION_DISTANCE_INTERPOLATE(
                                          tempLines[line].p1.squaredNorm());
                    int secondMaxLineExtensionDistance =
                        MAX_LINE_EXTENSION_DISTANCE_INTERPOLATE(
                                          tempLines[line].p2.squaredNorm());

                    // If all the points on the new line are far from the
                    // existing line don't connect it.
                    if(firstToStart > firstMaxLineExtensionDistance &&
                            firstToEnd > firstMaxLineExtensionDistance &&
                            secondToStart > secondMaxLineExtensionDistance
                            && secondToEnd > secondMaxLineExtensionDistance)
                        continue;

                    // Add the new line segment.
                    merged[line] = true;
                    newLineAdded = true;
                    curFeature.push_back(line);

                    // If this is farther from the start than the old end,
                    // and is on the same side of the line as the start, it
                    // should be the new start.
                    if(firstToEnd > lineLength && firstToEnd > firstToStart)
                    {
                        // Extend the line.
                        start = tempLines[line].p1;
                        lineLength = firstToEnd;

                        // Update the line equation.
                        delete lineEquation;
                        lineEquation = new RANSACLine(start, end);
                    }

                    // If this is farther from the start than the old end,
                    // and is on the same side of the line as the start, it
                    // should be the new start.
                    if(secondToEnd > lineLength && secondToEnd >
                                                              secondToStart)
                    {
                        // Extend the line.
                        start = tempLines[line].p2;
                        lineLength = secondToEnd;

                        // Update the line equation.
                        delete lineEquation;
                        lineEquation = new RANSACLine(start, end);
                    }

                    // If this is farther from the end than the old start,
                    // and is on the same side of the line as the end, it
                    // should be the new end.
                    if(firstToStart > lineLength && firstToStart >
                                                                 firstToEnd)
                    {
                        // Extend the line.
                        end = tempLines[line].p1;
                        lineLength = firstToStart;

                        // Update the line equation.
                        delete lineEquation;
                        lineEquation = new RANSACLine(start, end);
                    }

                    // If this is farther from the start than the old end,
                    // and is on the same side of the line as the start, it
                    // should be the new start.
                    if(secondToStart > lineLength && secondToStart >
                                                                secondToEnd)
                    {
                        // Extend the line.
                        end = tempLines[line].p2;
                        lineLength = secondToStart;

                        // Update the line equation.
                        delete lineEquation;
                        lineEquation = new RANSACLine(start, end);
                    }
                }
            }
        }

        // Note that this is making a copy.
        tempLines.push_back(*lineEquation);
        merged.push_back(false);

        // Clean up variables.
        delete lineEquation;
        curFeature.clear();
    }
    //*/
}


/*
Locates centre circles using region line segments.
*/
void RegionFieldFeatureDetector::detectCentreCircles_(
    VisionInfoMiddle& info_middle, std::vector<int>& curFeature,
                                                  std::vector<int>& curFeature2)
{
    // Get a shortcut to the region data.
    std::vector<FieldFeatureRegionData>& regionDataVec =
                                             info_middle.fieldFeatureRegionData;

    // Get a shortcut to the circle centre vector.
    std::vector<PointF, Eigen::aligned_allocator<PointF> >& circle_centres =
                                                     info_middle.circle_centres;
    std::vector<float>& circle_qualities = info_middle.circle_qualities;

    // The centre point of the circle currently being considered.
    PointF circle_centre;

    // Look for a centre circle.
    for(unsigned int sourceRegionID=0; sourceRegionID<regionDataVec.size();
                                                               ++sourceRegionID)
    {
        // The source region data.
        FieldFeatureRegionData& sourceRegionData =
                                                  regionDataVec[sourceRegionID];

        // If this is a line, attempt to build a circle from it.
        if(sourceRegionData.isLine)
        {
            // Every line corresponds to two potential centre circle centre
            // points. Try to build a circle from each, stopping if one is
            // found.
            RANSACCircle possibleCircle(sourceRegionData.lineEnds.first,
                    sourceRegionData.lineEnds.second, CENTER_CIRCLE_DIAMETER/2);

            // The total length of all the lines added to each circle.
            int circle1Length = 0;
            int circle2Length = 0;

            // The total length of the curved lines added to each circle.
            int curvedLength1 = 0;
            int curvedLength2 = 0;

            // Calculate the error bounds.
            int centreCircle1Error =
                CENTRE_CIRCLE_ERROR_INTERPOLATE(
                                           possibleCircle.centre.squaredNorm());
            int centreCircle2Error = CENTRE_CIRCLE_ERROR_INTERPOLATE(
                                  possibleCircle.secondaryCentre.squaredNorm());
            int minCentreCircle1DistanceSquared = ((CENTER_CIRCLE_DIAMETER/2 -
                    centreCircle1Error) * (CENTER_CIRCLE_DIAMETER/2 -
                                                           centreCircle1Error));
            int minCentreCircle2DistanceSquared = ((CENTER_CIRCLE_DIAMETER/2 -
                    centreCircle2Error) * (CENTER_CIRCLE_DIAMETER/2 -
                                                           centreCircle2Error));
            int maxCentreCircle1DistanceSquared = ((CENTER_CIRCLE_DIAMETER/2 +
                    centreCircle1Error) * (CENTER_CIRCLE_DIAMETER/2 +
                                                           centreCircle1Error));
            int maxCentreCircle2DistanceSquared = ((CENTER_CIRCLE_DIAMETER/2 +
                    centreCircle2Error) * (CENTER_CIRCLE_DIAMETER/2 +
                                                           centreCircle2Error));

            // The value associated with the minimum line lengths for this
            // circle.
            float circleLengthValue;
            float curveLengthValue;

            // Check each line to see if it is part of the circle.
            for(unsigned int regionID=0; regionID<regionDataVec.size();
                                                                     ++regionID)
            {
                // The region data.
                FieldFeatureRegionData& regionData = regionDataVec[regionID];

                if(regionData.isLine)
                {
                    // The distance to each possible centre from each line end.
                    int possibleCentre1Distance1 = DISTANCE_SQR(
                              regionData.lineEnds.first, possibleCircle.centre);
                    int possibleCentre1Distance2 = DISTANCE_SQR(
                             regionData.lineEnds.second, possibleCircle.centre);
                    int possibleCentre2Distance1 = DISTANCE_SQR(
                        regionData.lineEnds.first,
                                                possibleCircle.secondaryCentre);
                    int possibleCentre2Distance2 = DISTANCE_SQR(
                        regionData.lineEnds.second,
                                                possibleCircle.secondaryCentre);

                    // The length of the line.
                    int lineLength = DISTANCE(regionData.lineEnds.first,
                                                    regionData.lineEnds.second);

                    // Calculate the equation for this line.
                    RANSACLine lineEquation(regionData.lineEnds.first,
                                                    regionData.lineEnds.second);

                    // The minimum distance from the centre.
                    int minDistanceToCentre1 = lineEquation.distance(
                                             possibleCircle.centre.cast<int>());
                    int minDistanceToCentre2 = lineEquation.distance(
                                    possibleCircle.secondaryCentre.cast<int>());

                    // If both ends of the line are on the circle, allow it.
                    if(possibleCentre1Distance1 >
                        minCentreCircle1DistanceSquared &&
                        possibleCentre1Distance1 <
                        maxCentreCircle1DistanceSquared &&
                        possibleCentre1Distance2 >
                        minCentreCircle1DistanceSquared &&
                        possibleCentre1Distance2 <
                        maxCentreCircle1DistanceSquared &&
                        minDistanceToCentre1
                                       > MIN_CENTRE_CIRCLE_DISTANCE_FROM_CENTRE)
                    {
                        curFeature.push_back(regionID);
                        circle1Length += lineLength;
                        if(regionData.isCurveConfidence >= 1.0f)
                            curvedLength1 += lineLength;
                    }

                    if(possibleCentre2Distance1 >
                        minCentreCircle2DistanceSquared &&
                        possibleCentre2Distance1 <
                        maxCentreCircle2DistanceSquared &&
                        possibleCentre2Distance2 >
                        minCentreCircle2DistanceSquared &&
                        possibleCentre2Distance2
                        < maxCentreCircle2DistanceSquared &&
                        minDistanceToCentre2 >
                                         MIN_CENTRE_CIRCLE_DISTANCE_FROM_CENTRE)
                    {
                        curFeature2.push_back(regionID);
                        circle2Length += lineLength;
                        if(regionData.isCurveConfidence >= 1.0f)
                            curvedLength2 += lineLength;
                    }
                }
            }

            // Calculate the values for the first circle.
            circleLengthValue = MIN_CENTRE_CIRCLE_LENGTH_INTERPOLATE(
                                           possibleCircle.centre.squaredNorm());
            curveLengthValue = circleLengthValue *
                                             MIN_CURVE_PORTION_OF_CENTRE_CIRCLE;
            circleLengthValue = createValueFromThresholds_(
                circleLengthValue/2.0f, circleLengthValue, circleLengthValue*2,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_LENGTH,
                          MAX_QUALITY_FROM_CENTRE_CIRCLE_LENGTH, circle1Length);
            curveLengthValue = createValueFromThresholds_(curveLengthValue/2.0f,
                curveLengthValue, curveLengthValue*2,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH,
                    MAX_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH, curvedLength1);

            // Check if the total length of the first centre circle is enough to
            // form a valid circle.
            if(circleLengthValue > 0.0f && curveLengthValue > 0.0f)
            {
                // Add the circle with a minimal probability.
                circle_centre = possibleCircle.centre;
                circle_centres.push_back(circle_centre);
                circle_qualities.push_back(circleLengthValue+curveLengthValue);
            }

            // Calculate the values for the second circle.
            circleLengthValue = MIN_CENTRE_CIRCLE_LENGTH_INTERPOLATE(
                                           possibleCircle.centre.squaredNorm());
            curveLengthValue = circleLengthValue *
                                             MIN_CURVE_PORTION_OF_CENTRE_CIRCLE;
            circleLengthValue = createValueFromThresholds_(
                circleLengthValue/2.0f, circleLengthValue, circleLengthValue*2,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_LENGTH,
                          MAX_QUALITY_FROM_CENTRE_CIRCLE_LENGTH, circle2Length);
            curveLengthValue = createValueFromThresholds_(curveLengthValue/2.0f,
                curveLengthValue, curveLengthValue*2,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH,
                    MAX_QUALITY_FROM_CENTRE_CIRCLE_CURVE_LENGTH, curvedLength2);

            // Check if the total length of the second centre circle is enough
            // to form a valid circle.
            if(circleLengthValue > 0.0f && curveLengthValue > 0.0f)
            {
                circle_centre = possibleCircle.secondaryCentre;
                circle_centres.push_back(circle_centre);
                circle_qualities.push_back(circleLengthValue+curveLengthValue);
            }
        }
    }
}


/*
Adjust centre cirlce quality based on the quality of possible centre lines
through the centre circle.
*/
void RegionFieldFeatureDetector::detectCentreCircleLines_(
                                                  VisionInfoMiddle& info_middle)
{
    // Get the set of valid lines.
    std::vector<RANSACLine>& valid_lines = info_middle.valid_lines;

    // Get the set of candidate centre circles.
    std::vector<PointF, Eigen::aligned_allocator<PointF> >& circle_centres =
                                                     info_middle.circle_centres;
    std::vector<float>& circle_qualities = info_middle.circle_qualities;

    // Make space for the centre lines.
    info_middle.circle_centre_lines.reserve(circle_centres.size());

    // Consider every candidate centre circle.
    for(unsigned int circle=0; circle<circle_centres.size(); ++circle)
    {
        // The best line found for this circle.
        int bestLine = -1;
        float bestLineValue = -1.0;

        // Check if any of the lines can be the centre line.
        for(unsigned int line=0; line<valid_lines.size(); ++line)
        {
            // The length of this line.
            int lineLength = DISTANCE_SQR(valid_lines[line].p1,
                                                          valid_lines[line].p2);

            // The distance of the circle centre's from the line.
            int minDistanceToCentre = valid_lines[line].distance(
                                            circle_centres[circle].cast<int>());

            // The distances between the line ends and the centre circle.
            int minDistanceToLineEnd1 = DISTANCE_SQR(circle_centres[circle],
                                                          valid_lines[line].p1);
            int minDistanceToLineEnd2 = DISTANCE_SQR(circle_centres[circle],
                                                          valid_lines[line].p2);

            // The value of this line.
            float lineValue = 0.0f;

            // Construct a value based on the minimum distance to centre.
            lineValue += createValueFromThresholds_(
                MAX_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CIRCLE_CENTRE*2,
                MAX_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CIRCLE_CENTRE, 0.0f,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CENTRE_CIRCLE,
                MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_DISTANCE_FROM_CENTRE_CIRCLE,
                                                           minDistanceToCentre);

            // Construct a value based on the line length.
            lineValue += createValueFromThresholds_(
                MIN_CENTRE_CIRCLE_LINE_LENGTH/2, MIN_CENTRE_CIRCLE_LINE_LENGTH,
                2*MIN_CENTRE_CIRCLE_LINE_LENGTH,
                MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_LENGTH,
                        MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_LENGTH, lineLength);

            // Construct a value based on line ends.
            float lineEndValue =
                MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE *
                (minDistanceToLineEnd1 < lineLength && minDistanceToLineEnd2 <
                                                                    lineLength);



            // Line end probability is max if the line passes through the centre
            // area, otherwise interpolated based on the closest line end.
            if(lineEndValue != MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE)
            {
                lineEndValue = std::max(createValueFromThresholds_(
                    MAX_CENTRE_CIRCLE_LINE_END_DISTANCE*2,
                    MAX_CENTRE_CIRCLE_LINE_END_DISTANCE, 0.0f,
                    MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE,
                    MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE,
                    minDistanceToLineEnd1), createValueFromThresholds_(
                    MAX_CENTRE_CIRCLE_LINE_END_DISTANCE*2,
                    MAX_CENTRE_CIRCLE_LINE_END_DISTANCE, 0.0f,
                    MIN_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE,
                    MAX_QUALITY_FROM_CENTRE_CIRCLE_LINE_END_DISTANCE,
                                                        minDistanceToLineEnd2));
            }

            // Update the best line value.
            lineValue += lineEndValue;
            if(lineValue > bestLineValue)
            {
                bestLineValue = lineValue;
                bestLine = line;
            }
        }

        // Add the best line.
        if(bestLineValue > -1)
        {
            info_middle.circle_centre_lines.push_back(valid_lines[bestLine]);
            circle_qualities[circle] += bestLineValue;
        }
        else
        {
            // No lines, but there is a centre circle.
            info_middle.circle_centre_lines.push_back(RANSACLine());
        }
    }
}


/*
Determines the best candidate centre circle and adds it as a field feature, if
it is of sufficient quality.
*/
void RegionFieldFeatureDetector::determineBestCentreCircle_(
                         VisionInfoMiddle& info_middle, VisionInfoOut& info_out)
{
    // Get the potential centre circle info.
    std::vector<PointF, Eigen::aligned_allocator<PointF> >& circle_centres =
                                                     info_middle.circle_centres;
    std::vector<float>& circle_qualities = info_middle.circle_qualities;

    // If there are no potential circles, do nothing.
    if(circle_centres.size() == 0)
    {
        return;
    }

    // The best centre circle found.
    PointF best_circle_centre = circle_centres[0];
    float best_circle_quality = circle_qualities[0];
    int best_circle_ID = 0;

    // Determine the best potential centre cirlce.
    for(unsigned int circle = 0; circle < circle_centres.size(); ++circle)
    {
        if(circle_qualities[circle] > best_circle_quality)
        {
            best_circle_centre = circle_centres[circle];
            best_circle_quality = circle_qualities[circle];
            best_circle_ID = circle;
        }
    }

    // If the best circle is of sufficient quality, add it.
    if(best_circle_quality > MIN_CENTRE_CIRCLE_QUALITY)
    {
        // Send the centre cirlce to localisation.
        FieldFeatureInfo f = constructCenterCircleFeature_(best_circle_centre,
                               info_middle.circle_centre_lines[best_circle_ID]);
        info_out.features.push_back(f);
        info_middle.best_centre_circle = best_circle_ID;
        ++info_middle.features_sent;
    }
}

/*
Locates corners and T intersections using region line segments.
*/
void RegionFieldFeatureDetector::detectIntersections_(
                                                  VisionInfoMiddle& info_middle)
{
    // The set of lines found.
    std::vector<RANSACLine>& validLines = info_middle.valid_lines;

    // The IDs of the lines that form each intersection.
    std::vector<std::pair<int, int> >& intersection_lines =
                                                 info_middle.intersection_lines;

    // Get references to the field feature vectors.
    std::vector<Point, Eigen::aligned_allocator<Point> >& intersections =
                                                      info_middle.intersections;
    std::vector<float>& intersection_qualities =
                                             info_middle.intersection_qualities;
    std::vector<float>& intersection_corner_qualities =
                                      info_middle.intersection_corner_qualities;
    std::vector<float>& intersection_T_qualities =
                                           info_middle.intersection_T_qualities;
    std::vector<float>& intersection_4_way_qualities =
                                       info_middle.intersection_4_way_qualities;

    // Generate corners and T intersections from line intersects.
    for(unsigned int sourceLine=0; sourceLine<validLines.size(); ++sourceLine)
    {
        // Calculate the relevant attributes of the source line.
        float sourceLineAngle = validLines[sourceLine].getAngle();
        if(sourceLineAngle < 0)
            sourceLineAngle += M_PI;

        // Whether this line has a T and corner for goal box purposes.
        for(unsigned int line=sourceLine+1; line<validLines.size(); ++line)
        {
            // The angle to this line.
            float lineAngle = validLines[line].getAngle();

            // The angle between the lines.
            float angleBetween;

            // The value associated with the line angle.
            float lineAngleValue;

            // Adjust angle if needed.
            if(lineAngle < 0.0f)
                lineAngle += M_PI;

            // Calculate the angle between the lines.
            if(lineAngle > sourceLineAngle)
                angleBetween = lineAngle - sourceLineAngle;
            else
                angleBetween = sourceLineAngle - lineAngle;

            // Calculate the qualtiy value from line angle.
            lineAngleValue = createValueFromThresholds_(
                MAX_PERPENDICULAR_ANGLE_ERROR*2.0f,
                MAX_PERPENDICULAR_ANGLE_ERROR, 0.0f,
                MIN_QUALITY_FROM_PERPENDICULAR_ANGLE_ERROR,
                MAX_QUALITY_FROM_PERPENDICULAR_ANGLE_ERROR, fabs(angleBetween -
                                                                  (M_PI/2.0f)));

            // Determine if the lines are roughly perpendicular.
            if(lineAngleValue > 0.0f)
            {
                // Find the intersection between these two lines. As we know
                // they're pretty perpendicular there should be one.
                intersections.push_back(validLines[sourceLine].getIntersect(
                                                             validLines[line]));
                intersection_lines.push_back(std::make_pair(sourceLine, line));
                intersection_qualities.push_back(lineAngleValue);
                intersection_corner_qualities.push_back(0.0f);
                intersection_T_qualities.push_back(0.0f);
                intersection_4_way_qualities.push_back(0.0f);
                info_middle.on_lines.push_back(std::make_pair(false, false));
            }
        }
    }
}

/*
Adds quality to each intersection based on the length and proximity of the
associated lines. Also adds some T intersection quality if the feature is on
one of the lines.
*/
void RegionFieldFeatureDetector::adjustIntersectionQualityWithLines_(
                                                  VisionInfoMiddle& info_middle)
{
    // The set of lines found.
    std::vector<RANSACLine>& validLines = info_middle.valid_lines;

    // All intersections between near perpendicular lines.
    std::vector<Point, Eigen::aligned_allocator<Point> >& intersections =
                                                      info_middle.intersections;

    // The IDs of the lines that form each intersection.
    std::vector<std::pair<int, int> >& intersection_lines =
                                                 info_middle.intersection_lines;

    // The squared lengths of the lines.
    std::vector<int>& line_lengths = info_middle.line_lengths;

    // The quality of each intersection.
    std::vector<float>& intersection_qualities =
                                             info_middle.intersection_qualities;
    std::vector<float>& intersection_corner_qualities =
                                           info_middle.intersection_corner_qualities;
    std::vector<float>& intersection_T_qualities =
                                           info_middle.intersection_T_qualities;
    // std::vector<float>& intersection_4_way_qualities =
    //                                    info_middle.intersection_4_way_qualities;

    // Adjust each intersection.
    for(unsigned int intersectionID=0; intersectionID<intersections.size();
                                                               ++intersectionID)
    {
        // The IDs of the lines forming this intersection.
        int firstLine = intersection_lines[intersectionID].first;
        int secondLine = intersection_lines[intersectionID].second;

        // The location of the intersection.
        Point& intersection = intersections[intersectionID];

        // The quality from the lengths of the forming lines.
        float lineLengthQuality = 0.0f;

        // The minimum line length gain intersection quality at this distance.
        int minFeatureLineLength = MIN_FEATURE_LINE_LENGTH_INTERPOLATE(
                                                    intersection.squaredNorm());

        // The quality from the closest line end to the intersection.
        float closestLineEndQuality = 0.0f;

        // The distances to each line end.
        int firstToIntersection1 = DISTANCE_SQR(validLines[firstLine].p1,
                                                                  intersection);
        int firstToIntersection2 = DISTANCE_SQR(validLines[firstLine].p2,
                                                                  intersection);
        int secondToIntersection1 = DISTANCE_SQR(validLines[secondLine].p1,
                                                                  intersection);
        int secondToIntersection2 = DISTANCE_SQR(validLines[secondLine].p2,
                                                                  intersection);

        // Whether the intersection is on the first and/or second line.
        bool onFirst = firstToIntersection1 < line_lengths[firstLine] &&
                                 firstToIntersection2 < line_lengths[firstLine];
        bool onSecond = secondToIntersection1 < line_lengths[secondLine] &&
                               secondToIntersection2 < line_lengths[secondLine];

        // The maximum distance from the intersection the closest line end can
        // be to gain quality.
        int maxLineEndIntersectionDistance =
            MAX_LINE_END_INTERSECTION_DISTANCE_INTERPOLATE(
                                                    intersection.squaredNorm());

        // Calculate the quality from line length.
        lineLengthQuality += createValueFromThresholds_(minFeatureLineLength/2,
            minFeatureLineLength, minFeatureLineLength*2,
            MIN_QUALITY_FROM_LINE_LENGTH_INTERSECTION,
            MAX_QUALITY_FROM_LINE_LENGTH_INTERSECTION, line_lengths[firstLine]);
        lineLengthQuality += createValueFromThresholds_(minFeatureLineLength/2,
            minFeatureLineLength, minFeatureLineLength*2,
            MIN_QUALITY_FROM_LINE_LENGTH_INTERSECTION,
            MAX_QUALITY_FROM_LINE_LENGTH_INTERSECTION,
                                                      line_lengths[secondLine]);

        // Calculate the quality from line end proximity.

        // First line.
        if(firstToIntersection1 < firstToIntersection2)
        {
            closestLineEndQuality += createValueFromThresholds_(
                maxLineEndIntersectionDistance*2,
                maxLineEndIntersectionDistance, 0.0f,
                MIN_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                MAX_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                                                          firstToIntersection1);
        }
        else
        {
            closestLineEndQuality += createValueFromThresholds_(
                maxLineEndIntersectionDistance*2,
                maxLineEndIntersectionDistance, 0.0f,
                MIN_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                MAX_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                                                          firstToIntersection2);
        }

        // Second line.
        if(secondToIntersection1 < secondToIntersection2)
        {
            closestLineEndQuality += createValueFromThresholds_(
                maxLineEndIntersectionDistance*2,
                maxLineEndIntersectionDistance, 0.0f,
                MIN_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                MAX_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                                                         secondToIntersection1);
        }
        else
        {
            closestLineEndQuality += createValueFromThresholds_(
                maxLineEndIntersectionDistance*2,
                maxLineEndIntersectionDistance, 0.0f,
                MIN_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                MAX_QUALITY_FROM_LINE_END_DISTANCE_INTERSECTION,
                                                         secondToIntersection2);
        }

        // Update the intersection quality.
        intersection_qualities[intersectionID] += lineLengthQuality;
        intersection_qualities[intersectionID] += closestLineEndQuality;

        // If the intersection appears to lie on both lines, increase the
        // centre circle 4 way intersection quality.
        if(onFirst && onSecond)
        {
            // intersection_4_way_qualities[intersectionID] +=
            //     createValueFromThresholds_(T_INTERSECTION_EXCESS/2,
            //     T_INTERSECTION_EXCESS, T_INTERSECTION_EXCESS*2,
            //     MIN_QUALITY_FROM_T_INTERSECTION_ON_LINE,
            //     MAX_QUALITY_FROM_T_INTERSECTION_ON_LINE, std::min(
            //                        firstToIntersection1, firstToIntersection2));
            // intersection_4_way_qualities[intersectionID] +=
            //     createValueFromThresholds_(T_INTERSECTION_EXCESS/2,
            //     T_INTERSECTION_EXCESS, T_INTERSECTION_EXCESS*2,
            //     MIN_QUALITY_FROM_T_INTERSECTION_ON_LINE,
            //     MAX_QUALITY_FROM_T_INTERSECTION_ON_LINE, std::min(
            //                      secondToIntersection1, secondToIntersection2));
        }

        // Update T intersection quality if the intersection is on one (and only
        // one) of the lines.
        else if(onFirst)
        {
            intersection_T_qualities[intersectionID] +=
                createValueFromThresholds_(T_INTERSECTION_EXCESS/2,
                T_INTERSECTION_EXCESS, T_INTERSECTION_EXCESS*2,
                MIN_QUALITY_FROM_T_INTERSECTION_ON_LINE,
                MAX_QUALITY_FROM_T_INTERSECTION_ON_LINE, std::min(
                                   firstToIntersection1, firstToIntersection2));
        }
        else if(onSecond)
        {
            intersection_T_qualities[intersectionID] +=
                createValueFromThresholds_(T_INTERSECTION_EXCESS/2,
                T_INTERSECTION_EXCESS, T_INTERSECTION_EXCESS*2,
                MIN_QUALITY_FROM_T_INTERSECTION_ON_LINE,
                MAX_QUALITY_FROM_T_INTERSECTION_ON_LINE, std::min(
                                 secondToIntersection1, secondToIntersection2));
        }
        else
        {
            intersection_corner_qualities[intersectionID] += 1.0f;
        }

        // Record which lines this intersection is on.
        info_middle.on_lines[intersectionID].first = onFirst;
        info_middle.on_lines[intersectionID].second = onSecond;
    }
}

/*
Adds quality to each intersection based on the all types of confidence of the
regions that contains the intersections
*/
void RegionFieldFeatureDetector::adjustIntersectionQualityWithRegions_(
                                                VisionInfoMiddle& info_middle)
{
    // All intersections between near perpendicular lines.
    std::vector<Point, Eigen::aligned_allocator<Point> >& intersections =
                                                      info_middle.intersections;

    // The quality of each intersection.
    std::vector<float>& intersection_corner_qualities =
                                      info_middle.intersection_corner_qualities;
    std::vector<float>& intersection_T_qualities =
                                           info_middle.intersection_T_qualities;


    std::vector<Point, Eigen::aligned_allocator<Point> >& potentialCornerIntersections
                                            = info_middle.potentialCornerIntersections;
    std::vector<Point, Eigen::aligned_allocator<Point> >& potentialTIntersections
                                            = info_middle.potentialTIntersections;

    // Run through all intersections
    for(unsigned int intersectionID=0; intersectionID<intersections.size();
                                                           ++intersectionID)
    {
        // The location of the intersection.
        Point& intersection = intersections[intersectionID];

        for(unsigned int j=0; j<potentialCornerIntersections.size(); ++j)
        {
            Point& C = potentialCornerIntersections[j];
            if(DISTANCE_SQR(intersection, C) <= MAX_CORNER_INTERSECTION_ERROR)
            {
                intersection_corner_qualities[intersectionID] += 100.0f;
                break;
            }
        }

        for(unsigned int i=0; i<potentialTIntersections.size(); ++i)
        {
            Point& T = potentialTIntersections[i];
            if( DISTANCE_SQR(intersection, T) <= MAX_T_INTERSECTION_ERROR)
            {
                intersection_T_qualities[intersectionID] += 200.0f;
                //std::cout<<"Got one T"<<std::endl;
                break;
            }
        }
    }
}

/*
Add corner quality to the other end of lines with good T quality as it might be
the goal box.
*/
void RegionFieldFeatureDetector::goalBoxCheck_(VisionInfoMiddle& info_middle)
{
    // All intersections between near perpendicular lines.
    std::vector<Point, Eigen::aligned_allocator<Point> >& intersections =
                                                      info_middle.intersections;

    // The IDs of the lines that form each intersection.
    std::vector<std::pair<int, int> >& intersection_lines =
                                                 info_middle.intersection_lines;

    // The quality of each intersection.
    std::vector<float>& intersection_corner_qualities =
                                      info_middle.intersection_corner_qualities;
    std::vector<float>& intersection_T_qualities =
                                           info_middle.intersection_T_qualities;

    // Search for pairs of intersections close enough to be a goal box.
    for(unsigned int intersectionID1=0; intersectionID1 < intersections.size();
                                                              ++intersectionID1)
    {
        // The IDs of the lines forming this intersection.
        int firstLine1 = intersection_lines[intersectionID1].first;
        int secondLine1 = intersection_lines[intersectionID1].second;

        for(unsigned int intersectionID2 = intersectionID1+1; intersectionID2 <
                                        intersections.size(); ++intersectionID2)
        {
            // The IDs of the lines forming this intersection.
            int firstLine2 = intersection_lines[intersectionID2].first;
            int secondLine2 = intersection_lines[intersectionID2].second;

            // If both features show no signs of being a T, do nothing.
            // TODO: Maybe we can do something useful here?
            if(intersection_T_qualities[intersectionID1] == 0.0f &&
                              intersection_T_qualities[intersectionID2] == 0.0f)
            {
                continue;
            }

            // Check for a shared line.
            if(firstLine1 == firstLine2 || firstLine1 == secondLine2 ||
                        secondLine1 == firstLine2 || secondLine1 == secondLine2)
            {
                // The distance between the two intersections.
                int intersectionDistance = DISTANCE(
                    intersections[intersectionID1],
                                                intersections[intersectionID2]);

                // The error of the intersection distance relative to the goal
                // box distance.
                float goalBoxError = abs(intersectionDistance-GOAL_BOX_LENGTH);

                // The value of this pairing.
                float pairValue = createValueFromThresholds_(
                    MAX_GOAL_BOX_ERROR*2, MAX_GOAL_BOX_ERROR, 0.0f,
                    MIN_QUALITY_FROM_GOAL_BOX_PROXIMITY,
                             MAX_QUALITY_FROM_GOAL_BOX_PROXIMITY, goalBoxError);

                // Strengthen the stronger T as a T, and the other as a corner.
                if(intersection_T_qualities[intersectionID1] >
                                      intersection_T_qualities[intersectionID2])
                {
                    intersection_T_qualities[intersectionID1] += pairValue;
                    intersection_corner_qualities[intersectionID2] += pairValue;
                }
                else
                {
                    intersection_T_qualities[intersectionID2] += pairValue;
                    intersection_corner_qualities[intersectionID1] += pairValue;
                }
            }
        }
    }
}

/*
Sends Ts and coners of high enough quality to localisation.
TODO: Select the best features to send, rather than just the first ones to pass
      the threshold. Note: usually wont matter.
*/
void RegionFieldFeatureDetector::createTsAndCorners_(
    const VisionInfoIn& info_in, VisionInfoMiddle& info_middle,
                                                        VisionInfoOut& info_out)
{
    // All intersections between near perpendicular lines.
    std::vector<Point, Eigen::aligned_allocator<Point> >& intersections =
                                                      info_middle.intersections;

    // The IDs of the lines that form each intersection.
    std::vector<std::pair<int, int> >& intersection_lines =
                                                 info_middle.intersection_lines;

    // The quality of each intersection.
    std::vector<float>& intersection_qualities =
                                             info_middle.intersection_qualities;

    // Whether the intersections are on each source line.
    std::vector<std::pair<bool, bool> >& onLines = info_middle.on_lines;

    // All valid lines.
    std::vector<RANSACLine>& validLines = info_middle.valid_lines;

    // Check all intersections.
    for(unsigned int intersectionID=0; intersectionID<intersections.size();
                                                               ++intersectionID)
    {
        // The IDs of the lines forming this intersection.
        int firstLine = intersection_lines[intersectionID].first;
        int secondLine = intersection_lines[intersectionID].second;

        // The location of the intersection.
        Point& intersection = intersections[intersectionID];

        // The quality of this intersection as a T.
        float tQuality = info_middle.intersection_T_qualities[intersectionID] -
            info_middle.intersection_corner_qualities[intersectionID] -
                       info_middle.intersection_4_way_qualities[intersectionID];

        // The quality of this intersection as a corner.
        float cornerQuality =
            info_middle.intersection_corner_qualities[intersectionID] -
            info_middle.intersection_T_qualities[intersectionID] -
                       info_middle.intersection_4_way_qualities[intersectionID];

        std::vector<Point, Eigen::aligned_allocator<Point> >& potentialTIntersections
                                                    = info_middle.potentialTIntersections;

        std::vector<float>& potentialTIntersectionAngles
                                                = info_middle.potentialTIntersectionAngles;

        std::vector<Point, Eigen::aligned_allocator<Point> >& potentialCornerIntersections
                                                    = info_middle.potentialCornerIntersections;

        std::vector<float>& potentialCornerIntersectionAngles
                                                = info_middle.potentialCornerIntersectionAngles;

        // If the intersection has enough T quality, try to form a T.
        if(tQuality > T_INTERSECTION_QUALITY_THRESHOLD && tQuality >
                                                                  cornerQuality)
        {
            // Check if the intersection is high enough quality overall.
            if(intersection_qualities[intersectionID] + tQuality >
                                                 INTERSECTION_QUALITY_THRESHOLD)
            {
                // std::cout<<"T detected: "<<std::endl;
                // std::cout<<"intersection_qualities: "<<intersection_qualities[intersectionID]<<std::endl;
                // std::cout<<"intersection_corner_qualities: "<<info_middle.intersection_corner_qualities[intersectionID]<<std::endl;
                // std::cout<<"cornerQuality: "<<cornerQuality<<std::endl;
                // std::cout<<"intersection_T_qualities: "<<info_middle.intersection_T_qualities[intersectionID]<<std::endl;
                // std::cout<<"tQuality: "<<tQuality<<std::endl;
                // std::cout<<"intersection_4_way_qualities"<<info_middle.intersection_4_way_qualities[intersectionID]<<std::endl<<std::endl;
                float angle = findTAngle_(intersection,
                    (onLines[intersectionID].first ? validLines[secondLine] :
                                                        validLines[firstLine]));
                for(unsigned int i=0; i<potentialTIntersections.size(); ++i)
                {
                    Point& T = potentialTIntersections[i];
                    if( DISTANCE_SQR(intersection, T) <= MAX_T_INTERSECTION_ERROR)
                    {
                        angle = potentialTIntersectionAngles[i];
                    }
                }

                float distance = sqrt(intersection.x() * intersection.x() +
                                           intersection.y() * intersection.y());
                float heading = atan2(intersection.y(), intersection.x());
                RRCoord r = RRCoord(distance, heading, -angle);
                Point intersectionImage =
                      info_in.cameraToRR.convertToImageXY(intersection);
                TJunctionInfo t = TJunctionInfo(intersectionImage);
                FieldFeatureInfo f = FieldFeatureInfo(r, t);
                ++info_middle.features_sent;
                info_out.features.push_back(f);
                if(info_middle.features_sent >= MAX_LOCALISATION_FEATURES)
                    return;
            }
        }

        // Otherwise try to build a corner.
        else if(cornerQuality > CORNER_QUALITY_THRESHOLD
                && info_middle.intersection_corner_qualities[intersectionID] > CORNER_QUALITY_THRESHOLD)
        {
            // Check if the intersection is high enough quality overall.
            if(intersection_qualities[intersectionID] + cornerQuality >
                                                 INTERSECTION_QUALITY_THRESHOLD)
            {
                // std::cout<<"Corner detected: "<<std::endl;
                // std::cout<<"intersection_qualities: "<<intersection_qualities[intersectionID]<<std::endl;
                // std::cout<<"intersection_corner_qualities: "<<info_middle.intersection_corner_qualities[intersectionID]<<std::endl;
                // std::cout<<"cornerQuality: "<<cornerQuality<<std::endl;
                // std::cout<<"intersection_T_qualities: "<<info_middle.intersection_T_qualities[intersectionID]<<std::endl;
                // std::cout<<"tQuality: "<<tQuality<<std::endl;
                // std::cout<<"intersection_4_way_qualities"<<info_middle.intersection_4_way_qualities[intersectionID]<<std::endl<<std::endl;
                // Determine the robot relative coordinates.

                // Determine the robot relative heading relative angle.
                float angle = findCAngle_(intersection, validLines[firstLine],
                                                        validLines[secondLine]);
                for(unsigned int i=0; i<potentialCornerIntersections.size(); ++i)
                {
                    Point& C = potentialCornerIntersections[i];
                    if( DISTANCE_SQR(intersection, C) <= MAX_CORNER_INTERSECTION_ERROR)
                    {
                        angle = potentialCornerIntersectionAngles[i];
                    }
                }

                Point e1, e2;
                float distance = sqrt(intersection.x() * intersection.x() +
                                           intersection.y() * intersection.y());
                float heading = atan2(intersection.y(), intersection.x());

                // Determine the image relative location of the feature.
                Point intersectionImage =
                              info_in.cameraToRR.convertToImageXY(intersection);
                e1 = info_in.cameraToRR.convertToImageXY(e1);
                e2 = info_in.cameraToRR.convertToImageXY(e2);

                // Create the final feature.
                RRCoord r = RRCoord(distance, heading, -angle);
                CornerInfo c = CornerInfo(intersectionImage, e1, e2);
                FieldFeatureInfo f = FieldFeatureInfo(r, c);
                ++info_middle.features_sent;
                info_out.features.push_back(f);
                if(info_middle.features_sent >= MAX_LOCALISATION_FEATURES)
                    return;
            }
        }
    }
}

/*
Finally create penalty cross from previous detector
*/
void RegionFieldFeatureDetector::createPenaltyCross_(const VisionInfoIn& info_in,
                        VisionInfoMiddle& info_middle, VisionInfoOut& info_out)
{
    // All valid penalty cross
    std::vector<Point>& valid_penalty_cross = info_middle.valid_penalty_cross;

    // Check all penalty cross
    for(unsigned int crossID=0; crossID<valid_penalty_cross.size(); ++crossID)
    {
        Point intersection = valid_penalty_cross[crossID];
        float distance = sqrt(intersection.x() * intersection.x() +
                                   intersection.y() * intersection.y());
        float heading = 0.0f;
        if(intersection.x()!=0)
        {
            heading = atan2(intersection.y(), intersection.x());
        }
        else
        {
            heading = M_PI / 2 * ((intersection.y() >= 0) - (intersection.y() < 0));
        }

        RRCoord r = RRCoord(distance, heading);
        Point tl = intersection + Point(50, 50);
        Point intersectionImage = info_in.cameraToRR.convertToImageXY(intersection);
        Point tl_Image = info_in.cameraToRR.convertToImageXY(tl);
        int w = (intersectionImage.x() - tl_Image.x()) * 2;
        int h = (intersectionImage.y() - tl_Image.y()) * 2;
        PenaltySpotInfo pc = PenaltySpotInfo(intersectionImage, w, h);
        FieldFeatureInfo f = FieldFeatureInfo(r, pc);
        ++info_middle.features_sent;
        info_out.features.push_back(f);
        if(info_middle.features_sent >= MAX_LOCALISATION_FEATURES)
            return;
    }

}

/*
Clears lines that are merged or too near the centre circle.
*/
void RegionFieldFeatureDetector::cleanUpAndSendLines_(
    VisionInfoMiddle& info_middle, VisionInfoOut& info_out,
                                                      std::vector<bool>& merged)
{
    // The set of lines found.
    std::vector<RANSACLine>& validLines = info_middle.valid_lines;

    // Copy the lines found so valid ones can be extracted.
    std::vector<RANSACLine> tempLines = info_middle.valid_lines;

    // The squared lengths of the lines.
    std::vector<int>& line_lengths = info_middle.line_lengths;

    // Whether a centre circle was found.
    bool circle_found = info_middle.best_centre_circle != -1;

    // The centre of the centre circle found.
    PointF circle_centre(0,0);

    // Get the centre of the centre circle, if any.
    if(circle_found)
    {
        circle_centre = info_middle.circle_centres[
                                                info_middle.best_centre_circle];
    }

    // Clear the current valid lines, so it can be refilled with only acceptable
    // lines.
    validLines.clear();

    // Keep valid lines.
    for(unsigned int line=0; line<tempLines.size(); ++line)
    {
        if(!merged[line] && (!circle_found ||
            (DISTANCE_SQR(tempLines[line].p1, circle_centre) >
            MIN_DISTANCE_TO_CENTRE_CIRCLE && DISTANCE_SQR(tempLines[line].p2,
                               circle_centre) > MIN_DISTANCE_TO_CENTRE_CIRCLE)))
        {
            // Keep the line.
            validLines.push_back(tempLines[line]);

            // Buffer the length of each valid line for use in later sections.
            line_lengths.push_back(DISTANCE_SQR(tempLines[line].p1,
                                                            tempLines[line].p2));

            // Pass the line to localisation.
            LineInfo l1(tempLines[line].p1, tempLines[line].p2);
            RRCoord c1(0, 0);
            FieldFeatureInfo f1 = FieldFeatureInfo(c1, l1);
            info_out.features.push_back(f1);
        }
    }
}


/*
Interpolates between hyperparameter values based on distance.
*/
inline int RegionFieldFeatureDetector::interpolateHyperparameters_(
    const long long distance, const long long min, const long long max,
                     const long long min_distance, const long long max_distance)
{
    // First check the thresholds.
    if(distance < min_distance)
        return(min);
    else if(distance > max_distance)
        return(max);

    // Interpolate between min and max.
    else
    {
        return(min +
             (((max-min)*(distance-min_distance))/(max_distance-min_distance)));
    }
}


/*
Determines the angle of a T intersection in the form required by
localisation.
*/
float RegionFieldFeatureDetector::findTAngle_(Point &p, RANSACLine &l) {
   float angle = findGradient_(l, p);
   float theta = atan2(p.x(), p.y());

   if (angle > 0) {
      angle = theta + DEG2RAD(180) - angle;
   } else {
      angle = theta - (DEG2RAD(180) + angle);
   }

   if (angle > M_PI) {
      angle = NORMALISE(angle);
    }
   return angle;
}

/*
Determines the angle of a corner in the form required by localisation.
*/
float RegionFieldFeatureDetector::findCAngle_(Point &p, RANSACLine &l1,
                                                                 RANSACLine &l2)
{
    float g1 = findGradient_(l1, p);
    float g2 = findGradient_(l2, p);
    float angle = (g1+g2)/2;
    float roughQuadrantLimit = (8*(M_PI/18));
    if ((g1 > 0) && (g2 < 0) && (g1 > roughQuadrantLimit) &&
                                                     (g2 < -roughQuadrantLimit))
    {
        g2 += 2*M_PI;
        angle = (g1+g2)/2;
    }
    if ((g2 > 0) && (g1 < 0) && (g2 > roughQuadrantLimit) &&
                                                     (g1 < -roughQuadrantLimit))
    {
        g1 += 2*M_PI;
        angle = (g1+g2)/2;
    }
    if (angle > M_PI) angle -= 2*M_PI;

    float theta = atan2(p.x(), p.y());

    if (angle > 0)
        angle = theta + M_PI - angle;
    else
      angle = theta - (M_PI + angle);

   if (angle > M_PI)
      angle = fmod(angle - M_PI, 2 * M_PI) + ((angle > 0) ? -M_PI : M_PI);

   return angle;
}


/*
Determines the gradient of a line relative to a point.
*/
float RegionFieldFeatureDetector::findGradient_(RANSACLine &l, Point &p)
{
    // Work out which direction to find gradient in
    float distp1 = DISTANCE_SQR(p, l.p1);
    float distp2 = DISTANCE_SQR(p, l.p2);
    Point far;
    Point close;
    if (distp1 > distp2)
    {
        far = l.p1;
        close = l.p2;
    }
    else
    {
        far = l.p2;
        close = l.p1;
    }
    return (atan2(far.x() - close.x(), far.y() - close.y()));
}

/*
Creates an instance of FieldFeatureInfo for a centre circle given its centre
point and the centre line.
*/
FieldFeatureInfo RegionFieldFeatureDetector::constructCenterCircleFeature_(
                                          PointF circle_centre, RANSACLine line)
{
    float distance = sqrt(
        pow(circle_centre.x(), 2) +
        pow(circle_centre.y(), 2)
    );
    float heading = atan2(circle_centre.y(), circle_centre.x());
    RRCoord coord = RRCoord(distance, heading);
    CentreCircleInfo info = CentreCircleInfo();
    FieldFeatureInfo feature = FieldFeatureInfo(coord, info);
    float angle = getRobotCentreAngle_(feature) - getCentreLineAngle_(line);
    if (angle < 0) {
       angle += M_PI;
    }
    feature.rr.orientation() = angle;
    return feature;
}


/*
Calculates the angle between the robot and the given field feature.
*/
float RegionFieldFeatureDetector::getRobotCentreAngle_(FieldFeatureInfo feature)
{
    Point directionRight = Point(0, 0);
    Point directionLeft = feature.rr.toCartesian();
    if (directionRight.y() > directionLeft.y()) {
        directionRight = feature.rr.toCartesian();
        directionLeft = Point(0, 0);
    }
    return atan2(
        directionRight.y() - directionLeft.y(),
        directionRight.x() - directionLeft.x()
    );
}

/*
Calculates the angle of the centre line given a line.
*/
float RegionFieldFeatureDetector::getCentreLineAngle_(RANSACLine line)
{
    Point centreRight = line.p1;
    Point centreLeft = line.p2;
    if (centreRight.y() > centreLeft.y()) {
       centreRight = line.p2;
       centreLeft = line.p1;
    }
    return atan2(
        centreRight.y() - centreLeft.y(),
        centreRight.x() - centreLeft.x()
    );
}
