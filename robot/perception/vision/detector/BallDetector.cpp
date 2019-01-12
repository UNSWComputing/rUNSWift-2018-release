#include "perception/vision/detector/BallDetector.hpp"
#include "perception/vision/Region.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/other/histogram.hpp"
#include "perception/vision/other/Ransac.hpp"
#include "perception/vision/other/WriteImage.hpp"

#include "types/RansacTypes.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/GroupLinks.hpp"
#include "types/Point.hpp"
#include "types/BBox.hpp"

#include "soccer.hpp"

#include <iostream>
#include <math.h>
#include <climits>
#include <vector>
#include <iomanip>

// The internal regions parameters.
#define MIN_REGION_SIZE 32*32
#define MIN_INTERNAL_REGION_GROUP_SIZE 28
#define MAX_INTERNAL_REGION_GROUP_SIZE 46 // TODO: Should this be adjusted by region size?

#define BLACK_ROI_MIN_GROUP_SIZE 3

// Amount of padding for the region, in pixels.
#define PADDING_WIDTH 2
#define PADDING_HEIGHT 2

// The proportion off a 1.0 width/height ratio a ball can be.
#define BALL_RATIO 1.0

#define NORMALISE_PIXEL(val, c) std::min(std::max(((double)val* c) , 0.0), 255.0)

#define HEADTILTLIMIT 0.2

// Ball radius max/min in no. of pixels
#define TOP_CAMERA_MIN_RADIUS 10
#define TOP_CAMERA_MAX_RADIUS 110
#define BOT_CAMERA_MIN_RADIUS 20
#define BOT_CAMERA_MAX_RADIUS 90

// Internal Region sidelength:radius ratio limits
#define INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MIN 0.3
#define INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MAX 0.8

// The portion of white in the bottom camera required to run the extensive ball
// detector.
#define WHITE_HEAVY_THRESHOLD 0.05

#define CIRCLE_FIT_MAX_CENTER_POINTS 100
#define CIRCLE_FIT_GAUSSIAN_SIZE 5
#define CIRCLE_FIT_CENTRE_VOTE_THRESHOLD 300
#define CIRCLE_FIT_RADIUS_VOTE_THRESHOLD 8

#define TRIANGLE_FIT_ERROR_THRESHOLD 115
#define TRIANGLE_FIT_DIST_RAD_RATIO_MIN 0.25
#define TRIANGLE_FIT_DIST_RAD_RATIO_MAX 0.65

#define BALL_Y_RANGE_THRESHOLD 40

// The minimum portion of region cols before and after a black pixel for a
// region to be considered a potential ball.
#define MIN_SECTION_SIZE_PORTION 0.01f

//#define BALL_DETECTOR_TIMINGS 1
//#define BALL_DEBUG 1
#define EARLY_EXIT 1 // Find the first ball and stop.
//#define BLOBROI_DEBUG 1

// Write the ball to file.
// First `mkdir media/usb` on board the robot
// The mount the usb `sudo mount /dev/sdb1 /media/usb -o umask=000`
// Then run runswift.
// Unmount the usb `sudo umount /media/usb`
//#define BALL_TO_FILE 1
//#define BALL_TO_FILE_DIR "/media/usb/Balls/"

// random numbers pregenerated for circle fitting
static const int randomNums[300] =
    {2186,3239,1,8418,7672,1986,4083,5536,3430,2753,6445,3645,6085,5719,6570,4466,
    3535,3566,2529,6752,497,7368,3026,7023,521,6979,5402,512,6176,4284,4099,4635,
    8336,2467,1082,4949,5164,8169,8791,9351,5662,6454,4139,7898,2011,4156,9192,
    1434,20,6415,7837,4374,4414,686,6346,3598,1834,1194,1807,3162,5011,9100,6463,
    4583,5725,8222,1453,4164,9179,7993,2862,1158,9176,323,6279,9546,6635,4309,1475,
    3849,2489,7054,484,3873,1393,7154,6164,5857,2375,8764,2584,8908,8703,2332,8854,
    5544,9189,7693,4712,8832,515,3048,7709,3797,1296,7111,7094,5796,6641,8237,4787,
    5118,4299,9309,2643,434,9806,2498,4614,9683,4656,6721,4117,5680,9244,5441,4253,
    8441,9916,7601,8849,4576,872,374,2778,3809,2865,2959,5314,493,1866,9688,3709,
    9202,2392,1655,4504,1364,7977,3423,7742,3863,6692,5563,3155,3753,313,4464,9466,
    339,2745,2837,1456,8461,6914,8700,4653,4664,2640,9115,4762,2814,6662,3456,2540,
    3320,5751,1446,3156,16,6643,4710,7298,1769,8043,2342,3440,9086,2583,1430,6441,
    5253,7039,6341,316,4395,8307,6990,2755,1366,8108,8044,5319,1771,4595,9630,348,
    4661,2873,4465,4615,7519,6416,6614,8882,2053,8736,4563,1268,2131,2062,1837,
    6223,8059,4645,1912,9126,7882,388,5675,9915,2964,4272,5202,2409,7617,9593,291,
    3444,756,1404,8239,4546,2917,5483,8704,2382,7824,6156,832,2654,7523,4258,9358,
    5962,4345,6383,6136,3362,7045,7666,2420,8506,7917,2097,1370,9755,9235,3098,
    3195,2296,4377,9614,3077,2027,8472,3547,4136,7088,6654,49,7532,5446,2153,2610,
    5614,8149,1601,7671,1646,4851,5565,1250,163,7798,3164,8532,5811,5136,9497};

// TODO: move this into the class for goodness sake.
#ifdef BALL_DETECTOR_TIMINGS
#include "utils/Timer.hpp"
#define BALL_DETECTOR_TIMINGS_RECORD_FRAMES 1000
    static int frame_time = 0;
    static int frame_count = 0;
    static int roi_time = 0;
    static int roi_count = 0;
    static int blob_roi_time = 0;
    static int blob_roi_count = 0;
    static int inspect_ball_time = 0;
    static int inspect_ball_count = 0;
    static int check_partial_region_time = 0;
    static int check_partial_region_count = 0;
    static int get_average_brightness_time = 0;
    static int get_average_brightness_count = 0;
    static int calculate_adaptive_values_for_circle_fitting_time = 0;
    static int calculate_adaptive_values_for_circle_fitting_count = 0;
    static int calculate_adaptive_values_for_internal_regions_time = 0;
    static int calculate_adaptive_values_for_internal_regions_count = 0;
    static int pre_process_adaptive_time = 0;
    static int pre_process_adaptive_count = 0;
    static int get_circle_candidate_points_time = 0;
    static int get_circle_candidate_points_count = 0;
    static int find_circle_kenji_time = 0;
    static int find_circle_kenji_count = 0;
    static int process_internal_regions_time = 0;
    static int process_internal_regions_count = 0;
    static int find_region_triangles_time = 0;
    static int find_region_triangles_count = 0;
    static int analyse_internal_regions_time = 0;
    static int analyse_internal_regions_count = 0;
    static int analyse_internal_regions_total_time = 0;
    static int analyse_internal_regions_total_count = 0;
    static int analyse_internal_region_triangles_time = 0;
    static int analyse_internal_region_triangles_count = 0;
    static int gmm_classifier_time = 0;
    static int gmm_classifier_count = 0;
    static int circle_fit_iterations = 0;
    static int circle_fit_max_iterations = 0;
    Timer timer;
    Timer timer2;
    Timer timer3;
    Timer frame_timer;
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_TO_FILE
    static int num_images = 1;
#endif //BALL_TO_FILE

/* Given a region, determine if we want to zoom in.
 Returns true/false depending on if a new region is created. */
void BallDetector::regenerateRegion(BallDetectorVisionBundle &bdvb, bool aspectCheck){
    // If the region is wider than it is tall, expand it over the bottom to
    // ensure the inclusion of any extra bottom area.

    float aspect = (float)(bdvb.region->getCols()) / (float)(bdvb.region->getRows());

    BBox newBounds = bdvb.region->getBoundingBoxRel();

    // if aspectCheck is false, we don't do one and go into the if statement
    if (aspect > 1.0 && (!aspectCheck || aspect < 4.0)) {
        int excess_width = bdvb.region->getCols() - bdvb.region->getRows();

        // Don't let it extend too far
        // Limit it by our ball size estimate

        excess_width = std::min(excess_width, (int) (bdvb.diam_expected_size_pixels / bdvb.region->getDensity()));
        //std::cout << "excess width: " << excess_width << " rows: " << bdvb.region->getRows()
            //<< " cols: " << bdvb.region->getCols() << "\n";

        // Valid coordinate checking is done inside Region
        newBounds.b.y() += excess_width;

        //std::cout << "Newbounds B: " << newBounds.b.y() << "\n";
    }

    // Add padding

    //int width_padding = (newBounds.b.x() - newBounds.a.x()) * 0.1;
    //int height_padding = (newBounds.b.y() - newBounds.a.y()) * 0.2;
    //int raw_pixel_padding = 5;

    //int width_padding = raw_pixel_padding / region->getDensity();
    //int height_padding = raw_pixel_padding / region->getDensity();

    int width_padding = PADDING_WIDTH;
    int height_padding = PADDING_HEIGHT;

    newBounds.a.x() -= width_padding;
    newBounds.b.x() += width_padding;
    newBounds.a.y() -= height_padding;
    newBounds.b.y() += height_padding;

    //std::cout << "width: " << width_padding << " height: " << height_padding << "\n";

    RegionI *region = new RegionI(*bdvb.region, BBox(newBounds.a, newBounds.b), 1, 0, true);
    if (bdvb.region_created == true){
        delete bdvb.region;
    }

    bdvb.region = region;
    bdvb.region_created = true;
}

/* Given a region and the candidate points we generate for circle fit,
trim the empty sections above, left and to the right of our candidate points. */
void BallDetector::trimRegionBasedOnCircleCandidatePoints(BallDetectorVisionBundle &bdvb){
    BBox newBounds = bdvb.region->getBoundingBoxRel();

    int min_x = bdvb.region->getCols();
    int max_x = 0;
    int min_y = bdvb.region->getRows();

    for (std::vector<Point>::iterator it = bdvb.circle_fit_points.begin(); it != bdvb.circle_fit_points.end(); it++) {
        min_x = std::min(min_x, it->x());
        max_x = std::max(max_x, it->x());
        min_y = std::min(min_y, it->y());
    }

    // Give a one pixel buffer
    min_x -= 1;
    max_x += 1;
    min_y -= 1;

    for (std::vector<Point>::iterator it = bdvb.circle_fit_points.begin(); it != bdvb.circle_fit_points.end(); it++) {
        it->x() -= min_x;
        it->y() -= min_y;
    }

    //std::cout << "Min_x: " << min_x << " Max_x: " << max_x << " Min_y: " << min_y << "\n";

    newBounds.a.x() = min_x;
    newBounds.b.x() = max_x;
    newBounds.a.y() = min_y;

    RegionI *region = new RegionI(*bdvb.region, BBox(newBounds.a, newBounds.b));
    if (bdvb.region_created == true){
        delete bdvb.region;
    }

    bdvb.region = region;
    bdvb.region_created = true;
}

// Regenerate region.
void BallDetector::regenerateRegionFromCircleFit(BallDetectorVisionBundle &bdvb, RANSACCircle &circle){
        // TODO - need to change circlefit radius and centre after regenereation and rescale
    BBox newBounds = BBox();

    newBounds.a = Point(circle.centre.x() - circle.radius, circle.centre.y() - circle.radius);
    newBounds.b = Point(circle.centre.x() + circle.radius, circle.centre.y() + circle.radius);

    newBounds.a.x() = std::max(newBounds.a.x(), bdvb.region->getBoundingBoxRel().a.x());
    newBounds.a.y() = std::min(newBounds.a.y(), bdvb.region->getBoundingBoxRel().a.y());
    newBounds.b.x() = std::max(newBounds.b.x(), bdvb.region->getBoundingBoxRel().b.x());
    newBounds.b.y() = std::min(newBounds.b.y(), bdvb.region->getBoundingBoxRel().b.y());

    int width_padding = PADDING_WIDTH;
    int height_padding = PADDING_HEIGHT;

    newBounds.a.x() -= width_padding;
    newBounds.b.x() += width_padding;
    newBounds.a.y() -= height_padding;
    newBounds.b.y() += height_padding;

    bdvb.region_created = true;

    int left_diff = bdvb.region->getBoundingBoxRel().a.x() - newBounds.a.x();
    int top_diff = bdvb.region->getBoundingBoxRel().a.y() - newBounds.a.y();

#ifdef BALL_DEBUG
    std::cout << "left_diff: " << left_diff << "\n";
    std::cout << "top_diff: " << top_diff << "\n";

    std::cout << "newbounds a: " << newBounds.a.x() << ", " << newBounds.a.y() << "\n";
    std::cout << "newbounds b: " << newBounds.b.x() << ", " << newBounds.b.y() << "\n";
#endif // BALL_DEBUG

    for (std::vector<Point>::iterator it = bdvb.circle_fit_points.begin();
            it != bdvb.circle_fit_points.end(); ) {
        it->x() += left_diff;
        it->y() += top_diff;

        if (it->x() < 0 || it->x() >= newBounds.width() || it->y() < 0 || it->y() >= newBounds.height()) {
            bdvb.circle_fit_points.erase(it);
        }
        else {
            it++;
        }
    }

    bdvb.circle_fit.result_circle.centre.x() += left_diff;
    bdvb.circle_fit.result_circle.centre.y() += top_diff;

    RegionI *region = new RegionI(*bdvb.region, BBox(newBounds.a, newBounds.b));
    if (bdvb.region_created == true){
        delete bdvb.region;
    }
    bdvb.region = region;
    bdvb.region_created = true;

}

/*
Determines whether the region is just a simple white blob, or actually contains
internal dots that could be ball spots.
*/
bool BallDetector::checkSimpleBlob(BallDetectorVisionBundle& bdvb)
{
    // The number of rows and columns in the region.
    const int rows = bdvb.region->getRows();
    const int cols = bdvb.region->getCols();

    // The current pixel.
    RegionI::iterator_fovea curPixel = bdvb.region->begin_fovea();

    // The minimum size of a white section.
    int minSectionSize = std::max((int)(MIN_SECTION_SIZE_PORTION * cols), 1);

    // Whether this is a valid ball candidate.
    bool ballCandidate = false;

    // Run through all rows.
    for(int row=0; !ballCandidate && row<rows; ++row)
    {
        // Variables to track the white blocks before and after any black
        // pixels.
        int start = -1;
        int stop = -1;

        // The size of the current white section.
        int sectionSize = 0;

        // The largest and second largest white section.
        int largestWhite = 0;
        int secondLargestWhite = 0;

        // The current column.
        int col = 0;

        // Whether we're done with this row.
        bool done = false;

        // Run through this row.
        while(!done)
        {
            // Reset start and stop.
            start = -1;
            stop = -1;

            // Looking for the first white pixel.
            for(; col<cols; ++col)
            {
                // Record the first white pixel.
                if(curPixel.colour() == cWHITE)
                {
                    start = col;
                    break;
                }

                // Move on to the next pixel.
                ++curPixel;
            }

            // Looking for a follow up black pixel.
            for(; col<cols; ++col)
            {
                // Record the last white pixel.
                if(curPixel.colour() != cWHITE)
                {
                    stop = col;
                    break;
                }

                // Move on to the next pixel.
                ++curPixel;
            }

            // If we've hit the end, then stop is here.
            if(start != -1 && col>=cols)
                stop = col;

            // Update the section size record.
            sectionSize = stop-start;
            if(sectionSize > largestWhite)
            {
                secondLargestWhite = largestWhite;
                largestWhite = sectionSize;
            }
            else if(sectionSize > secondLargestWhite)
                secondLargestWhite = sectionSize;

            // Check if this row makes a valid ball candidate.
            if(secondLargestWhite >= minSectionSize)
            {
                ballCandidate = true;
                done = true;
            }

            // If we've reached the end of the row, we're done.
            if(col >= cols)
                done = true;
        }
    }

#ifdef BALL_DEBUG
    std::cout << "Simple Blob: " << !ballCandidate << std::endl;
#endif // BALL_DEBUG

    // Return whether this is a valid ball candidate.
    return(ballCandidate);
}

bool BallDetector::checkPartialRegion(BallDetectorVisionBundle& bdvb){
    bdvb.is_partial_region = false;
    int num_rows = bdvb.region->isTopCamera() ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
    int num_cols = bdvb.region->isTopCamera() ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    if (bdvb.region->getBoundingBoxRaw().a.x() == 0){
        bdvb.is_partial_region = true;
        bdvb.partial_ball_side = BALL_SIDE_LEFT;
    } else if (bdvb.region->getBoundingBoxRaw().a.y() == 0){
        bdvb.is_partial_region = true;
        bdvb.partial_ball_side = BALL_SIDE_TOP;
    } else if (bdvb.region->getBoundingBoxRaw().b.x() == num_cols){
        bdvb.is_partial_region = true;
        bdvb.partial_ball_side = BALL_SIDE_RIGHT;
    } else if (bdvb.region->getBoundingBoxRaw().b.y() == num_rows){
        bdvb.is_partial_region = true;
        bdvb.partial_ball_side = BALL_SIDE_BOTTOM;
    }
    else {
        bdvb.partial_ball_side = BALL_SIDE_TOTAL;
    }

#ifdef BALL_DEBUG
    std::cout << "isPartial: " << bdvb.is_partial_region << " partial_ball_side: " <<
        bdvb.partial_ball_side << "\n";
#endif // BALL_DEBUG

    return bdvb.is_partial_region;
}

/* Clear the ball detector vision bundles */
static void clearBDVBs(std::vector <BallDetectorVisionBundle> &bdvbs){

        for (std::vector <BallDetectorVisionBundle>::iterator it = bdvbs.begin();
                it != bdvbs.end(); it++) {

            if (it->region_created) {
                delete it->region;
            }
            if (it->model_region_created){
                delete it->modelRegion;
            }
        }
}

#ifdef BALL_DETECTOR_TIMINGS
// print timing statistics PER FRAME, averaged over BALL_DETECTOR_TIMINGS_RECORD_FRAMES frames.
static inline void printTimings(std::string title, int time, int count){
    std::cout << std::setprecision(5) << std::setw(35) << title <<
    "| Total Calls: " << std::setw(6) << (float)count / frame_count <<
    "| Total Time (ms): " << std::setw(6) << (float)time / frame_count <<
    "| Time / Call (us): " << std::setw(6) << (float)time / count * 1000 <<
    "| % Time: " << std::setw(7) << (float)time / frame_time * 100 <<
    "| Time / region: " << std::setw(3) << (float)time / roi_count << std::endl;
}
#endif // BALL_DETECTOR_TIMINGS

////////////////////////////////////////////////////////////////////////////////
////////////////////// ENTRY POINT TO THE BALL DETECTOR ////////////////////////
////////////////////////////////////////////////////////////////////////////////
void BallDetector::detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
    // If you are the goalie and it is looking over its shoulder, don't let it detect balls

    // Dear goalie (player 1), please do not look over your shoulder outside the field for balls.
    // 70 degrees
    if (info_in.my_player_number == 1 && fabs(info_in.cameraToRR.values.joints.angles[Joints::HeadYaw]) > M_PI_2 * 7/9) {
        return;
    }

    const std::vector<RegionI> &regions = info_middle.roi;

    unsigned int region_index = regions.size() - 1;
    unsigned int subregion_index = 0;
#ifdef BALL_DETECTOR_TIMINGS
    frame_timer.restart();
#endif // BALL_DETECTOR_TIMINGS

    for (std::vector<RegionI>::const_reverse_iterator rit = regions.rbegin();
        rit != regions.rend(); ++rit, region_index--)
    {
        // This is our internal ROI

        std::vector <BallDetectorVisionBundle> ball_regions;
#ifdef BALL_DETECTOR_USES_VDM
        if (vdm != NULL) {
            VisionDebugQuery q = vdm->getQuery();
            if (region_index == q.region_index) {
                vdm->vision_debug_blackboard.values["Draw This Region"] = 1;
            } else {
                vdm->vision_debug_blackboard.values["Draw This Region"] = 0;
            }
        }
#endif // BALL_DETECTOR_USES_VDM

#ifdef BALL_DETECTOR_TIMINGS
        timer.restart();
#endif // BALL_DETECTOR_TIMINGS

        comboROI(info_in, *rit, info_middle, info_out, true, ball_regions);

#ifdef BALL_DETECTOR_TIMINGS
        roi_count++;
        roi_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

        subregion_index = 1;
        //std::cout << "SUB_BALLS " << ball_regions.size() << std::endl;
        for (std::vector <BallDetectorVisionBundle>::iterator it = ball_regions.begin();
                it != ball_regions.end(); it++, subregion_index++) {

#ifdef BALL_DETECTOR_TIMINGS
            timer.restart();
#endif // BALL_DETECTOR_TIMINGS
            bool isBall = inspectBall(*it);
#ifdef BALL_DETECTOR_TIMINGS
            inspect_ball_count++;
            inspect_ball_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

            if (isBall){

#ifdef BALL_DEBUG
                std::cout << "FOUND BALL" << std::endl;
                //std::cout << "ball.rr.distance(): " << it->ball.rr.distance() << " ball.rr.heading(): " << it->ball.rr.heading() << std::endl;
                std::cout << "ball.imageCoords[0]: " << it->ball.imageCoords[0] << " ball.imageCoords[1]: " << it->ball.imageCoords[1] << std::endl;
                std::cout << "ball.radius: " << it->ball.radius << std::endl;
                std::cout << "ball.neckRelative.x: " << it->ball.neckRelative.x << " ball.neckRelative.y: " << it->ball.neckRelative.y << " ball.neckRelative.z: " << it->ball.neckRelative.z << std::endl;
                std::cout << "ball.topCamera: " << it->ball.topCamera << std::endl;
                std::cout << "ball.visionVar: " << it->ball.visionVar << std::endl;
#endif // BALL_DEBUG
#ifdef BALL_TO_FILE
                char name[] = "a";
                char location[] = BALL_TO_FILE_DIR;
                std::stringstream dir;
                dir << location << num_images << ".png";
                num_images++;
                std::cout << "Writing binary ball image to: " << dir.str().c_str() << "\n";
                WriteImage w;
                if(w.writeImage(*it->region, COLOUR_FORMAT, dir.str().c_str(), name)) {
                    std::cout << "Success\n";
                }
                else {
                    std::cout << "Failed\n";
                }
#endif  // BALL_TO_FILE

                // Set the ball back to the region
                Point center = it->region->getInternalFovea()->mapFoveaToImage(Point(it->circle_fit.result_circle.centre.x(),it->circle_fit.result_circle.centre.y()));
                it->ball.imageCoords.x() = center.x();
                it->ball.imageCoords.y() = center.y();

                it->ball.topCamera = (it->region->isTopCamera());
                it->ball.radius = (it->circle_fit.result_circle.radius * it->region->getDensity());

                last_ball_distance_ = it->ball.rr.distance();
                info_out.balls.push_back(it->ball);
#ifdef EARLY_EXIT
// We don't want to early exit while using vdm
#ifdef BALL_DETECTOR_USES_VDM
                if (vdm == NULL) {
                    clearBDVBs(ball_regions);
                    return;
                }
#else
                clearBDVBs(ball_regions);
                last_normal_ball_ = 0;
                return;
#endif // BALL_DETECTOR_USES_VDM
#endif // EARLY_EXIT
            }
#ifdef BALL_DETECTOR_USES_VDM
            if (vdm != NULL) {
                VisionDebugQuery q = vdm->getQuery();
                std::cout << "REGION " << region_index << "/" << q.region_index << std::endl;
                std::cout << "SUBREGION " << subregion_index << "/" << q.subregion_index << std::endl;
                if (region_index == q.region_index && subregion_index == q.subregion_index) {
                    it->drawBall();
                }
            }
#endif // BALL_DETECTOR_USES_VDM
        }
        // If inspect fails and the region aspect was deemed to be normal
        // we should let blobROI find the ball and rerun detect() on the
        // regions created by blobROI.
        for (std::vector <BallDetectorVisionBundle>::iterator it = ball_regions.begin();
        it != ball_regions.end(); it++) {
            if (it->region_aspect_type == NORMAL) {

#ifdef BALL_DEBUG
                std::cout << "Normal Region failed, retrying with blobROI\n";
#endif //BALL_DEBUG

                std::vector <BallDetectorVisionBundle> ball_regions_blob;
#ifdef BALL_DETECTOR_TIMINGS
                timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
                blobROI(info_in, *it->original_region, info_middle, info_out, ball_regions_blob, it->region_aspect_type);
#ifdef BALL_DETECTOR_TIMINGS
                blob_roi_count++;
                blob_roi_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

                unsigned int subregion_index_blob = 1;
                for (std::vector <BallDetectorVisionBundle>::iterator itb = ball_regions_blob.begin();
                itb != ball_regions_blob.end(); itb++, subregion_index_blob++) {
#ifdef BALL_DETECTOR_TIMINGS
                    timer.restart();
#endif // BALL_DETECTOR_TIMINGS
                    bool isBall = inspectBall(*itb);
#ifdef BALL_DETECTOR_TIMINGS
                    inspect_ball_count++;
                    inspect_ball_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

                    if (isBall){

#ifdef BALL_DEBUG
                        std::cout << "FOUND BALL" << std::endl;
                        //std::cout << "ball.rr.distance(): " << it->ball.rr.distance() << " ball.rr.heading(): " << it->ball.rr.heading() << std::endl;
                        std::cout << "ball.imageCoords[0]: " << itb->ball.imageCoords[0] << " ball.imageCoords[1]: " << itb->ball.imageCoords[1] << std::endl;
                        std::cout << "ball.radius: " << itb->ball.radius << std::endl;
                        std::cout << "ball.neckRelative.x: " << itb->ball.neckRelative.x << " ball.neckRelative.y: " << itb->ball.neckRelative.y << " ball.neckRelative.z: " << itb->ball.neckRelative.z << std::endl;
                        std::cout << "ball.topCamera: " << itb->ball.topCamera << std::endl;
                        std::cout << "ball.visionVar: " << itb->ball.visionVar << std::endl;
#endif // BALL_DEBUG
#ifdef BALL_TO_FILE
                        char name[] = "a";
                        char location[] = BALL_TO_FILE_DIR;
                        std::stringstream dir;
                        dir << location << num_images << ".png";
                        num_images++;
                        std::cout << "Writing binary ball image to: " << dir.str().c_str() << "\n";
                        WriteImage w;
                        if(w.writeImage(*itb->region, COLOUR_FORMAT, dir.str().c_str(), name)) {
                            std::cout << "Success\n";
                        }
                        else {
                            std::cout << "Failed\n";
                        }
#endif  // BALL_TO_FILE

                        // Set the ball back to the region
                        Point center = itb->region->getInternalFovea()->mapFoveaToImage(Point(itb->circle_fit.result_circle.centre.x(),itb->circle_fit.result_circle.centre.y()));
                        itb->ball.imageCoords.x() = center.x();
                        itb->ball.imageCoords.y() = center.y();

                        itb->ball.topCamera = (itb->region->isTopCamera());
                        itb->ball.radius = (itb->circle_fit.result_circle.radius * itb->region->getDensity());

                        last_ball_distance_ = itb->ball.rr.distance();
                        info_out.balls.push_back(itb->ball);
#ifdef EARLY_EXIT
        // We don't want to early exit while using vdm
#ifdef BALL_DETECTOR_USES_VDM
                        if (vdm == NULL) {
                            clearBDVBs(ball_regions_blob);
                            return;
                        }
#else
                        clearBDVBs(ball_regions_blob);
                        last_normal_ball_ = 0;
                        return;
#endif // BALL_DETECTOR_USES_VDM
#endif // EARLY_EXIT
                    }
#ifdef BALL_DETECTOR_USES_VDM
                    if (vdm != NULL) {
                        VisionDebugQuery q = vdm->getQuery();
                        std::cout << "REGION " << region_index << "/" << q.region_index << std::endl;
                        std::cout << "SUBREGION " << subregion_index_blob << "/" << q.subregion_index_blob << std::endl;
                        if (region_index == q.region_index && subregion_index_blob == q.subregion_index_blob) {
                            itb->drawBall();
                        }
                    }
#endif // BALL_DETECTOR_USES_VDM
                }
                clearBDVBs(ball_regions_blob);
            }
        }
        clearBDVBs(ball_regions);
    }

    // If we havn't exited there is no normal ball.
    ++last_normal_ball_;

    // The start of the crazy ball detector
    if (shouldRunCrazyBallDetector(info_in))
    {
        // Check if there is a lot of white in the bottom camera.
        int total_pixels = 0;
        for (std::vector<RegionI>::const_reverse_iterator rit = regions.rbegin();
            rit != regions.rend(); ++rit, region_index--)
        {
            if(!rit->isTopCamera())
                total_pixels += rit->getCols() * rit->getRows();
        }

        // If there is a lot of white in the bottom camera perform the expensive
        // ball check.
        if(total_pixels > BOT_SALIENCY_ROWS*BOT_SALIENCY_COLS*WHITE_HEAVY_THRESHOLD)
        {
           if(crazy_ball_cycle_ > 2)
            {
                ++crazy_ball_cycle_;
                if(crazy_ball_cycle_ >= 10)
                    crazy_ball_cycle_ = 0;
            }
            else
            {
                ++crazy_ball_cycle_;

                for (std::vector<RegionI>::const_reverse_iterator rit = regions.rbegin();
                    rit != regions.rend(); ++rit, region_index--)
                {
                    // Only run on bottom camera.
                    if(rit->isTopCamera())
                        break;

                    // This is our internal ROI. Doesn't this all look familiar?
                    std::vector <BallDetectorVisionBundle> ball_regions;
                    //blackROI(info_in, &regions[region], info_middle, info_out, true, ball_regions);
                    //circleROI(info_in, &regions[region], info_middle, info_out, true, ball_regions);
#ifdef BALL_DETECTOR_USES_VDM
                    if (vdm != NULL) {
                        VisionDebugQuery q = vdm->getQuery();
                        if (region_index == q.region_index) {
                            vdm->vision_debug_blackboard.values["Draw This Region"] = 1;
                        } else {
                            vdm->vision_debug_blackboard.values["Draw This Region"] = 0;
                        }
                    }
#endif // BALL_DETECTOR_USES_VDM

#ifdef BALL_DETECTOR_TIMINGS
                    timer.restart();
#endif // BALL_DETECTOR_TIMINGS
                    circleROI(info_in, *rit, info_middle, info_out, true, ball_regions);
#ifdef BALL_DETECTOR_TIMINGS
                        roi_count++;
                        roi_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

                    subregion_index = 1;
                    //std::cout << "SUB_BALLS " << ball_regions.size() << std::endl;
                    for (std::vector <BallDetectorVisionBundle>::iterator it = ball_regions.begin();
                            it != ball_regions.end(); it++, subregion_index++) {

#ifdef BALL_DETECTOR_TIMINGS
                        timer.restart();
#endif // BALL_DETECTOR_TIMINGS
                        bool isBall = inspectBall(*it);
#ifdef BALL_DETECTOR_TIMINGS
                        inspect_ball_count++;
                        inspect_ball_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

                        if (isBall){
                            //std::cout << "FOUND BALL dist: " << it->ball.rr.distance() << " heading: "
                                //<< RAD2DEG(it->ball.rr.heading()) << std::endl;
#ifdef BALL_DEBUG
                            std::cout << "FOUND BALL" << std::endl;
                            //std::cout << "ball.rr.distance(): " << rr.distance() << " ball.rr.heading(): " << rr.heading() << std::endl;
                            std::cout << "ball.imageCoords[0]: " << it->ball.imageCoords[0] << " ball.imageCoords[1]: " << it->ball.imageCoords[1] << std::endl;
                            std::cout << "ball.radius: " << it->ball.radius << std::endl;
                            std::cout << "ball.neckRelative.x: " << it->ball.neckRelative.x << " ball.neckRelative.y: " << it->ball.neckRelative.y << " ball.neckRelative.z: " << it->ball.neckRelative.z << std::endl;
                            std::cout << "ball.topCamera: " << it->ball.topCamera << std::endl;
                            std::cout << "ball.visionVar: " << it->ball.visionVar << std::endl;
#endif // BALL_DEBUG
                            //*/

                            // Set the ball back to the region
                            //it->ball.imageCoords.x() = (it->region->getBoundingBoxRaw().a.x() + it->region->getBoundingBoxRaw().b.x())/2;
                            //it->ball.imageCoords.y() = (it->region->getBoundingBoxRaw().a.y() + it->region->getBoundingBoxRaw().b.y())/2;
                            it->ball.imageCoords.x() = it->region->getBoundingBoxRaw().a.x() + it->circle_fit.result_circle.centre.x() * it->region->getDensity();
                            it->ball.imageCoords.y() = it->region->getBoundingBoxRaw().a.y() + it->circle_fit.result_circle.centre.y() * it->region->getDensity();
                            // Adjustment for the old system that continued the coordinate system for the bottom frame.
                            it->ball.topCamera = (it->region->isTopCamera());
                            if (!(it->region->isTopCamera())){
                                it->ball.imageCoords.y() += TOP_IMAGE_ROWS;
                            }
                            it->ball.radius = (it->circle_fit.result_circle.radius * it->region->getDensity());

                            last_ball_distance_ = it->ball.rr.distance();
                            info_out.uncertain_balls.push_back(it->ball);
                        }
#ifdef BALL_DETECTOR_USES_VDM
                        if (vdm != NULL) {
                            VisionDebugQuery q = vdm->getQuery();
                            std::cout << "REGION " << region_index << "/" << q.region_index << std::endl;
                            std::cout << "SUBREGION " << subregion_index << "/" << q.subregion_index << std::endl;
                            if (region_index == q.region_index && subregion_index == q.subregion_index) {
                                it->drawBall();
                            }
                        }
#endif // BALL_DETECTOR_USES_VDM
                    }
                    clearBDVBs(ball_regions);
                }
            }
        }
    }

#ifdef BALL_DETECTOR_TIMINGS
    frame_time += frame_timer.elapsed_us();

    if (frame_count == BALL_DETECTOR_TIMINGS_RECORD_FRAMES) {
        roi_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        blob_roi_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        inspect_ball_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        check_partial_region_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        get_average_brightness_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        calculate_adaptive_values_for_circle_fitting_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        calculate_adaptive_values_for_internal_regions_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        pre_process_adaptive_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        get_circle_candidate_points_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        find_circle_kenji_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        process_internal_regions_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        find_region_triangles_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        analyse_internal_regions_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        analyse_internal_regions_total_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        analyse_internal_region_triangles_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        gmm_classifier_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;
        frame_time /= BALL_DETECTOR_TIMINGS_RECORD_FRAMES;

        std::cout << "\n" << std::string(50, '=') << " BALL_DETECTOR_TIMINGS " << std::string(50, '=') << std::endl;

        std::cout << std::setw(35) << "FRAMES" <<
            "| Total: " << std::setw(6) << frame_count <<
            "| Time: " << std::setw(6) << frame_time <<
            "| %Time: " << std::setw(10) << 100.0 * frame_time / frame_time <<
            "| Time / frame: " << std::setw(3) << frame_time / frame_count << std::endl;

        printTimings("ROI", roi_time, roi_count);
        printTimings("BLOB ROI", blob_roi_time, blob_roi_count);
        std::cout << std::endl;
        printTimings("INSPECT BALL", inspect_ball_time, inspect_ball_count);
        std::cout << std::endl;
        printTimings("CHECK PARTIAL REGION", check_partial_region_time, check_partial_region_count);
        printTimings("GET AVERAGE BRIGHTNESS", get_average_brightness_time, get_average_brightness_count);
        printTimings("CALCULATE ADAPTIVE VALUES FOR CF", calculate_adaptive_values_for_circle_fitting_time, calculate_adaptive_values_for_circle_fitting_count);
        printTimings("CALCULATE ADAPTIVE VALUES FOR IR", calculate_adaptive_values_for_internal_regions_time, calculate_adaptive_values_for_internal_regions_count);
        printTimings("PRE-PROCESS ADAPTIVE", pre_process_adaptive_time, pre_process_adaptive_count);
        printTimings("GET CIRCLE CANDIDATE POINTS", get_circle_candidate_points_time, get_circle_candidate_points_count);
        printTimings("FIND CIRCLE KENJI", find_circle_kenji_time, find_circle_kenji_count);
        printTimings("PROCESS INTERNAL REGIONS", process_internal_regions_time, process_internal_regions_count);
        printTimings("FIND REGION TRIANGLES", find_region_triangles_time, find_region_triangles_count);
        printTimings("ANALYSE INTERNAL REGIONS", analyse_internal_regions_time, analyse_internal_regions_count);
        printTimings("ANALYSE INTERNAL REGIONS TOTAL", analyse_internal_regions_total_time, analyse_internal_regions_total_count);
        printTimings("ANALYSE INTERNAL REGION TRIANGLES", analyse_internal_region_triangles_time, analyse_internal_region_triangles_count);
        printTimings("GMM CLASSIFIER", gmm_classifier_time, gmm_classifier_count);

        roi_time = 0;
        roi_count = 0;
        blob_roi_time = 0;
        blob_roi_count = 0;
        inspect_ball_time = 0;
        inspect_ball_count = 0;
        check_partial_region_time = 0;
        check_partial_region_count = 0;
        get_average_brightness_time = 0;
        get_average_brightness_count = 0;
        calculate_adaptive_values_for_circle_fitting_time = 0;
        calculate_adaptive_values_for_circle_fitting_count = 0;
        calculate_adaptive_values_for_internal_regions_time = 0;
        calculate_adaptive_values_for_internal_regions_count = 0;
        pre_process_adaptive_time = 0;
        pre_process_adaptive_count = 0;
        get_circle_candidate_points_time = 0;
        get_circle_candidate_points_count = 0;
        find_circle_kenji_time = 0;
        find_circle_kenji_count = 0;
        process_internal_regions_time = 0;
        process_internal_regions_count = 0;
        find_region_triangles_time = 0;
        find_region_triangles_count = 0;
        analyse_internal_regions_time = 0;
        analyse_internal_regions_count = 0;
        analyse_internal_regions_total_time = 0;
        analyse_internal_regions_total_count = 0;
        analyse_internal_region_triangles_time = 0;
        analyse_internal_region_triangles_count = 0;
        gmm_classifier_time = 0;
        gmm_classifier_count = 0;
        frame_time = 0;
        frame_count = 0;
    }
    else {
        frame_count++;
    }
#endif // BALL_DETECTOR_TIMINGS
}

/* Given a region, check the aspect ratio. This is simply length/height. */
static inline double checkRegionAspectRatio(const RegionI& region, BallDetectorVisionBundle& bdvb){
    return (double) region.getCols()/region.getRows();
}

// ComboROI
// Routes the region of interest to the correct internal ROI finder based on its size and location.
bool BallDetector::comboROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res) {
#ifdef BALL_DEBUG
    std::cout << "comboROI\n";
#endif // BALL_DEBUG

    bool bdvbAdded = false;

    BallDetectorVisionBundle bdvb;
    bdvb.region = &region;
    bdvb.original_region = &region;
    bdvb.region_created = false;
    bdvb.model_region_created = false;
    bdvb.original_region_base_y_ = region.getBoundingBoxRaw().b.y()/region.getDensity();
    bdvb.is_crazy_ball = false;

    // Throw out regions above the field boundary.
    if (bdvb.region->isTopCamera() && (bdvb.region->getBoundingBoxRaw().a.y() <
            info_out.topStartScanCoords[bdvb.region->getBoundingBoxRaw().a.x()]))
    {
#ifdef BALL_DEBUG
        std::cout << "Field boundary reject: isTop: " << bdvb.region->isTopCamera()
            << "y: " << bdvb.region->getBoundingBoxRaw().a.y() << " fieldBoundary: "
            << info_out.topStartScanCoords[bdvb.region->getBoundingBoxRaw().a.x()] << "\n";
#endif //BALL_DEBUG
        return false;
    }

    if(!checkSimpleBlob(bdvb))
        return false;

    getSizeEst(bdvb, info_in, info_out);

#ifdef BALL_DEBUG
        std::cout << "Diam_size_est: " << bdvb.diam_size_est << "\n";
        std::cout << "Rows: " << bdvb.region->getRows() << " Cols: " << bdvb.region->getCols() << "\n";
#endif //BALL_DEBUG

    /* Sanity check for size */
    /* Checks the ball estimate size is reasonable, and the region is below the field boundary or on the bottom camera.*/
    //std::cout << "Diam size est " << bdvb.diam_size_est << " AND target bbox" << bdvb.region->getBoundingBoxRaw().a.y() << std::endl;
    if (bdvb.diam_size_est < 200 && bdvb.diam_size_est > 50) {
        /* Do a check for the ball's aspect ratio. Depending on the results, we may need to pass it into another
         * ROI like blobROI to locate the ball inside*/
        double aspect_ratio = checkRegionAspectRatio(region, bdvb);

#ifdef BALL_DEBUG
        std::cout << "aspect_ratio: " << aspect_ratio << "\n";
#endif //BALL_DEBUG

        if (aspect_ratio <= 0.5){
            // Too tall, but there might be a ball inside.
            // Blob ROI might be able to find it inside.

#ifdef BALL_DEBUG
            std::cout << "Tall\n";
#endif //BALL_DEBUG

#ifdef BALL_DETECTOR_TIMINGS
            timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
            bdvb.region_aspect_type = TALL;

            blobROI(info_in, region, info_middle, info_out, res, bdvb.region_aspect_type);

#ifdef BALL_DETECTOR_TIMINGS
            blob_roi_count++;
            blob_roi_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

        } else if (aspect_ratio >= 1.5) {
            // Too short/wide, squarify will fix this.
            // The let blobROI find the ball
            // We know that is the case, because the size check on width must have passed to get to this point.
            // TODO: Is this just the general case with a location check? Can't we do location check regardless?
            //

#ifdef BALL_DEBUG
            std::cout << "Short\n";
#endif //BALL_DEBUG

            // Squarify to include more potential blobs
            regenerateRegion(bdvb, true);
            bdvb.region_aspect_type = SHORT;
            
            blobROI(info_in, *bdvb.region, info_middle, info_out, res, bdvb.region_aspect_type);
            

        } else {
            // Roughly good in terms of size checks and ratio. This means we get a good region around
            // a ball if it exists.

#ifdef BALL_DEBUG
            std::cout << "Normal\n";
#endif //BALL_DEBUG

            regenerateRegion(bdvb, false);
            rescaleRegion(bdvb);
            bdvb.region_aspect_type = NORMAL;

            res.push_back(bdvb);
            bdvbAdded = true;

        }
    } else if (bdvb.diam_size_est > 40) {
        // When the width size check fails so the region is too wide initially to be a ball.
        // Ask blobROI to locate it inside.

#ifdef BALL_DETECTOR_TIMINGS
        timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
        bdvb.region_aspect_type = UNDEFINED;
        blobROI(info_in, region, info_middle, info_out, res, bdvb.region_aspect_type);

#ifdef BALL_DETECTOR_TIMINGS
        blob_roi_count++;
        blob_roi_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS
    } else {
        // Diam size estimate for the ball is less than 4cm, so it is likely to be noise on the field.
        // Ignore these regions.

    }
#ifdef BALL_DETECTOR_USES_VDM
    if (vdm != NULL && vdm->vision_debug_blackboard.values["Draw This Region"] == 1) {
        bdvb.drawBall();
    }
#endif // BALL_DETECTOR_USES_VDM
    return bdvbAdded;
}

// Black ROI
// Finds the black patch inside the big ROI and forms a ball candidate around that.
bool BallDetector::blackROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res) {

#ifdef BALL_DEBUG
    std::cout << "blackROI\n";
#endif // BALL_DEBUG

    bool bdvbAdded = false;

    BallDetectorVisionBundle bdvb;
    bdvb.region = &region;
    bdvb.original_region = &region;
    bdvb.region_created = false;
    bdvb.model_region_created = false;
    bdvb.is_crazy_ball = false;

    getSizeEst(bdvb, info_in, info_out);

    //std::cout << "rows: " << bdvb.region->getRows() << " cols: " << bdvb.region->getCols() << "\n";
    regenerateRegion(bdvb, false);

    // Basic
    RegionI::iterator_raw cur_point = bdvb.region->begin_raw();

    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;

    // The number of rows and columns in the region.
    int rows = bdvb.region->getRows();
    int cols = bdvb.region->getCols();

    //std::cout << "rows: " << rows << " cols: " << cols << "\n";

    int min_x = 0;
    int max_x = 0;
    int min_y = 0;
    int max_y = 0;
    bool first = true;

    ColourClassifier* colour_classifier;

    if (region.isTopCamera()) {
        colour_classifier = info_middle.colour_classifier_top_;
    }
    else {
        colour_classifier = info_middle.colour_classifier_bot_;
    }

    int darkest_pixel = 255;

    for(int pixel=0; pixel < cols*rows; ++pixel)
    {
        if (*cur_point.raw() < darkest_pixel) {
            //std::cout << x << ", " << y << "\n";
            min_x = x;
            max_x = x;
            min_y = y;
            max_y = y;

            first = false;
            darkest_pixel = *(cur_point.raw());

        }

        ++cur_point;
        ++x;
        if(x == cols)
        {
            x = 0;
            ++y;
        }
    }

    if (!first) {
        int bottom_pad = 2;

        // If the size of our black region is bigger than the expected size of the ball, then stop
        if (max_x - min_x > bdvb.diam_expected_size || max_y - min_y > bdvb.diam_expected_size) {
#ifdef BALL_DEBUG
    std::cout << "BlackROI reject\n";
    std::cout << "max_x - min_x: " << max_x - min_x << " max_y - min_y: " << max_y - min_y <<
        " diam_expected_size: " << bdvb.diam_expected_size << "\n";
#endif // BALL_DEBUG
            return bdvbAdded;
        }

        Point middle_bottom = Point(0.5 * (min_x + max_x), (double) max_y);
        //Point middle_bottom = Point(0.5 * (min_x + max_x), (double) bdvb.region->getRows());
        //std::cout << "MiddleBottom: " << 0.5 * (min_x + max_x) << "," << max_y << "\n";
        //std::cout << "Rows: " << bdvb.region->getRows() << ", Cols: " << bdvb.region->getCols() << "\n";

        BBox newBounds = BBox();
        newBounds.a.x() = middle_bottom.x() - 0.6 * bdvb.diam_expected_size;
        newBounds.a.y() = middle_bottom.y() - bdvb.diam_expected_size;
        newBounds.b.x() = middle_bottom.x() + 0.6 * bdvb.diam_expected_size;
/*
        newBounds.a.x() = middle_bottom.x() - 0.6 * (bdvb.diam_expected_size_pixels / bdvb.region->getDensity());
        newBounds.a.y() = middle_bottom.y() - (bdvb.diam_expected_size_pixels / bdvb.region->getDensity());
        newBounds.b.x() = middle_bottom.x() + 0.6 * (bdvb.diam_expected_size_pixels / bdvb.region->getDensity());
        */
        newBounds.b.y() = middle_bottom.y() + bottom_pad;

        // If most of the new region does not intersect with the old region, then reject

        int intersecting_width = std::min(newBounds.b.x(), bdvb.region->getBoundingBoxRel().b.x()) -
            std::max(newBounds.a.x(), bdvb.region->getBoundingBoxRel().a.x());
        int intersecting_height = std::min(newBounds.b.y(), bdvb.region->getBoundingBoxRel().b.y()) -
            std::max(newBounds.a.y(), bdvb.region->getBoundingBoxRel().a.y());
        int intersecting_area = intersecting_height * intersecting_width;
        int new_region_area = newBounds.width() * newBounds.height();

        if (2 * intersecting_area < new_region_area || new_region_area == 0) {
#ifdef BALL_DEBUG
            std::cout << "BlackROI reject\n";
            std::cout << "Intersecting area prop: " << (double) intersecting_area / new_region_area << "\n";
            std::cout << "New region area: " << new_region_area << "\n";
#endif // BALL_DEBUG
            return bdvbAdded;
        }

        RegionI *region = new RegionI(*bdvb.region, BBox(newBounds.a, newBounds.b));

        if (bdvb.region_created == true){
            delete bdvb.region;
        }

        bdvb.region = region;
        bdvb.region_created = true;
        bdvbAdded = true;

        rescaleRegion(bdvb);

        res.push_back(bdvb);
    }
#ifdef BALL_DEBUG
    else {
        std::cout << "No black pixels for blackROI\n";
    }
#endif // BALL_DEBUG

    return bdvbAdded;
}

bool BallDetector::blobROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out, std::vector <BallDetectorVisionBundle> &res, RegionAspectType region_aspect_type) {
#ifdef BALL_DEBUG
            std::cout << "blobROI\n";
#endif // BALL_DEBUG

    bool bdvbAdded = false;

    // If we don't want to use it

    BallDetectorVisionBundle temp_bdvb;
    temp_bdvb.region = &region;
    temp_bdvb.original_region = &region;
    temp_bdvb.region_created = false;
    temp_bdvb.model_region_created = false;
    temp_bdvb.is_crazy_ball = false;

    //Copy RegionAspectType from comboROI
    temp_bdvb.region_aspect_type = region_aspect_type;

    // Amount to expand each region by
    int expand_by;

    getAverageBrightness(temp_bdvb);
    calculateAdaptiveValuesForBlobROI(temp_bdvb);
    preProcessAdaptive(temp_bdvb);

    processInternalRegionsROI(*temp_bdvb.region, temp_bdvb, temp_bdvb.internal_regions);

#ifdef BLOBROI_DEBUG
    std::vector<Point> region_centres;
    for (unsigned int i = 0; i < temp_bdvb.internal_regions.groups.size(); i++) {
        Point centre(((temp_bdvb.internal_regions.groups[i].max_x - temp_bdvb.internal_regions.groups[i].min_x + 1) / 2) + temp_bdvb.internal_regions.groups[i].min_x,
                     ((temp_bdvb.internal_regions.groups[i].max_y - temp_bdvb.internal_regions.groups[i].min_y + 1) / 2) + temp_bdvb.internal_regions.groups[i].min_y);
        region_centres.push_back(centre);
    }
    std::cout << "------------------- Old region centres --------------------\n";
    printRegionsAndCentres(*temp_bdvb.region, region_centres);
#endif //BLOBROI_DEBUG

    // Number of blobs we have to work with
    int number_of_blobs = temp_bdvb.internal_regions.groups.size();

    // If 1 or more but less than 6 regions found create think about making a bdvb
    if (number_of_blobs > 0 && number_of_blobs < 7) {
        // Collect data about the most dense region. This assumes that this dense region is a ball blob (bb)
        // and that other bb will have similar size.
        // Comparison is done using the larger of the width or the height
        // as other bb will have a height or width similar

        int dense_reg_num = temp_bdvb.internal_regions.max_region_density_number - 1;        // -1 so it accesses the correct data in the vector
        int bb_size_x = temp_bdvb.internal_regions.groups[dense_reg_num].max_x -
                        temp_bdvb.internal_regions.groups[dense_reg_num].min_x + 1;          // +1 so the size is correct
        int bb_size_y = temp_bdvb.internal_regions.groups[dense_reg_num].max_y -
                        temp_bdvb.internal_regions.groups[dense_reg_num].min_y + 1;

        int blob_max_size = std::max(bb_size_x, bb_size_y);
        int blob_min = floor(blob_max_size * 0.7);
        int blob_max = ceil(blob_max_size * 1.3);

#ifdef BALL_DEBUG
        std::cout << "Dense region number: " << temp_bdvb.internal_regions.max_region_density_number

                  << " - width: " << bb_size_x << " height: " << bb_size_y << "\n";
        std::cout << "Allowable blob min: " << blob_min << " max: " << blob_max << "\n";
#endif // BALL_DEBUG

        // Find the region centres using the approx bb to throw out regions when we have more than 2
        std::vector<Point> region_centres;
        region_centres.reserve(8);
            for (std::vector<InternalRegion>::iterator it = temp_bdvb.internal_regions.groups.begin(); it != temp_bdvb.internal_regions.groups.end(); ++it) {
                int min_x = it->min_x;
                int min_y = it->min_y;
                int x_size = it->max_x - min_x + 1;         // +1 so the size is correct
                int y_size = it->max_y - min_y + 1;

                    // If more than 2 blobs then try to throw some out using the dense blob
                if (number_of_blobs > 2) {
                    // If the height or width of the blob are within the bounds of the likely ball blob store its centre
                    if (((x_size > blob_min) && (x_size < blob_max)) || ((y_size > blob_min) && (y_size < blob_max))) {
                        Point centre((x_size / 2) + min_x, (y_size / 2) + min_y);
                        region_centres.push_back(centre);
#ifdef BALL_DEBUG
                    std::cout << "Accepted blob - x size: " << x_size << " Y size: " << y_size << "\n";
                    }
                    else {
                        std::cout << "Rejected X size: " << x_size << " Y size: " << y_size << "\n";
#endif //BALL_DEBUG
                    }
                }
                // Don't throw any out
                else {
                    Point centre((x_size / 2) + min_x, (y_size / 2) + min_y);
                    region_centres.push_back(centre);
                }
            }
#ifdef BLOBROI_DEBUG
        std::cout << "------------------- New region centres --------------------\n";
        printRegionsAndCentres(*temp_bdvb.region, region_centres);
#endif //BLOBROI_DEBUG

        for (std::vector<Point>::iterator it = region_centres.begin(); it != region_centres.end(); ++it) {

            // If we don't want to use it
            BallDetectorVisionBundle bdvb;
            bdvb.region = &region;
            bdvb.original_region = &region;
            bdvb.region_created = false;
            bdvb.model_region_created = false;
            bdvb.is_crazy_ball = false;

            //Copy RegionAspectType from comboROI
            bdvb.region_aspect_type = region_aspect_type;

            // Blob is likely to be to the side of the ball so expand by x 2.5
            // unless it is the only blob and probably centred so expand by x 2
            if (region_centres.size() == 1) {
                expand_by = blob_max_size * 2;
            }
            else {
                expand_by = blob_max_size * 2.5;
            }

            BBox newBounds = BBox();
            newBounds.a.x() = it->x() - expand_by;
            newBounds.a.y() = it->y() - expand_by;
            newBounds.b.x() = it->x() + expand_by;
            newBounds.b.y() = it->y() + expand_by;

            BBox old = region.getBoundingBoxRaw();

            RegionI *blob_region = new RegionI(region, BBox(newBounds.a, newBounds.b), 1, 0, true);

            if (bdvb.region_created == true){
                delete bdvb.region;
            }

            bdvb.region = blob_region;
            bdvb.region_created = true;
            bdvbAdded = true;
            getSizeEst(bdvb, info_in, info_out);
            rescaleRegion(bdvb);
            res.push_back(bdvb);
        }
        // Push back extra bdvb using regions posistions to pick the likely centre
        if (region_centres.size() == 2) {

            // If we don't want to use it
            BallDetectorVisionBundle bdvb;
            bdvb.region = &region;
            bdvb.original_region = &region;
            bdvb.region_created = false;
            bdvb.model_region_created = false;
            bdvb.is_crazy_ball = false;

            //Copy RegionAspectType from comboROI
            bdvb.region_aspect_type = region_aspect_type;

            // If we have two regions use the midpoint
            Point midpoint = Point(((region_centres[0].x() + region_centres[1].x()) / 2),
                                   ((region_centres[0].y() + region_centres[1].y()) / 2));

            // Likely to be closer to the actual centre of the ball so expand by x 2
            int expand_by = blob_max_size * 2.0;

            BBox newBounds = BBox();
            newBounds.a.x() = midpoint.x() - expand_by;
            newBounds.a.y() = midpoint.y() - expand_by;
            newBounds.b.x() = midpoint.x() + expand_by;
            newBounds.b.y() = midpoint.y() + expand_by;

            RegionI *blob_region = new RegionI(region, BBox(newBounds.a, newBounds.b), 1, 0, true);

            if (bdvb.region_created == true){
                delete bdvb.region;
            }

            bdvb.region = blob_region;
            bdvb.region_created = true;
            bdvbAdded = true;
            getSizeEst(bdvb, info_in, info_out);

            rescaleRegion(bdvb);
            res.push_back(bdvb);
        }
        if (region_centres.size() == 3) {

            // If we don't want to use it
            BallDetectorVisionBundle bdvb;
            bdvb.region = &region;
            bdvb.original_region = &region;
            bdvb.region_created = false;
            bdvb.model_region_created = false;
            bdvb.is_crazy_ball = false;

            //Copy RegionAspectType from comboROI
            bdvb.region_aspect_type = region_aspect_type;

            Point centroid = Point(((region_centres[0].x() + region_centres[1].x() + region_centres[2].x()) / 3),
                                   ((region_centres[0].y() + region_centres[1].y() + region_centres[2].y()) / 3));

            // Likely to be closer to the actual centre of the ball so expand by x 2
            int expand_by = blob_max_size * 2.0;

            BBox newBounds = BBox();
            newBounds.a.x() = centroid.x() - expand_by;
            newBounds.a.y() = centroid.y() - expand_by;
            newBounds.b.x() = centroid.x() + expand_by;
            newBounds.b.y() = centroid.y() + expand_by;

            RegionI *blob_region = new RegionI(region, BBox(newBounds.a, newBounds.b), 1, 0, true);

            if (bdvb.region_created == true){
                delete bdvb.region;
            }

            bdvb.region = blob_region;
            bdvb.region_created = true;
            bdvbAdded = true;
            getSizeEst(bdvb, info_in, info_out);

            rescaleRegion(bdvb);
            res.push_back(bdvb);
        }
    }
    if (temp_bdvb.region_created == true) {
        delete temp_bdvb.region;
    }
    return bdvbAdded;
}

bool BallDetector::circleROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res) {
    //std::cout << "CircleROI\n";
    bool bdvbAdded = false;

    // If we don't want to use it
    BallDetectorVisionBundle bdvb;
    bdvb.region = &region;
    bdvb.original_region = &region;
    bdvb.region_created = false;
    bdvb.model_region_created = false;
    bdvb.is_crazy_ball = true;

    getSizeEst(bdvb, info_in, info_out);

    regenerateRegion(bdvb, false);
    calculateAdaptiveValuesForCircleFitting(bdvb);
    preProcessAdaptive(bdvb);

    /*
    // Version1: using adjusted RANSAC
    processCircleFitSizeEst(*bdvb.region, bdvb);

    */
    // Version2: using hough inspired RANSAC

    getCircleCandidatePoints(*bdvb.region, bdvb, false);

    //radius = region.getRows() * 0.5;
    float radius = bdvb.diam_expected_size * 0.5;
    float e = std::max((int) (radius * 0.1), 1); // 5.0; // was 15.0
    uint16_t n = std::max((int) (2.5 * radius), 20);

    std::vector<bool> *con, cons_buf[2];
    cons_buf[0].resize(bdvb.circle_fit_points.size());
    cons_buf[1].resize(bdvb.circle_fit_points.size());
    con = &cons_buf[0];

    int BALL_SIZE_PIXELS = 9;
    findBestCircleFit(bdvb, BALL_SIZE_PIXELS, &con, cons_buf, e, n, 0.7, 2, bdvb.partial_ball_side);

    if (bdvb.region->getRows() < 0 || bdvb.region->getCols() < 0) {
        return bdvbAdded;
    }
#ifdef BALL_DEBUG
    std::cout << "CircleROI\n";
    std::cout << "Rows: " << bdvb.region->getRows() << " Cols: " << bdvb.region->getCols() << "\n";
    std::cout << "Centre: " << bdvb.circle_fit.result_circle.centre.x() << "," << bdvb.circle_fit.result_circle.centre.y() <<
        " Radius: " << bdvb.circle_fit.result_circle.radius << "\n";
#endif // BALL_DEBUG


    /* Sanity check for size */
    /* Checks the ball estimate size is reasonable, and the region is below the field boundary or on the bottom camera.*/

    //std::cout << "Diam size est " << bdvb.diam_size_est << " AND target bbox" << bdvb.region->getBoundingBoxRaw().a.y() << std::endl;

    regenerateRegionFromCircleFit(bdvb, bdvb.circle_fit.result_circle);

    /*
    rescaleRegion(bdvb);

    bdvb.ball.imageCoords.y() =
    ((bdvb.region->getBoundingBoxRaw().b.y() - bdvb.region->getBoundingBoxRaw().a.y())*0.5
        + bdvb.region->getBoundingBoxRaw().a.y())
    + (!bdvb.region->isTopCamera()) * TOP_IMAGE_ROWS;
    */

    res.push_back(bdvb);
    bdvbAdded = true;

    return bdvbAdded;
}

bool BallDetector::inspectBall(BallDetectorVisionBundle &bdvb){

    // Must pass all tests to be ball

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    checkPartialRegion(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    check_partial_region_count++;
    check_partial_region_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    getAverageBrightness(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    get_average_brightness_count++;
    get_average_brightness_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    // Early exit if the ROI has no white pixels in parent fovea binary
    if (bdvb.avg_brightness == 0) {
#ifdef BALL_DEBUG
        std::cout << "No white pixels in original fovea" << "\n";
#endif // BALL_DEBUG
        return false;
    }

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    calculateAdaptiveValuesForCircleFitting(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    calculate_adaptive_values_for_circle_fitting_count++;
    calculate_adaptive_values_for_circle_fitting_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    preProcessAdaptive(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    pre_process_adaptive_count++;
    pre_process_adaptive_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    // Save the region used by the GMM to the bdvb
    RegionI *modelRegion = new RegionI(*bdvb.region, 1, 0, true);

    if (bdvb.model_region_created == true){
        delete bdvb.modelRegion;
    }

    bdvb.modelRegion = modelRegion;
    bdvb.model_region_created = true;

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    getCircleCandidatePoints(*bdvb.region, bdvb, false);
#ifdef BALL_DETECTOR_TIMINGS
    get_circle_candidate_points_count++;
    get_circle_candidate_points_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    findCircleKenji(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    find_circle_kenji_count++;
    find_circle_kenji_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    bdvb.ball.imageCoords.x() = bdvb.region->getBoundingBoxRaw().a.x() +
        bdvb.circle_fit.result_circle.centre.x() * bdvb.region->getDensity();

    bdvb.ball.imageCoords.y() = bdvb.region->getBoundingBoxRaw().a.y() +
        bdvb.circle_fit.result_circle.centre.y() * bdvb.region->getDensity() +
        (!bdvb.region->isTopCamera()) * TOP_IMAGE_ROWS;

    bdvb.ball.radius = (bdvb.circle_fit.result_circle.radius) * bdvb.region->getDensity();

    if (bdvb.ball.radius < 2) {
        return false;
    }

    // Throw out ridiculously small/large balls
    if (bdvb.ball.topCamera) {
        if (bdvb.ball.radius < TOP_CAMERA_MIN_RADIUS ||
            bdvb.ball.radius > TOP_CAMERA_MAX_RADIUS) {
#ifdef BALL_DEBUG
            std::cout << "Radius ridiculously small or large. Radius: " << bdvb.ball.radius << std::endl;
#endif
            return false;
        }
    } else {
        if (bdvb.ball.radius < BOT_CAMERA_MIN_RADIUS ||
            bdvb.ball.radius > BOT_CAMERA_MAX_RADIUS) {
#ifdef BALL_DEBUG
            std::cout << "Radius ridiculously small or large. Radius: " << bdvb.ball.radius << std::endl;
#endif
            return false;
        }
    }

    ballRawRange(bdvb);

    if (analyseBallRawRange(bdvb)) {
#ifdef BALL_DEBUG
        std::cout << "Inter-circle Y variance below threshold\n";
#endif //BALL_DEBUG
        return false;
    }

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    calculateAdaptiveValuesForInternalRegions(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    calculate_adaptive_values_for_internal_regions_count++;
    calculate_adaptive_values_for_internal_regions_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    preProcessAdaptive(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    pre_process_adaptive_count++;
    pre_process_adaptive_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    processInternalRegions(*bdvb.region, bdvb, bdvb.circle_fit.result_circle, bdvb.internal_regions);
#ifdef BALL_DEBUG
    bdvb.internal_regions.print();
#endif //BALL_DEBUG
    if (bdvb.internal_regions.groups.size() < 3) {

#ifdef BALL_DEBUG
        std::cout << "Less than 3 regions" << std::endl;
#endif //BALL_DEBUG
        return false;
    }
#ifdef BALL_DETECTOR_TIMINGS
    process_internal_regions_count++;
    process_internal_regions_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    if (!analyseInternalRegions(bdvb.internal_regions, bdvb)) {

#ifdef BALL_DEBUG
        std::cout << "Analyse Internal Regions fails" << std::endl;
#endif //BALL_DEBUG

        return false;
    }
#ifdef BALL_DETECTOR_TIMINGS
    analyse_internal_regions_count++;
    analyse_internal_regions_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    if (!analyseInternalRegionsTotal(bdvb.internal_regions, bdvb)) {

#ifdef BALL_DEBUG
        std::cout << "Analyse Internal Regions Total fails" << std::endl;
#endif //BALL_DEBUG

        return false;
    }
#ifdef BALL_DETECTOR_TIMINGS
    analyse_internal_regions_total_count++;
    analyse_internal_regions_total_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    findRegionTriangles(bdvb, bdvb.internal_regions, bdvb.region_triangle_features);
#ifdef BALL_DETECTOR_TIMINGS
    find_region_triangles_count++;
    find_region_triangles_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    if (!analyseInternalRegionTriangles(bdvb.region_triangle_features)) {
        return false;
    }
#ifdef BALL_DETECTOR_TIMINGS
    analyse_internal_region_triangles_count++;
    analyse_internal_region_triangles_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    if (estimator.predict(*bdvb.modelRegion) == CLASSIFIER_FALSE) {
#ifdef BALL_DEBUG
        std::cout << "GMM classified FALSE\n";
#endif //BALL_DEBUG
        return false;
    }

#ifdef BALL_DETECTOR_TIMINGS
    gmm_classifier_count++;
    gmm_classifier_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    return true; // Yay, made it!!
}

void BallDetector::rescaleRegion(BallDetectorVisionBundle &bdvb) {
    int region_pixels = bdvb.region->getRows()*(*bdvb.region).getCols();
    int density_change = 0;

    // Find how many powers of 2 density needs to shift to make region the
    // target size.
    while(region_pixels < MIN_REGION_SIZE)
    {
        region_pixels <<= 2;
        ++density_change;
    }

    // Prevent the system from requesting zoom to a density closer than the
    // original image.
    while(1 << density_change > (*bdvb.region).getDensity())
        --density_change;

    bdvb.diam_expected_size = (int) bdvb.diam_expected_size << density_change;

    // Actually create the region at the new density. Colour classification and
    // edge image is needed

    const RegionI *new_region = new RegionI((*bdvb.region), 1 << density_change, DENSITY_DECREASE, true);

    if (bdvb.region_created) {
        delete bdvb.region;
    }

    // For vatnao (might be necessary for other stuff)

    for (unsigned int i = 0; i < bdvb.circle_fit_points.size(); i++) {
        //std::cout << bdvb.circle_fit_points[i].x() << ", " << bdvb.circle_fit_points[i].y() << "\n";
        bdvb.circle_fit_points[i].x() <<= density_change;
        bdvb.circle_fit_points[i].y() <<= density_change;
    }

    bdvb.circle_fit.result_circle.centre.x() = (int) bdvb.circle_fit.result_circle.centre.x() << density_change;
    bdvb.circle_fit.result_circle.centre.y() = (int) bdvb.circle_fit.result_circle.centre.y() << density_change;
    bdvb.circle_fit.result_circle.radius = (int) bdvb.circle_fit.result_circle.radius << density_change;

    bdvb.region = new_region;
    bdvb.region_created = true;
}

inline bool isInsideRadius(int px, int py, RANSACCircle c){
    // TODO: If the circle fit is too loose, we can "shrink" it by subtracting from radius below.
    // This would reduce the risk of connecting dark blobs on the outer edges of the circle.
    if (DISTANCE_SQR((float)px, (float)py, c.centre.x(), c.centre.y()) <= (c.radius * c.radius)){
        return true;
    } else {
        return false;
    }
}

inline bool isOutsideRadius(int px, int py, RANSACCircle c){
    // TODO: If the circle fit is too loose, we can "shrink" it by subtracting from radius below.
    // This would reduce the risk of connecting dark blobs on the outer edges of the circle.
    if (DISTANCE_SQR((float)px, (float)py, c.centre.x(), c.centre.y()) > (c.radius * c.radius)){
        return true;
    } else {
        return false;
    }
}

inline bool isInvalidPos(Point &pos, int num_cols, int num_rows) {
    return pos.x() < 0 || pos.x() >= num_cols || pos.y() < 0 || pos.y() >= num_rows;
}

inline bool isEdgePos(Point &pos, int num_cols, int num_rows) {
    return pos.x() == 0 || pos.x() == num_cols - 1 || pos.y() == 0 || pos.y() == num_rows - 1;
}


/* Check if the region is not white, based on OTSU and the ransac circle */
// Anything outside of the radius will be considered as white, for CCA.

inline bool isNotWhiteAndInsideAdaptive(const Colour colour, int px, int py, RANSACCircle c) {
    return colour != cWHITE && isInsideRadius(px, py, c);
}

void BallDetector::preProcessAdaptive(BallDetectorVisionBundle &bdvb) {

    // Adaptive Thresholding
#ifdef BALL_DEBUG
    std::cout << "Set AT Values -\n";
    std::cout << "Window: " << bdvb.window_size << " Percentage: " << bdvb.percentage << "\n";
#endif //BALL_DEBUG

    RegionI *region = new RegionI(*bdvb.region, 1, 0, true, bdvb.window_size, bdvb.percentage);

    if (bdvb.region_created == true){
        delete bdvb.region;
    }
    bdvb.region = region;
    bdvb.region_created = true;
}

void BallDetector::processInternalRegions(const RegionI& base_region, BallDetectorVisionBundle &bdvb,
        RANSACCircle &result_circle, InternalRegionFeatures &internal_regions)
{
    connectedComponentAnalysisNotWhiteAndInside(base_region, bdvb, result_circle);

    InternalRegionFeatures internal_region_features;
    internal_region_features.num_internal_regions = 0;
    internal_region_features.num_regions = 0;
    internal_region_features.max_internal_region_prop = 0;
    internal_region_features.groups.reserve(10);

    int centre_x = result_circle.centre.x();
    int centre_y = result_circle.centre.y();
    double area_circle = result_circle.radius * result_circle.radius * M_PI;

    int min_internal_group_size, max_internal_group_size;

    if (bdvb.is_crazy_ball){
        min_internal_group_size = result_circle.radius * result_circle.radius * 0.005;
    }
    else {
        min_internal_group_size = area_circle * 0.01;
    }
    max_internal_group_size = area_circle * 0.2;

    // Count the number of groups that do not touch the edge.
    for(int group=0; group<group_links_.size(); ++group)
    {
        if(group_counts_[group] > min_internal_group_size &&
                group_counts_[group] < max_internal_group_size)
            {
                InternalRegion r;
            r.num_pixels = group_counts_[group];
            r.min_x = group_low_xs_[group];
            r.max_x = group_high_xs_[group];
            r.min_y = group_low_ys_[group];
            r.max_y = group_high_ys_[group];

            if (
                (DISTANCE_SQR(centre_x, centre_y, group_low_xs_[group], group_low_ys_[group])
                    < result_circle.radius * result_circle.radius) &&
                (DISTANCE_SQR(centre_x, centre_y, group_low_xs_[group], group_high_ys_[group])
                    < result_circle.radius * result_circle.radius) &&
                (DISTANCE_SQR(centre_x, centre_y, group_high_xs_[group], group_low_ys_[group])
                    < result_circle.radius * result_circle.radius) &&
                (DISTANCE_SQR(centre_x, centre_y, group_high_xs_[group], group_high_ys_[group])
                    < result_circle.radius * result_circle.radius)) {
                r.completely_internal = true;
                internal_region_features.num_internal_regions++;

                internal_region_features.max_internal_region_prop =
                    std::max(internal_region_features.max_internal_region_prop,
                        1.0 * r.num_pixels / area_circle);
            }
            else {
                r.completely_internal = false;
            }

            // If both width AND height of internal region is smaller than min, throw out
            if ((r.max_x - r.min_x) < result_circle.radius * INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MIN &&
                (r.max_y - r.min_y) < result_circle.radius * INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MIN){
                continue;
            }

            // If either width OR height of internal region is larger than max, throw out
            if ((r.max_x - r.min_x) > result_circle.radius * INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MAX ||
                (r.max_y - r.min_y) > result_circle.radius * INTERNAL_REGION_SIDE_TO_RADIUS_RATIO_MAX){
                continue;
            }

            internal_region_features.num_regions++;
            internal_region_features.groups.push_back(r);
        }
    }

    internal_regions = internal_region_features;

#ifdef BALL_DEBUG
    std::cout << "NumRegions: " << internal_region_features.num_regions << "\n";
    std::cout << "InternalRegions: " << internal_region_features.num_internal_regions << "\n";
#endif // BALL_DEBUG
}

void BallDetector::processInternalRegionsROI(const RegionI &base_region, BallDetectorVisionBundle &bdvb,
     InternalRegionFeatures &internal_regions) {

    connectedComponentAnalysisNotWhite(base_region, bdvb);

    InternalRegionFeatures internal_region_features;
    internal_region_features.num_internal_regions = 0;
    internal_region_features.num_regions = 0;
    internal_region_features.max_internal_region_prop = 0;
    internal_region_features.groups.reserve(10);

    int min_internal_group_size, max_internal_group_size;

    // Max and min pix in a blob
    min_internal_group_size = 6;
    max_internal_group_size = 200;

    // For finding the most dence region
    int density_error = 100;
    int cur_den_err;

    // Count the number of groups that might be blobby.
    for(int group=0; group<group_links_.size(); ++group)
    {
        if(group_counts_[group] > min_internal_group_size &&
                group_counts_[group] < max_internal_group_size)
        {
            InternalRegion r;
            r.num_pixels = group_counts_[group];
            r.min_x = group_low_xs_[group];
            r.max_x = group_high_xs_[group];
            r.min_y = group_low_ys_[group];
            r.max_y = group_high_ys_[group];

            float x_size = (group_high_xs_[group] - group_low_xs_[group]);
            float y_size = (group_high_ys_[group] - group_low_ys_[group]);

            //If the aspect ratio of the blob not blobby enough, throw out
            float aspect = x_size / y_size;
            //std::cout << "Aspect: " << aspect << "\n";
            if (aspect <= 0.5 || aspect >= 2) {
#ifdef BLOBROI_DEBUG
                std::cout << "Aspect: " << aspect << "\n";
                std::cout << "Aspect bad\n";
#endif //BLOBROI_DEBUG
                continue;
            }

            // If the density of the pixels is low, throw out
            int area = (x_size * y_size);
            float density = (float) r.num_pixels / area;
            if (density <= 0.75) {
#ifdef BLOBROI_DEBUG
                std::cout << "Area: " << area << " Num pix:" << r.num_pixels << " density: " << density << "\n";
                std::cout << "Desity bad\n";
#endif //BLOBROI_DEBUG
                continue;
            }

            r.completely_internal = true;
            internal_region_features.num_internal_regions++;
            internal_region_features.num_regions++;

            // Find the most dence region and same its region number for use in BlobROI
            cur_den_err = 1 - density;
            if (cur_den_err < density_error) {
                density_error = cur_den_err;
                internal_region_features.max_region_density_number = internal_region_features.num_regions;
            }
#ifdef BLOBROI_DEBUG
            std::cout << "---Region Accepted---\n";
            std::cout << "Aspect: " << aspect << " Density: " << density << "\n";
#endif //BLOBROI_DEBUG
            internal_region_features.groups.push_back(r);
        }
    }
    internal_regions = internal_region_features;

#ifdef BALL_DEBUG
        std::cout << "Num Blob Regions: " << internal_region_features.num_regions << "\n";
#endif // BALL_DEBUG
}

bool BallDetector::analyseInternalRegions(InternalRegionFeatures &internal_region_features, BallDetectorVisionBundle &bdvb) {
    // std::cout << "internalregions: " << internal_region_features.num_internal_regions <<
    //  " maxinternalregionprop: " << internal_region_features.max_internal_region_prop << "\n";
    if (bdvb.is_partial_region){
        return internal_region_features.num_internal_regions >= 1
            && internal_region_features.max_internal_region_prop >= 0.05;
    } else {
        return internal_region_features.num_internal_regions >= 1
            && internal_region_features.max_internal_region_prop >= 0.05;
    }
}

bool BallDetector::analyseInternalRegionsTotal(InternalRegionFeatures &internal_region_features, BallDetectorVisionBundle &bdvb) {
    for (int i=0; i<internal_region_features.num_regions; ++i){
        // if we have a nice blob in the middle, return true
        if (internal_region_features.groups[i].completely_internal){
            int width = internal_region_features.groups[i].max_x - internal_region_features.groups[i].min_x;
            int height = internal_region_features.groups[i].max_y - internal_region_features.groups[i].min_y;
            int centre_x = (internal_region_features.groups[i].max_x + internal_region_features.groups[i].min_x)/2;
            int centre_y = (internal_region_features.groups[i].max_y + internal_region_features.groups[i].min_y)/2;

            int max_w = bdvb.circle_fit.result_circle.radius * 0.9;
            int min_w = bdvb.circle_fit.result_circle.radius * 0.4;
            int max_centre_error = bdvb.circle_fit.result_circle.radius * 0.2;

            // std::cout << "width: " << width << "\n"
            //           << "height: " << height << "\n"
            //           << "centre_x: " << centre_x << "\n"
            //           << "centre_y: " << centre_y << "\n"
            //           << "max_w: " << max_w << "\n"
            //           << "min_w: " << min_w << "\n"
            //           << "result_circle.centre.x(): " << bdvb.circle_fit.result_circle.centre.x() << "\n"
            //           << "result_circle.centre.y(): " << bdvb.circle_fit.result_circle.centre.y() << "\n"
            //           << "max_centre_error" << max_centre_error << std::endl;

            if (width < max_w && width > min_w && height < max_w && height > min_w
                    && abs(centre_x - bdvb.circle_fit.result_circle.centre.x()) < max_centre_error
                    && abs(centre_y - bdvb.circle_fit.result_circle.centre.y()) < max_centre_error){
                // std::cout << "Region in center!" << std::endl;
                return true;
            }
        }
    }

    // Check the number of regions
    if (bdvb.is_partial_region){
        return (internal_region_features.num_regions >= 3 && internal_region_features.num_regions <= 10);
    } else {
        return (internal_region_features.num_regions >= 3 && internal_region_features.num_regions <= 10);
    }
}

bool BallDetector::analyseInternalRegionTriangles(RegionTriangleFeatures region_triangle_features) {
    return region_triangle_features.region_triangles.size() >= 1;
}

bool BallDetector::analyseInternalRegionCircles(std::vector <CircleFitFeatures> &internal_region_circles) {
    return internal_region_circles.size() >= 1;
}

bool BallDetector::analyseBallRawRange(BallDetectorVisionBundle &bdvb) {
    return (abs(bdvb.max_y_value - bdvb.min_y_value) < BALL_Y_RANGE_THRESHOLD);
}

void BallDetector::findBestCircleFit(BallDetectorVisionBundle &bdvb, float max_radius, std::vector<bool> **cons, std::vector <bool> cons_buf[2], float e, unsigned int n,
        float min_radius_prop, float step_size, PartialBallSide partial_ball_side) {
    // Systematically start from radius equal to half the columns and slowly reducing the radius
    // Try all centres such that the circle wholly fits inside the region
    // Find the first circle that meets criteria (similar to RANSAC)

    // Code largely (at least variance code) taken from RANSAC. Not sure what it means ~ VictorW
    RANSACCircle c = RANSACCircle(PointF(0, 0), 0);

    float curr_radius = max_radius;

    unsigned int j;
    float pos_var[4], neg_var[4];
    const int e2 = e * e;

#ifdef BALL_DEBUG
    float max_prop_x = 0;
    float max_prop_y_l = 0;
    float max_prop_y_r = 0;
#endif // BALL_DEBUG

#ifdef BALL_DETECTOR_TIMINGS
    int curr_iterations = 0;
#endif // BALL_DETECTOR_TIMINGS

    /* error of best circle found so far */
    float minerr = std::numeric_limits<float>::max();
    c.var = std::numeric_limits<float>::max();

    std::vector<bool> *best_concensus, *this_concensus;
    best_concensus = &cons_buf[0];
    //std::cout << "Cols: " << bdvb.region->getCols() << " Rows: " << bdvb.region->getRows() << "\n";

    while (curr_radius > min_radius_prop * max_radius) {
        //std::cout << "Rad: " << curr_radius << "\n";
        c.radius = curr_radius;

        // Start from the top left most centre
        float centre_x = (partial_ball_side == BALL_SIDE_LEFT) ? 0 : curr_radius;
        float right_bound = bdvb.region->getCols();

        right_bound -= (partial_ball_side == BALL_SIDE_RIGHT) ? 0 : curr_radius;

        while (centre_x < right_bound + 1) {
            //std::cout << "Centre_x: " << centre_x << "\n";
            float centre_y = (partial_ball_side == BALL_SIDE_TOP) ? 0 : curr_radius;
            float bottom_bound = bdvb.region->getRows();

            bottom_bound -= (partial_ball_side == BALL_SIDE_BOTTOM) ? 0 : curr_radius;

            while (centre_y < bottom_bound + 1) {
#ifdef BALL_DETECTOR_TIMINGS
                circle_fit_iterations++;
                curr_iterations++;
#endif // BALL_DETECTOR_TIMINGS
                //std::cout << "Centre: " << centre_x << "," << centre_y << " rad: " << curr_radius << "\n";
                c.centre = PointF(centre_x, centre_y);

                if (best_concensus == &cons_buf[0]) {
                    this_concensus = &cons_buf[1];
                } else {
                   this_concensus = &cons_buf[0];
                }

                for (j = 0; j < 4; ++ j) {
                   pos_var[j] = neg_var[j] = 0;
                }

                unsigned int n_concensus_points = 0;
                std::set <int> x_values;
                std::set <int> y_values_l;
                std::set <int> y_values_r;

                for (j = 0; j != bdvb.circle_fit_points.size(); ++ j) {
                   const PointF &p = bdvb.circle_fit_points[j].cast<float>();
                   const PointF &d = c.centre - p;

                   // Technically incorrect, but faster than a sqrt.
                    float dist = d.norm() - c.radius;
                    float dist2 = dist*dist;

                    if (dist2 < e2) {
                        int quadrant = 0;
                        if (d.x() > 0) {
                            if (d.y() > 0) {
                                quadrant = 0;
                            } else {
                                quadrant = 3;
                            }
                        } else {
                            if (d.y() > 0) {
                                quadrant = 1;
                            } else {
                                quadrant = 2;
                            }
                        }

                        if (dist > 0) {
                            pos_var[quadrant] += dist2;
                        } else {
                            neg_var[quadrant] += dist2;
                        }

                        ++ n_concensus_points;
                        (*this_concensus)[j] = true;
                    } else {
                        (*this_concensus)[j] = false;
                    }
                }

                const float k = 0.2;

                c.var = 0;
                for (j = 0; j < 4; ++ j) {
                    float diff_var = pos_var[j] - neg_var[j];
                    c.var += pos_var[j] + neg_var[j] + diff_var * diff_var;
                }

                c.var = (k * c.var) - n_concensus_points;

//std::cout << "Centrex: " << c.centre.x() << " Centrey: " << c.centre.y() << " radius: " << c.radius << " var: " << c.var << "\n";

                // Changed condition to look at x_coverage instead

                if (c.var < minerr && n_concensus_points >= n) {
#ifdef BALL_DEBUG
                    std::cout << "____________MAXPROP FOUND: " << ((float) x_values.size()) / (2 * curr_radius) << ", " <<
                        ((float) y_values_l.size()) / (curr_radius) << ", " <<
                        ((float) y_values_r.size()) / (curr_radius) << "\n";
                    std::cout << "Circle Found: " << c.centre.x() << "," << c.centre.y() << " radius: " << c.radius << "\n";
#endif // BALL_DEBUG
                    minerr = c.var;
                    c.var  = c.var / (bdvb.circle_fit_points.size() * e);
                    bdvb.circle_fit.result_circle = c;
                    best_concensus = this_concensus;

                        // Select the first one that meets the requirements

                    bdvb.circle_fit.result_circle = c;
                }

                centre_y += step_size;
            }

            centre_x += step_size;
        }

        curr_radius -= 1;
    }

#ifdef BALL_DETECTOR_TIMINGS
    circle_fit_max_iterations = std::max(circle_fit_max_iterations, curr_iterations);
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DEBUG
    std::cout << "____________MAXPROP: " << max_prop_x << ", " <<
        max_prop_y_l << ", " << max_prop_y_r << "\n";
    std::cout << "No circle found\n";
#endif // BALL_DEBUG
}


void BallDetector::findLargestCircleFit(BallDetectorVisionBundle &bdvb, float max_radius, std::vector<bool> **cons, std::vector <bool> cons_buf[2], float e, unsigned int n,
        float min_radius_prop, float step_size, PartialBallSide partial_ball_side) {
    // Systematically start from radius equal to half the columns and slowly reducing the radius
    // Try all centres such that the circle wholly fits inside the region
    // Find the first circle that meets criteria (similar to RANSAC)

    // Code largely (at least variance code) taken from RANSAC. Not sure what it means ~ VictorW
    RANSACCircle c = RANSACCircle(PointF(0, 0), 0);

    float curr_radius = max_radius;

    unsigned int j;
    float pos_var[4], neg_var[4];
    const int e2 = e * e;

    float x_coverage_prop = 0.75;
    float y_coverage_prop = 0.55;

#ifdef BALL_DEBUG
    float max_prop_x = 0;
    float max_prop_y_l = 0;
    float max_prop_y_r = 0;
#endif // BALL_DEBUG

#ifdef BALL_DETECTOR_TIMINGS
    int curr_iterations = 0;
#endif // BALL_DETECTOR_TIMINGS

    c.var = std::numeric_limits<float>::max();

    std::vector<bool> *best_concensus, *this_concensus;
    best_concensus = &cons_buf[0];

    while (curr_radius > min_radius_prop * max_radius) {
        c.radius = curr_radius;

        // Start from the top left most centre
        float centre_x = (partial_ball_side == BALL_SIDE_LEFT) ? 0 : curr_radius;
        float right_bound = bdvb.region->getCols();

        right_bound -= (partial_ball_side == BALL_SIDE_RIGHT) ? 0 : curr_radius;

        while (centre_x < right_bound + 1) {
            float centre_y = (partial_ball_side == BALL_SIDE_TOP) ? 0 : curr_radius;
            float bottom_bound = bdvb.region->getRows();

            bottom_bound -= (partial_ball_side == BALL_SIDE_BOTTOM) ? 0 : curr_radius;

            while (centre_y < bottom_bound + 1) {
#ifdef BALL_DETECTOR_TIMINGS
                circle_fit_iterations++;
                curr_iterations++;
#endif // BALL_DETECTOR_TIMINGS
                c.centre = PointF(centre_x, centre_y);

                if (best_concensus == &cons_buf[0]) {
                    this_concensus = &cons_buf[1];
                } else {
                   this_concensus = &cons_buf[0];
                }

                for (j = 0; j < 4; ++ j) {
                   pos_var[j] = neg_var[j] = 0;
                }

                unsigned int n_concensus_points = 0;
                std::set <int> x_values;
                std::set <int> y_values_l;
                std::set <int> y_values_r;

                for (j = 0; j != bdvb.circle_fit_points.size(); ++ j) {
                   const PointF &p = bdvb.circle_fit_points[j].cast<float>();
                   const PointF &d = c.centre - p;

                   /* TODO(carl) look into integer version of this */
                    float dist = d.norm() - c.radius;
                    float dist2 = dist * dist;

                    if (dist2 < e2) {
                        int quadrant = 0;
                        if (d.x() > 0) {
                            if (d.y() > 0) {
                                quadrant = 0;
                            } else {
                                quadrant = 3;
                            }
                        } else {
                            if (d.y() > 0) {
                                quadrant = 1;
                            } else {
                                quadrant = 2;
                            }
                        }

                        if (dist > 0) {
                            pos_var[quadrant] += dist2;
                        } else {
                            neg_var[quadrant] += dist2;
                        }

                        if (bdvb.circle_fit_points[j].y() <= centre_y
                            && bdvb.circle_fit_points[j].y() >= centre_y - curr_radius
                            && bdvb.circle_fit_points[j].x() >= centre_x - curr_radius
                            && bdvb.circle_fit_points[j].x() <= centre_x + curr_radius) {
                            x_values.insert(bdvb.circle_fit_points[j].x());

                            if (bdvb.circle_fit_points[j].x() < centre_x) {
                                y_values_l.insert(bdvb.circle_fit_points[j].y());
                            }
                            else {
                                y_values_r.insert(bdvb.circle_fit_points[j].y());
                            }
                        }

                        ++ n_concensus_points;
                        (*this_concensus)[j] = true;
                    } else {
                        (*this_concensus)[j] = false;
                    }
                }

                // Changed condition to look at x_coverage instead

#ifdef BALL_DEBUG
                if (((float) x_values.size()) / (2 * curr_radius) > max_prop_x &&
                        ((float) y_values_l.size() / curr_radius) > max_prop_y_l &&
                        ((float) y_values_r.size() / curr_radius) > max_prop_y_r) {
                    max_prop_x = std::max(max_prop_x, ((float) x_values.size()) / (2 * curr_radius));
                    max_prop_y_l = std::max(max_prop_y_l, ((float) y_values_l.size()) / (curr_radius));
                    max_prop_y_r = std::max(max_prop_y_r, ((float) y_values_r.size()) / (curr_radius));
                }
#endif // BALL_DEBUG

                if (x_values.size() >= x_coverage_prop * (2 * curr_radius)
                    && y_values_l.size() >= y_coverage_prop * curr_radius
                    && y_values_r.size() >= y_coverage_prop * curr_radius) {
#ifdef BALL_DEBUG
                    std::cout << "____________MAXPROP FOUND: " << ((float) x_values.size()) / (2 * curr_radius) << ", " <<
                        ((float) y_values_l.size()) / (curr_radius) << ", " <<
                        ((float) y_values_r.size()) / (curr_radius) << "\n";
                    std::cout << "Circle Found: " << c.centre.x() << "," << c.centre.y() << " radius: " << c.radius << "\n";
#endif // BALL_DEBUG
                    bdvb.circle_fit.result_circle = c;
                    best_concensus = this_concensus;

                        // Select the first one that meets the requirements

                    bdvb.circle_fit.result_circle = c;
#ifdef BALL_DETECTOR_TIMINGS
                    circle_fit_max_iterations = std::max(circle_fit_max_iterations, curr_iterations);
#endif // BALL_DETECTOR_TIMINGS
                    return;
                }

                centre_y += step_size;
            }

            centre_x += step_size;
        }

        curr_radius -= step_size;
    }

#ifdef BALL_DETECTOR_TIMINGS
    circle_fit_max_iterations = std::max(circle_fit_max_iterations, curr_iterations);
#endif // BALL_DETECTOR_TIMINGS

#ifdef BALL_DEBUG
    std::cout << "____________MAXPROP: " << max_prop_x << ", " <<
        max_prop_y_l << ", " << max_prop_y_r << "\n";
    std::cout << "No circle found\n";
#endif // BALL_DEBUG
}

void BallDetector::processCircleFitSizeEst(const RegionI& region, BallDetectorVisionBundle &bdvb){
    /* Generate the candidate points */
    getCircleCandidatePoints(region, bdvb, false);

    // Ransac
    float radius = (bdvb.diam_expected_size_pixels) * 0.45;
    //float radius = region.getRows() * 0.45;

    float radius_e = radius * 0.1;
    uint16_t k = 10;
    float e = radius * 0.05;//3.0; // was 15.0
    //uint16_t n = std::max((int) (0.7 * bdvb.circle_fit_points.size()), 10);
    uint16_t n = std::max((int) (1.5 * 2 * M_PI * radius), 20);
    unsigned int seed = 42;

    //if (region.isTopCamera()) n += 5;

    std::vector<bool> *con, cons_buf[2];
    cons_buf[0].resize(bdvb.circle_fit_points.size());
    cons_buf[1].resize(bdvb.circle_fit_points.size());
    con = &cons_buf[0];
    bdvb.circle_fit.result_circle = RANSACCircle(PointF(0,0), 0.0);

    //bdvb.circle_fit.circle_found = RANSAC::findCircleOfRadius3P(bdvb.circle_fit_points, radius, radius_e, &con, bdvb.circle_fit.result_circle, k, e, n, cons_buf, &seed);
    float centre_bound = radius * 0.4;
    bdvb.circle_fit.circle_found = RANSAC::findCircleOfRadius3PInsideBounds(bdvb.circle_fit_points, radius, radius_e, &con,
        bdvb.circle_fit.result_circle, k, e, n, cons_buf, &seed,
        centre_bound, bdvb.region->getCols() - centre_bound, centre_bound, bdvb.region->getRows() - centre_bound);

    std::cout << "Target radius: " << radius << "\n";
    std::cout << "Rows: " << bdvb.region->getRows() << " Cols: " << bdvb.region->getCols() << "\n";
    std::cout << "Circle x: " << bdvb.circle_fit.result_circle.centre.x() << " , " << bdvb.circle_fit.result_circle.centre.y() << " radius: " <<
        bdvb.circle_fit.result_circle.radius << "\n";

    if (bdvb.circle_fit.circle_found){
        //std::cout << "CIRCLE FOUND" << std::endl;
    }
}

void BallDetector::getCircleCandidatePoints(const RegionI& region, BallDetectorVisionBundle &bdvb, bool semiCircle) {

    RegionI::iterator_fovea cur_point = region.begin_fovea();
    int rows = region.getRows();
    int cols = region.getCols();

    bool top_cam = bdvb.region->isTopCamera();

    // reserve max number of circle_fit_points
    bdvb.circle_fit_points.reserve(2*rows + 2*cols);

    std::vector<int> tops(cols, -1);
    std::vector<int> bots(cols, -1);

    for (int y = 0; y < rows; ++y){
        int left = -1;
        int right = -1;

        for (int x = 0; x < cols; ++x){

            if (cur_point.colour() == cWHITE){

                // left code
                if ((x < cols - 1) && (left == -1)) {
                    if (cur_point.colourRight() == cWHITE){
                        left = x;
                    }
                }
                // right code
                if (x > 0) {
                    if (cur_point.colourLeft() == cWHITE) {
                        right = x;
                    }
                }
                // top code
                if ((y < rows - 1) && (tops[x] == -1)) {
                    if (cur_point.colourBelow() == cWHITE){
                        tops[x] = y;
                    }
                }
                // bottom code (only run in bottom camera)
                if ((y > 0) && !top_cam) {
                    if (cur_point.colourAbove() == cWHITE) {
                        bots[x] = y;
                    }
                }
            }
            cur_point++;
        }

        // add left point
        if (left > 0 && left < cols - 1){
            bdvb.circle_fit_points.push_back(Point(left,y));
        }

        // add right point
        if (right > 0 && right < rows - 1){
            bdvb.circle_fit_points.push_back(Point(right,y));
        }
    }

    // add top points
    for (int x = 0; x < cols; ++x){
        int val = tops[x];

        if (val > 0 && val < rows - 1){
            bdvb.circle_fit_points.push_back(Point(x, val));
        }
    }

    // add bottom points (only run in bottom camera)
    if (!top_cam){
        for (int x = 0; x < cols; ++x){
            int val = bots[x];

            if (val > 0 && val < rows - 1){
                bdvb.circle_fit_points.push_back(Point(x, val));
            }
        }
    }

#ifdef BALL_DEBUG
    std::cout << "GetCircleCandidatePoints: " << bdvb.circle_fit_points.size() << "\n";
#endif // BALL_DEBUG
}

void BallDetector::findCircleKenji(BallDetectorVisionBundle &bdvb){

    int rows = bdvb.region->getRows();
    int cols = bdvb.region->getCols();

    int minRadius = 1;
    int maxRadius = std::min(rows, cols);
    std::vector<int> radii(maxRadius+1,0);

    // 1. Find circle centre points
    int numPoints = bdvb.circle_fit_points.size();
    if (numPoints < 10){
        return;
    }

    int numCombinations = 0.5 * (numPoints-2) * (numPoints-1); // (xC3, x is real)
    int numTrials = std::min(numCombinations / 5, CIRCLE_FIT_MAX_CENTER_POINTS);

    Point p1;
    Point p2;
    Point p3;

    bdvb.circle_center_points.reserve(CIRCLE_FIT_MAX_CENTER_POINTS);

    for (int t = 0; t < numTrials; t++){

        // Use pregenerated random numbers
        p1 = bdvb.circle_fit_points[randomNums[3*t] % numPoints];
        p2 = bdvb.circle_fit_points[randomNums[3*t+1] % numPoints];
        p3 = bdvb.circle_fit_points[randomNums[3*t+2] % numPoints];

        // Alternative: generate random number
        // p1 = bdvb.circle_fit_points[rand() % numPoints];
        // p2 = bdvb.circle_fit_points[rand() % numPoints];
        // p3 = bdvb.circle_fit_points[rand() % numPoints];

        // make sure no two points are the same
        if (p1 == p2 || p2 == p3 || p1 == p3){
            continue;
        }

        /**
         * Explanation for following calculation:
         * 1. Obtain equation of equidistance from p1 and p2.
         *    ie. (x-x1)^2 + (y-y1)^2 = (x-x2)^2 + (y-y2)^2
         * 2. Rearrange into ax + by = c, and obtain equation for a,b,c.
         * 3. Repeat for p2 and p3
         * 4. Plug a,b into matrix A
         * 5. Plug c into matrix B
         * 6. Solve using AX = B
         */

        Eigen::Matrix<float, 2, 2> A;
        Eigen::Matrix<float, 2, 1> B;
        Eigen::Matrix<float, 2, 1> X;

        A(0, 0) = 2*p2.x() - 2*p1.x();
        A(0, 1) = 2*p2.y() - 2*p1.y();
        A(1, 0) = 2*p3.x() - 2*p2.x();
        A(1, 1) = 2*p3.y() - 2*p2.y();

        B(0, 0) = p2.x()*p2.x() + p2.y()*p2.y() - p1.x()*p1.x() - p1.y()*p1.y();
        B(1, 0) = p3.x()*p3.x() + p3.y()*p3.y() - p2.x()*p2.x() - p2.y()*p2.y();

        // std::cout << "A: \n" << A << std::endl;
        // std::cout << "B: \n" << B << std::endl;

        X = A.inverse() * B;

        // std::cout << "X: \n" << X << std::endl;

        // std::cout << "P1: " << p1.x() << ", " << p1.y() << std::endl;
        // std::cout << "P2: " << p2.x() << ", " << p2.y() << std::endl;
        // std::cout << "P3: " << p3.x() << ", " << p3.y() << std::endl;

        // throw out points that are outside image or on outer edges
        if (X(0,0) < CIRCLE_FIT_GAUSSIAN_SIZE || X(0,0) >= cols-1-CIRCLE_FIT_GAUSSIAN_SIZE){
            continue;
        }

        if (X(1,0) < CIRCLE_FIT_GAUSSIAN_SIZE || X(1,0) >= rows-1-CIRCLE_FIT_GAUSSIAN_SIZE){
            continue;
        }

        int radius = sqrt(pow((X(0,0)-p1.x()),2) + pow((X(1,0)-p1.y()),2));

        if (radius < minRadius || radius >= maxRadius){
            continue;
        }

        // vote on radius
        radii[radius-1] += 1;
        radii[radius] += 2;
        radii[radius+1] += 1;

        bdvb.circle_center_points.push_back(Point(X(0,0),X(1,0)));
    }

    // 2. Make 2d heat map
    Eigen::MatrixXi map = Eigen::MatrixXi::Zero(cols,rows);
    Eigen::Matrix<int, CIRCLE_FIT_GAUSSIAN_SIZE, CIRCLE_FIT_GAUSSIAN_SIZE> gaussianVote;

    gaussianVote << 1,  4,  7,  4,  1,
                    4, 16, 26, 16,  4,
                    7, 26, 41, 26,  7,
                    4, 16, 26, 16,  4,
                    1,  4,  7,  4,  1;

    for (std::vector<Point>::iterator it = bdvb.circle_center_points.begin(); it != bdvb.circle_center_points.end(); ++it)
    {
        int x = (*it).x();
        int y = (*it).y();

        // std::cout << "x: " << x << ", y: " << y << std::endl;
        map.block<CIRCLE_FIT_GAUSSIAN_SIZE, CIRCLE_FIT_GAUSSIAN_SIZE>(x-(CIRCLE_FIT_GAUSSIAN_SIZE)/2, y-(CIRCLE_FIT_GAUSSIAN_SIZE)/2) += gaussianVote;
    }

    // std::cout << "map \n" << map << std::endl;

    // 3. Search in map, highest score
    int max=0;
    int maxCol=0;
    int maxRow=0;
    for (int x = 0; x < cols; ++x){
        for (int y = 0; y < rows; ++y){
            int l = map(x, y);
            if (l > max){
                maxCol = x;
                maxRow = y;
                max = l;
            }
        }
    }


    if (max < CIRCLE_FIT_CENTRE_VOTE_THRESHOLD){
        return;
    }

    // 4. find highest score for radius
    std::vector<int>::iterator highest_radius = std::max_element(radii.begin(), radii.end());
    if (std::distance(radii.begin(), highest_radius) < CIRCLE_FIT_RADIUS_VOTE_THRESHOLD){
        return;
    }

    // 5. Compose circle
    bdvb.circle_fit.result_circle = RANSACCircle(PointF(maxCol, maxRow), std::distance(radii.begin(), highest_radius));
    bdvb.circle_fit.circle_found = true;

#ifdef BALL_DEBUG
    std::cout << "- Kenji circle fitted -\n" <<  bdvb.circle_fit.getSummary();
#endif //BALL_DEBUG
}


bool BallDetector::analyseCircleFit(CircleFitFeatures &cf){
    if (cf.circle_found) {
        return true;
    } else {
        return false;
    }
}

void BallDetector::getSizeEst(BallDetectorVisionBundle &bdvb, const VisionInfoIn& info_in,
        VisionInfoOut& info_out) {
    if (offNao) {
        // Offnao does not have cameraToRR
        return;
    }

    bdvb.ball.topCamera = bdvb.region->isTopCamera();

    // Ball position and size.
    bdvb.ball.imageCoords.x() =
        ((bdvb.region->getBoundingBoxRaw().b.x() - bdvb.region->getBoundingBoxRaw().a.x()) * 0.5
        + bdvb.region->getBoundingBoxRaw().a.x());

    // Ball radius is used for imageToRobotXY (assumed to be "height" of object from ground)
    bdvb.ball.radius = (bdvb.region->getCols() * bdvb.region->getDensity())/2.0f;

    // Not sure what this is about, but included as earlier code does
    // this.
    bdvb.ball.lastSeen = 0;
    bdvb.ball.lifetime = 0;

    // Determine robot relative ball pose.
    // Note that this implicitly determines whether the image is top or
    // bottom based on y value. Apparently bottom image coordinates
    // should start at the maximum top image y value.

    // The second argument to imageToRobotXY is the distance of the coordinate from the ground
    // For our bounding box, we can assume this is half the height of the bounding box since
    // the objects we are detecting should be on the ground and should be entirely captured by the
    // region (at least for the ball)

    // Point for RR is the top of the bounding box

    Point pointForRR = Point(bdvb.ball.imageCoords.x(),
        bdvb.region->getBoundingBoxRaw().a.y() + (!bdvb.region->isTopCamera()) * TOP_IMAGE_ROWS);

    int ball_height = 100;

    Point b = info_out.cameraToRR->pose.imageToRobotXY(pointForRR, ball_height);

    float diff = 190*tan(info_in.latestAngleX);

    b.y() -= diff;

    RRCoord rr;
    rr.distance() = hypotf(b.y(), b.x());
    rr.heading() = atan2f(b.y(), b.x());
    bdvb.ball.rr = rr;

    float robot_height = 500;
    float error = 30 * bdvb.ball.rr.distance() / robot_height;
    bdvb.ball.rr.distance() -= error;

    bdvb.ball.neckRelative =
        info_out.cameraToRR->pose.robotRelativeToNeckCoord(bdvb.ball.rr,
        bdvb.ball.radius);

    float neck_distance = sqrt(pow(sqrt(pow(bdvb.ball.neckRelative.x, 2) +
        pow(bdvb.ball.neckRelative.y, 2)), 2) + pow(bdvb.ball.neckRelative.z, 2));

    // Calculate size estimate

    int frame_width =
        bdvb.region->isTopCamera()? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

    float half_frame_width = frame_width / 2.0f;
    float ball_pixel_dist_to_centre = abs(bdvb.ball.imageCoords.x() - half_frame_width);

    bdvb.diam_size_est = 2 * neck_distance * bdvb.ball.radius / sqrt(3 *
        half_frame_width * half_frame_width + ball_pixel_dist_to_centre
        * ball_pixel_dist_to_centre);

    bdvb.diam_expected_size =
        (100 * sqrt(3 * half_frame_width * half_frame_width + ball_pixel_dist_to_centre * ball_pixel_dist_to_centre)
                    / neck_distance) / bdvb.region->getDensity();

    Point pointForSizeEst = Point(0,
        pointForRR.y() + (bdvb.region->getBoundingBoxRaw().b.y() - bdvb.region->getBoundingBoxRaw().a.y()) / 2);
    bdvb.diam_expected_size_pixels = getDiamInImage(info_out, pointForSizeEst);
}

// TODO: Check if this is even valid now camera pose is calibrated.
// Given a fovea and a y-coord in image coords, get the approximate radius we expect.
// The function below comes from measuring the diameter of the ball in pixels
// at different heights in each image, and then linearly interpolating over
// these data points. The resulting affine functions have little error, as
// the measurements were very linear, except when going from half a field away to
// the full field length away - but at that distance, the ball is unlikely to be
// seen anyway. The constants are from the affine approximations, derived in
// OS X's program Grapher
float BallDetector::getDiamInImage(VisionInfoOut& info_out, Point p)
{
   int y = p.y();
   double a = 0, b = 0;
   if (y < TOP_IMAGE_ROWS) {
      if (isHeadTiltedForward(info_out)) {
         a = 0.218;
         b = -8.86;
      } else {
         a = 0.21;
         b = -43.84;
      }
   } else {
      // Remove offset of top image coords
      y -= TOP_IMAGE_ROWS;

      // if neck is tilted, use different parameters
      if (isHeadTiltedForward(info_out)) {
         a = 0.146;
         b = 82.5;
      } else {
         a = 0.151;
         b = 76.77;
      }
   }
   double diameter = a*y + b;
   if (diameter < 0) return 0;

   // Compensation for kinematics error
   diameter += 8;

   return diameter;
}

// Is the robot's head tilted forward?
// Normal pitch is about 1.8, tilted is close to 0.5
// I use 0.3 as a threshold
bool BallDetector::isHeadTiltedForward(VisionInfoOut& info_out)
{
   // Should we be accessing the cameraToRR.values which was private data?
   double neck_pitch = info_out.cameraToRR->values.joints.angles[Joints::HeadPitch];
   if (!isnan(neck_pitch) && neck_pitch > HEADTILTLIMIT) {
      return true;
   }
   return false;
}

std::string BallDetector::getFeatureSummary(VisionInfoOut& info_out, BallDetectorVisionBundle &bdvb) {
    VisionInfoIn info_in;
    info_in.latestAngleX = 0;
    getSizeEst(bdvb, info_in, info_out);

    preProcessAdaptive(bdvb);

    processInternalRegions(*bdvb.region, bdvb, bdvb.circle_fit.result_circle, bdvb.internal_regions);

    std::ostringstream s;

    s << "Density: " << bdvb.region->getDensity()
      << "\nSize est (diam): " << bdvb.diam_size_est << " Expected size (diam): " << bdvb.diam_expected_size
      << "\nBall: " << bdvb.ball.imageCoords.x() << "," << bdvb.ball.imageCoords.y() << " r: " << bdvb.ball.radius
      << bdvb.internal_regions.getSummary()
      << bdvb.circle_fit.getSummary();

    return s.str();
}

// **************************** CONNECTED COMPONENT ANALYSIS *******************************************
// CCA for regions inside a circle
void BallDetector::connectedComponentAnalysisNotWhiteAndInside(const RegionI& base_region,
    BallDetectorVisionBundle &bdvb,
    RANSACCircle &circle)
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

    // Critical points where the x value is inside the circle
    int critical_point_left;
    int critical_point_right;
    int critical_point_above_left = 0;
    int critical_point_above_right = 0;

    bool groupCapHit = false;

    for(int pixel=0; pixel < cols*rows && !groupCapHit;)
    {
        critical_point_left = calculateCircleLeft(y, cols, circle);
        // If calculateCircleLeft has returned cols then there is no circle on this row
        // Increment all counters and continue the main loop
        if (critical_point_left == cols) {
            for (int i = 0; i < cols; i++) {
                ++x;
                ++group;
                ++cur_point;
                ++pixel;
            }
            if(x == cols)
            {
                // Store the previous critical point
                critical_point_above_left = critical_point_left;
                x = 0;
                ++y;
            }
            continue;
        }
        // Else increment counters to the critical point
        while (x < critical_point_left) {
            ++x;
            ++group;
            ++cur_point;
            ++pixel;
        }
        // Find the next critical point and do regular CCA up to and including it
        critical_point_right = calculateCircleRight(y, cols, circle);
        while (x <= critical_point_right) {
            //Normal Connected component analysis.
            // If this is not a white pixel, group it.
            if (cur_point.colour() != cWHITE)
            {
                // Get all neighbours.
                has_neighbour = false;
                // Check: the left pixel exists, the left pixel is not past the crit point, the left pixel != cWHITE
                if ((x > critical_point_left) && (cur_point.colourLeft() != cWHITE))
                {
                    left_neighbour = *(group-1);
                    has_neighbour = true;
                }
                else
                    left_neighbour = USHRT_MAX;
                // Check: the above pixel exists, the above row is some part in the circle,
                // the above pixel is between the critical points, the above pixel != cWHITE
                if ((y != 0) && (critical_point_above_left != cols) && (x >= critical_point_above_left) && (x <= critical_point_above_right) &&
                    (cur_point.colourAbove() != cWHITE))
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
                    {
                        groupCapHit = true;
                        break;
                    }

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
            }
            ++x;
            ++group;
            ++cur_point;
            ++pixel;
        }
        while (x < cols) {
            ++x;
            ++group;
            ++cur_point;
            ++pixel;
        }
        if(x == cols)
        {
            // Store the previous critical points
            critical_point_above_left = critical_point_left;
            critical_point_above_right = critical_point_right;
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

//TODO: This function should probably be templated and combined with the above function
// CCA for regions without a concern for a circle
void BallDetector::connectedComponentAnalysisNotWhite(const RegionI& base_region, BallDetectorVisionBundle &bdvb)
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

bool BallDetector::shouldRunCrazyBallDetector(const VisionInfoIn& info_in)
{
    return false;
    // return(info_in.behaviour_crazy_ball_override);
    // return(info_in.behaviour_crazy_ball_override ||
    //     (info_in.currentRole == 2 && last_normal_ball_ < 100 &&
    //                       last_normal_ball_ > 5 && last_ball_distance_ < 1000));
}

void BallDetector::getAverageBrightness(BallDetectorVisionBundle &bdvb)
{
    // for (int row = 0; row < bdvb.region->getRows(); row++) {
    //     std::cout << "\n";
    //     for (int col = 0; col < bdvb.region->getCols(); col++) {
    //         if (bdvb.region->getPixelColour(col, row) == 1) {
    //             std::cout << "  ";
    //         }
    //         else {
    //             std::cout << " o";
    //         }
    //     }
    // }

    RegionI::iterator_fovea cur_point_fov = bdvb.region->begin_fovea();
    RegionI::iterator_raw cur_point_raw = bdvb.region->begin_raw();

    // The number of rows and columns in the region.
    int rows = bdvb.region->getRows();
    int cols = bdvb.region->getCols();

    // Data
    int whites = 0;
    int total_raw = 0;

    // Loop
    for(int pixel = 0; pixel < cols*rows; ++pixel)
    {
        if (cur_point_fov.colour() == 1) {
            total_raw += int(*cur_point_raw.raw());
            whites++;
        }
        ++cur_point_fov;
        ++cur_point_raw;
    }
    if (whites == 0) {
        whites++;
    }
    // Store
    bdvb.avg_brightness = total_raw / whites;

#ifdef BALL_DEBUG
    std::cout << "Whites: " << whites << "Total: " << total_raw << "\n";
    std::cout << "AVG: " << bdvb.avg_brightness << "\n";
#endif // BALL_DEBUG
}

void BallDetector::calculateAdaptiveValuesForCircleFitting(BallDetectorVisionBundle &bdvb)
{
    int rows = bdvb.region->getRows();
    int win, per = 0;
    if (bdvb.region->isTopCamera()) {
        win = rows * 0.40;
        per = -5;
    }
    else {
        win = rows * 0.40;
        per = -5;
    }
    // If window size is negtive it will use the last correct window size
    if (win <= 0) {
        bdvb.window_size = DEFAULT_ADAPTIVE_THRESHOLDING_WINDOW_SIZE;   // Dummy value
    }
    else {
        bdvb.window_size = win;
    }
    bdvb.percentage = per;
#ifdef BALL_DEBUG
    std::cout << "Calc AT Values for CF -\n";
    std::cout << "Window Size: " << bdvb.window_size << " Percentage: " << bdvb.percentage << "\n";
#endif //BALL_DEBUG
}

void BallDetector::calculateAdaptiveValuesForInternalRegions(BallDetectorVisionBundle &bdvb)
{
    int rows = bdvb.region->getRows();
    int win, per = 0;
    if (bdvb.region->isTopCamera()) {
        win = rows * 0.30;
        per = int(bdvb.avg_brightness / 10);
    }
    else {
        win = rows * 0.50;
        per = 10;
    }
    // If window size is negtive it will use the last correct window size
    if (win <= 0) {
        bdvb.window_size = DEFAULT_ADAPTIVE_THRESHOLDING_WINDOW_SIZE;   // Dummy value
    }
    else {
        bdvb.window_size = win;
    }
    bdvb.percentage = per;
#ifdef BALL_DEBUG
    std::cout << "Calc AT Values for IR -\n";
    std::cout << "Window Size: " << bdvb.window_size << " Percentage: " << bdvb.percentage << "\n";
#endif //BALL_DEBUG
}

void BallDetector::calculateAdaptiveValuesForBlobROI(BallDetectorVisionBundle &bdvb)
{
    int rows = bdvb.region->getRows();
    int cols = bdvb.region->getCols();

    // Assuming the region that has the potential ball is as wide OR high as the ball
    int val = std::min(rows, cols);
    int win, per = 0;
    if (bdvb.region->isTopCamera()) {
        win = val * 0.50;
        per = int(bdvb.avg_brightness / 10);
    }
    else {
        win = val * 0.60;
        per = 10;
    }
    // If window size is negtive it will use the last correct window size
    if (win <= 0) {
        bdvb.window_size = DEFAULT_ADAPTIVE_THRESHOLDING_WINDOW_SIZE;   // Dummy value
    }
    else {
        bdvb.window_size = win;
    }
    bdvb.percentage = per;
#ifdef BALL_DEBUG
    std::cout << "Calc AT Values for blobROI -\n";
    std::cout << "Window Size: " << bdvb.window_size << " Percentage: " << bdvb.percentage << "\n";
#endif //BALL_DEBUG
}

void BallDetector::findRegionTriangles(BallDetectorVisionBundle &bdvb, InternalRegionFeatures &internal_regions, RegionTriangleFeatures &region_triangle_features)
{
    /*
     * --- Finding equilateral triangles in ball regions ---
     * A key region is a fully internal region that is closest to the centre of the circle.
     * Equilateral triangles can only be formed including this key region as a vertice point
     * When constructing all posible combos, vertice sets not including this key region centre will thrown out
     * These combo sets are then passed to a function to check if it is equilateral within a defined error threshold
     */

    // Number of regions
    int region_num = int(internal_regions.groups.size());

    // Internal data structures
    Triangle tri;
    RegionTriangleFeatures tri_features;
    std::vector<Point> region_centres;
    region_centres.reserve(region_num);

    // Circle features
    int cir_x = bdvb.circle_fit.result_circle.centre.x();
    int cir_y = bdvb.circle_fit.result_circle.centre.y();
    int cir_rad = bdvb.circle_fit.result_circle.radius;

    // Key Region
    int key_region_dist = cir_rad;
    int key_region_num = 0;

    // Find the centre of each region
    for (int i = 0; i < region_num; i++) {
        Point centre(((internal_regions.groups[i].max_x - internal_regions.groups[i].min_x) / 2) + internal_regions.groups[i].min_x,
                     ((internal_regions.groups[i].max_y - internal_regions.groups[i].min_y) / 2) + internal_regions.groups[i].min_y);
        // Finding the key region
        if (internal_regions.groups[i].completely_internal) {
            int x_d2 = pow(centre.x() - cir_x, 2);
            int y_d2 = pow(centre.y() - cir_y, 2);
            int dist = sqrt(x_d2 + y_d2);
            if (dist < key_region_dist) {
                key_region_dist = dist;
                key_region_num = i;
            }
        }
        region_centres.push_back(centre);       // Store the centre
    }

#ifdef BALL_DEBUG
    std::cout << "Key region number: " << key_region_num << "\nKey region -" <<
                 " x: " << region_centres[key_region_num].x() <<
                 " y: " << region_centres[key_region_num].y() << "\n";
#endif //BALL_DEBUG

    // For Vatnao
    tri_features.region_centres = region_centres;

    // Clear from memory
    combo_.clear();
    tri_combos_.clear();

    // Reserve memory
    combo_.reserve(4);

    /// Form combinations and return suitable (including key region) combos in member data tri_combos
    int k = 3;                          // Triangles have 3 vertices -- Would you like to know more?
    key_reg_combo_ = false;              // Initalise outside of function
    combinations(0, k, region_centres, key_region_num);

    // Test each combo for equilateral-ness
    for (std::vector<Triangle>::iterator it = tri_combos_.begin(); it != tri_combos_.end(); it++) {
        // If equilateral store its vertices in local struct
        if (equilateralTriangle(bdvb, it->vertices)) {
            tri.vertices = it->vertices;
            tri_features.region_triangles.push_back(tri);
        }
    }
#ifdef BALL_DEBUG
    std::cout << "Triangles found: " << tri_features.region_triangles.size() << "\n";
#endif //BALL_DEBUG

    // Store all data in bdvb structs
    region_triangle_features = tri_features;
}

bool BallDetector::equilateralTriangle(BallDetectorVisionBundle &bdvb, std::vector<Point> triangle_points) {

    int radius2 = bdvb.circle_fit.result_circle.radius * bdvb.circle_fit.result_circle.radius;
    int dist_min = radius2 - radius2 * TRIANGLE_FIT_DIST_RAD_RATIO_MIN;
    int dist_max = radius2 + radius2 * TRIANGLE_FIT_DIST_RAD_RATIO_MAX;

    std::vector<int> distance;
    distance.reserve(3);

    // Find the distance between each region centre with pythagorean theorem
    std::vector<Point>::iterator next_it = triangle_points.begin() + 1;      // Next iterator point

    for (std::vector<Point>::iterator it = triangle_points.begin(); it != triangle_points.end(); it++, next_it++) {
        if (next_it == triangle_points.end()) {  // If the next point is the end
            next_it = triangle_points.begin();   // Reset next point to the beginning
        }
        int x_d2 = pow((*it).x() - (*next_it).x(), 2);
        int y_d2 = pow((*it).y() - (*next_it).y(), 2);
        int dist2 = x_d2 + y_d2;
        if (dist2 < dist_min|| dist2 > dist_max) {

#ifdef BALL_DEBUG
            std::cout << "Dist between reg failed - Dist: " << dist2 << " Dist Min: " << dist_min << " Dist Max: " << dist_max << "\n";
#endif //BALL_DEBUG

            return false;   //Throw out lines that are too long to form the kind of equilateral triangle we are looking for ,m
        }
        distance.push_back(dist2);
    }
    int avg_error = (abs(distance[0] - distance[1]) + abs(distance[1] - distance[2]) + abs(distance[2] - distance[0])) / 3;
    if (avg_error < TRIANGLE_FIT_ERROR_THRESHOLD) {

#ifdef BALL_DEBUG
        std::cout << "Equilateral triangle found\n";
        std::cout << "Vertice 1: " << triangle_points[0].x() << ", " << triangle_points[0].y() << "\n"
                  << "Vertice 2: " << triangle_points[1].x() << ", " << triangle_points[1].y() << "\n"
                  << "Vertice 3: " << triangle_points[2].x() << ", " << triangle_points[2].y() << "\n";
#endif //BALL_DEBUG

        return true;
    }
    else {

#ifdef BALL_DEBUG
        std::cout << "Not Equilateral triangle. Avg error: " << avg_error << "\n";
        std::cout << "Vertice 1: " << triangle_points[0].x() << ", " << triangle_points[0].y() << "\n"
                  << "Vertice 2: " << triangle_points[1].x() << ", " << triangle_points[1].y() << "\n"
                  << "Vertice 3: " << triangle_points[2].x() << ", " << triangle_points[3].y() << "\n";
#endif //BALL_DEBUG
        return false;
    }
}

void BallDetector::combinations(int offset, int k, std::vector <Point> region_centres, int key_region_num) {
    // Create a list of possible combinations of vertice points that include the key region vertice
    Triangle tri;
    int k_x = region_centres[key_region_num].x();       // Key region centre x
    int k_y = region_centres[key_region_num].y();       // Key region centre y

    if (k == 0) {
        // Check if combo includes key region
        for (std::vector<Point>::iterator it = combo_.begin(); it != combo_.end(); it++) {
            if (((it->x() - k_x) + (it->y() - k_y)) == 0) {
                key_reg_combo_ = true;
            }
        }
        // Save combo if includes key region
        if (key_reg_combo_) {
            tri.vertices = combo_;
            tri_combos_.push_back(tri);
            key_reg_combo_ = false;      // Reset
            return;
        }
        // Throw out combos not including the key region
        else {
            return;
        }
    }
    for (int i = offset; i <= int(region_centres.size()) - k; ++i) {
        combo_.push_back(region_centres[i]);
        combinations(i+1, k-1, region_centres, key_region_num);
        combo_.pop_back();
    }
}

void BallDetector::ballRawRange(BallDetectorVisionBundle &bdvb) {

    RegionI::iterator_raw cur_point = bdvb.region->begin_raw();

    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;

    // Min and max raw Y values
    uint8_t minY = 255;
    uint8_t maxY = 0;

    // Current raw value...or a tasty meal?
    uint8_t currY = 0;

    // The number of rows and columns in the region.
    int rows = bdvb.region->getRows();
    int cols = bdvb.region->getCols();

    // Circle centre
    float circ_x = bdvb.circle_fit.result_circle.centre.x();
    float circ_y = bdvb.circle_fit.result_circle.centre.y();
    float circ_r2 = bdvb.circle_fit.result_circle.radius * bdvb.circle_fit.result_circle.radius;


    // Loop
    for(int pixel = 0; pixel < cols*rows; ++pixel)
    {
        if (DISTANCE_SQR((float)x, (float)y, circ_x, circ_y) < circ_r2) {       // If inside circle
            currY = *cur_point.raw();
            minY = std::min(minY, currY);
            maxY = std::max(maxY, currY);
        }
        ++cur_point;
        ++x;
        if(x == cols)
        {
            x = 0;
            ++y;
        }
    }

    // and store
    bdvb.max_y_value = int(maxY);
    bdvb.min_y_value = int(minY);

#ifdef BALL_DEBUG
    std::cout << "Max ball Y: "  << bdvb.max_y_value << " Min ball Y: " << bdvb.min_y_value
              << "\nBall Raw Variance: " << (bdvb.max_y_value - bdvb.min_y_value) << "\n";
#endif // BALL_DEBUG
}

// Debugging tool for blobROI
// Prints in terminal the binary image and region centres
void BallDetector::printRegionsAndCentres(const RegionI& region, std::vector<Point>& centres) {
    bool centre_flag = 0;
    for (int row = 0; row < region.getRows(); row++) {
        std::cout << "\n";
        for (int col = 0; col < region.getCols(); col++) {
            for (std::vector<Point>::iterator itc = centres.begin(); itc != centres.end(); ++itc) {
                int x = (*itc).x();
                int y = (*itc).y();
                if (x == col && y == row) {
                    centre_flag = 1;
                    continue;
                }
            }
            if (region.getPixelColour(col, row) == 1) {
                if (centre_flag) {
                    std::cout << " x";
                    centre_flag = 0;
                }
                else {
                    std::cout << "  ";
                }
            }
            else {
                if (centre_flag) {
                    std::cout << " X";
                    centre_flag = 0;
                }
                else {
                    std::cout << " o";
                }
            }
        }
    }
    std::cout << "\n";
}

// Returns the critical point using the equation of a circle
// the point where the x values enter the circle for that given row
// or cols if that row does not hit the circle
int BallDetector::calculateCircleLeft(int y, int cols, RANSACCircle c) {
    int crit_point = 0;
    int rad2 = c.radius * c.radius;
    int y_minus_k2 = pow(y - c.centre.y(), 2);
    int h = c.centre.x();

    if ((rad2 - y_minus_k2) < 0) {
        return crit_point = cols;
    }
    crit_point = int(std::max(double(0), ceil(-sqrt(rad2-y_minus_k2) + h)));
    crit_point = std::min(cols, crit_point);
    return crit_point;
}

// Returns the critical point using the equation of a circle
// the point where the x values exit the circle for that given row
int BallDetector::calculateCircleRight(int y, int cols, RANSACCircle c) {
    int crit_point = 0;
    int rad2 = c.radius * c.radius;
    int y_minus_k2 = pow(y - c.centre.y(), 2);
    int h = c.centre.x();

    // Should never need this as calculateCircleRight() is never called when
    // (rad2 - y_minus_k2) < 0, but better safe than sqrt(-ve)
    if ((rad2 - y_minus_k2) < 0) {
        return crit_point = cols - 1;
    }
    crit_point = int(std::min(double(cols - 1), floor(sqrt(rad2-y_minus_k2) + h)));
    crit_point = std::max(0, crit_point);
    return crit_point;
}
