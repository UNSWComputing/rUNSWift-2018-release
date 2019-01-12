#include "perception/vision/detector/BallDetector.hpp"
#include "perception/vision/Region.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/other/otsu.hpp"
#include "perception/vision/other/histogram.hpp"
#include "perception/vision/other/Ransac.hpp"

#include "types/RansacTypes.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/GroupLinks.hpp"
#include "types/Point.hpp"
#include "types/BBox.hpp"

#include "soccer.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <math.h>
#include <climits>
#include <vector>

#define VARIANCE_CHECK_CONFORMITY_WEIGHT  1

// Some threshold values for features that use edges. These depend on contrast and lighting.
#define STRONG_EDGE_THRESHOLD       (500*500)
#define MED_EDGE_THRESHOLD          (300*300)
#define WEAK_EDGE_THRESHOLD         (200*100)
#define ANGLE_THRESHOLD 1
#define EDGE_PORTION_THRESHOLD 0.0
#define UNDEFINED                   (-1)
#define LARGE_NUMBER                1000000

// Used for the internal otsu usage in preprocessing.
#define NUM_OTSU_HISTOGRAM_BUCKETS 256

// The internal regions parameters.
#define MIN_REGION_SIZE 32*32
#define MIN_INTERNAL_REGION_GROUP_SIZE 28
#define MAX_INTERNAL_REGION_GROUP_SIZE 46 // TODO: Should this be adjusted by region size?

#define BLACK_ROI_MIN_GROUP_SIZE 3

// Change in grey value required to be considered a shift in a black/white pattern.
#define ANALYSEPATTERN_BW_THRESHOLD 100
#define ANALYSEPATTERN_BW_SCORE 40

// Percentage of pixels in the bottom that need to be darker than the top.
#define SPHERE_SHADOW_THRESHOLD 0.30
#define SPHERE_SHADOW_EXPECTED_PROPORTION 0.5

// The mimimum confidence that the ball detector checks need to add to, to consider
// the region as a ball.
#define BALL_DETECTOR_CONFIDENCE_THRESHOLD 0.4

// Some preprocessing values for adjusting the brightness.
#define U_V_SUM_THRESHOLD 256
#define EXTRA_Y_SUB 0

// Some check to make sure that the otsu threshold is above a certain number.
#define OTSU_THRESH_MINIMUM 30

// Amount of padding for the region, in pixels.
#define PADDING_WIDTH 1
#define PADDING_HEIGHT 1

// The proportion off a 1.0 width/height ratio a ball can be.
#define BALL_RATIO 1.0

// Only use when things go bad in demos. Override the field boundary.
#define FIELD_BOUNDARY_ESTIMATE 300

#define NORMALISE_PIXEL(val, c) std::min(std::max(((double)val* c) , 0.0), 255.0)

#define HEADTILTLIMIT 0.2

// The portion of white in the bottom camera required to run the extensive ball
// detector.
#define WHITE_HEAVY_THRESHOLD 0.05

//#define BALL_DETECTOR_TIMINGS 1
//#define BALL_DEBUG 1
#define EARLY_EXIT 1 // Find the first ball and stop.

// TODO: move this into the class for goodness sake.
#ifdef BALL_DETECTOR_TIMINGS
#include "utils/Timer.hpp"
    static int roi_time = 0;
    static int roi_count = 0;
    static int confidence_time = 0;
    static int confidence_count = 0;
    static int preprocess_time = 0;
    static int preprocess_count = 0;
    static int circlefit_time = 0;
    static int circlefit_count = 0;
    static int candidate_point_generation_time = 0;
    static int candidate_point_generation_count = 0;
    static int circlefit_iterations = 0;
    static int circlefit_max_iterations = 0;
    static int max_y_value_time = 0;
    static int histogram_in_circle_time = 0;
    static int in_circle_otsu_time = 0;
    static int in_circle_otsu_count = 0;
    static int internal_region_time = 0;
    static int internal_region_count = 0;
    static int frame_time = 0;
    static int frame_count = 0;
    Timer timer;
    Timer timer2;
    Timer timer3;
    Timer frame_timer;
#endif // BALL_DETECTOR_TIMINGS

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

    RegionI *region = new RegionI(*bdvb.region, BBox(newBounds.a, newBounds.b));
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

            if (it->region_created){
                delete it->region;
            }
        }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////// ENTRY POINT TO THE BALL DETECTOR ////////////////////////
////////////////////////////////////////////////////////////////////////////////
void BallDetector::detect(const VisionInfoIn& info_in, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
    // If you are the goalie and it is looking over its shoulder, don't let it detect balls

    // Dear goalie (player 1), please do not look over your shoulder outside the field for balls.
    // 70 degrees
    if (info_in.my_player_number == 1 && fabs(info_in.cameraToRR.values.joints.angles[Joints::HeadYaw]) > M_PI_2 * 7/9) {
        return;
    }

    const std::vector<RegionI> &regions = info_middle.roi;

    unsigned int region_index = regions.size() - 1;
    unsigned int subregion_index = 0;
    
    const char* classNames[] = {"background", "ball"};
    const String modelConfiguration = "model/miniSqueezenet_deploy.prototxt";
    const String modelBinary = "model/miniSqueezenet_new_2_iter_100000.caffemodel";

    //! [Initialize network]
    std::cout << "[INFO] loading model..." << std::endl;
    cv::dnn::Net net = readNetFromCaffe(modelConfiguration, modelBinary);
    //! [Initialize network]

    if (net.empty())
    {
        cerr << "Can't load network by using the following files: " << endl;
        cerr << "prototxt:   " << modelConfiguration << endl;
        cerr << "caffemodel: " << modelBinary << endl;
        return;
    }

#ifdef BALL_DETECTOR_TIMINGS
    frame_timer.restart();
#endif // BALL_DETECTOR_TIMINGS

    for (std::vector<RegionI>::const_reverse_iterator rit = regions.rbegin();
        rit != regions.rend(); ++rit, region_index--)
    {
        // This is our internal ROI

        std::vector <BallDetectorVisionBundle> ball_regions;
        //naiveROI(info_in, &regions[region], info_middle, info_out, true, ball_regions);
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
        comboROI(info_in, *rit, info_middle, info_out, true, ball_regions);
#ifdef BALL_DETECTOR_TIMINGS
            roi_count++;
            roi_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

        subregion_index = 1;
        //std::cout << "SUB_BALLS " << ball_regions.size() << std::endl;
        for (std::vector <BallDetectorVisionBundle>::iterator it = ball_regions.begin();
                it != ball_regions.end(); it++, subregion_index++) {

            //float confidence = confidenceThatRegionIsBall(*it);
            BallDetectorVisionBundle &bdvb = *it;
            uint8_t* image = bdvb.region->getPixelRaw(bdvb.region->getBoundingBoxRaw.a.x(), bdvb.region->getBoundingBoxRaw.a.y());
            cv::Mat frame(image);
            
            Mat inputBlob = blobFromImage(frame, 1.0f, Size(100, 100), Scalar(104, 117, 123), false, false); //Convert Mat to batch of images
            //! [Prepare blob]

            //! [Set input blob]
            net.setInput(inputBlob, "data"); //set the network input
            //! [Set input blob]
            
#ifdef BALL_DETECTOR_TIMINGS
            timer.restart();
#endif // BALL_DETECTOR_TIMINGS
            //! [Make forward pass]
            std::cout << "[INFO] computing object detections..." << std::endl;
            Mat detection = net.forward("detection_out"); //compute output
            //! [Make forward pass]
#ifdef BALL_DETECTOR_TIMINGS
            confidence_count++;
            confidence_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

            Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

            for(int i = 0; i < detectionMat.rows; i++)
            {
                float confidence = detectionMat.at<float>(i, 2);

                size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));
                
                if (confidence > BALL_DETECTOR_CONFIDENCE_THRESHOLD && objectClass == 1){
                    std::cout << "Ball: " << confidence << endl;
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
                    it->ball.imageCoords.x() = (it->region->getBoundingBoxRaw().a.x() + it->region->getBoundingBoxRaw().b.x())/2;
                    it->ball.imageCoords.y() = (it->region->getBoundingBoxRaw().a.y() + it->region->getBoundingBoxRaw().b.y())/2;
                    //it->ball.imageCoords.x() = it->region->getBoundingBoxRaw().a.x() + it->circle_fit.result_circle.centre.x() * it->region->getDensity();
                    //it->ball.imageCoords.y() = it->region->getBoundingBoxRaw().a.y() + it->circle_fit.result_circle.centre.y() * it->region->getDensity();
                    // Adjustment for the old system that continued the coordinate system for the bottom frame.
                    it->ball.topCamera = (it->region->isTopCamera());
                    if (!(it->region->isTopCamera())){
                        it->ball.imageCoords.y() += TOP_IMAGE_ROWS;
                    }
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
                    //naiveROI(info_in, &regions[region], info_middle, info_out, true, ball_regions);
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
                        float confidence = confidenceThatRegionIsBall(*it);
#ifdef BALL_DETECTOR_TIMINGS
                        confidence_count++;
                        confidence_time += timer.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

                        if (confidence > BALL_DETECTOR_CONFIDENCE_THRESHOLD){
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

    if (frame_count == 1000) {
        roi_time /= 1000;
        confidence_time /= 1000;
        preprocess_time /= 1000;
        circlefit_time /= 1000;
        max_y_value_time /= 1000;
        histogram_in_circle_time /= 1000;
        candidate_point_generation_time /= 1000;
        in_circle_otsu_time /= 1000;
        internal_region_time /= 1000;
        frame_time /= 1000;

        std::cout << "FRAMES\nTotal: " << frame_count << " Time: " << frame_time <<
            " %Time: " << 100.0 * frame_time / frame_time <<
            " Time / frame: " << frame_time / frame_count << "\n";;
        std::cout << "REGIONS\nTotal: " << roi_count << " Time: " << roi_time <<
            " %Time: " << 100.0 * roi_time / frame_time <<
            " Time / frame: " << roi_time / frame_count <<
            " Time / region: " << roi_time / roi_count << "\n";

        std::cout << "CONFIDENCES\nTotal: " << confidence_count << " Time: " << confidence_time <<
            " %Time: " << 100.0 * confidence_time / frame_time <<
            " Time / frame: " << confidence_time / frame_count <<
            " Time / region: " << confidence_time / roi_count <<
            " Time / confidence: " << confidence_time / confidence_count << "\n";

        std::cout << "PREPROCESS\nTotal: " << preprocess_count << " Time: " << preprocess_time <<
            " %Time: " << 100.0 * preprocess_time / frame_time <<
            " Time / frame: " << preprocess_time / frame_count <<
            " Time / region: " << preprocess_time / roi_count <<
            " Time / confidence: " << preprocess_time / preprocess_count << "\n";

        std::cout << "CIRCLEFIT\nTotal: " << circlefit_count << " Time: " << circlefit_time <<
            " %Time: " << 100.0 * circlefit_time / frame_time <<
            " Time / frame: " << circlefit_time / frame_count <<
            " Time / region: " << circlefit_time / roi_count <<
            " Time / confidence: " << circlefit_time / circlefit_count << "\n";

        std::cout << "MAX_Y_VALUE\nTotal: " << 0 << " Time: " << max_y_value_time <<
            " %Time: " << 100.0 * max_y_value_time / frame_time <<
            " Time / frame: " << max_y_value_time / frame_count <<
            " Time / region: " << max_y_value_time / roi_count <<
            " Time / confidence: " << max_y_value_time / circlefit_count << "\n";

        std::cout << "HISTOGRAM IN CIRCLE\nTotal: " << 0 << " Time: " << histogram_in_circle_time <<
            " %Time: " << 100.0 * histogram_in_circle_time / frame_time <<
            " Time / frame: " << histogram_in_circle_time / frame_count <<
            " Time / region: " << histogram_in_circle_time / roi_count <<
            " Time / confidence: " << histogram_in_circle_time / circlefit_count << "\n";

        std::cout << "CANDIDATE POINT GENERATION\nTotal: " << candidate_point_generation_count << " Time: " << candidate_point_generation_time <<
            " %Time: " << 100.0 * candidate_point_generation_time / frame_time <<
            " Time / frame: " << candidate_point_generation_time / frame_count <<
            " Time / region: " << candidate_point_generation_time / roi_count <<
            " Time / confidence: " << candidate_point_generation_time / candidate_point_generation_count << "\n";

        std::cout << "IN CIRCLE OTSU\nTotal: " << in_circle_otsu_count << " Time: " << in_circle_otsu_time <<
            " %Time: " << 100.0 * in_circle_otsu_time / frame_time <<
            " Time / frame: " << in_circle_otsu_time / frame_count <<
            " Time / region: " << in_circle_otsu_time / roi_count <<
            " Time / confidence: " << in_circle_otsu_time / in_circle_otsu_count << "\n";

        std::cout << "INTERNAL REGION\nTotal: " << internal_region_count << " Time: " << internal_region_time <<
            " %Time: " << 100.0 * internal_region_time / frame_time <<
            " Time / frame: " << internal_region_time / frame_count <<
            " Time / region: " << internal_region_time / roi_count <<
            " Time / confidence: " << internal_region_time / internal_region_count << "\n";

        std::cout << "CIRCLEFIT ITERATIONS\n" <<
            " Total: " << circlefit_iterations <<
            " Total / region: "<< circlefit_iterations / roi_count <<
            " Max: " << circlefit_max_iterations << "\n";

        roi_time = 0;
        roi_count = 0;
        confidence_time = 0;
        confidence_count = 0;
        preprocess_time = 0;
        preprocess_count = 0;
        circlefit_time = 0;
        circlefit_count = 0;
        candidate_point_generation_time = 0;
        candidate_point_generation_count = 0;
        circlefit_iterations = 0;
        circlefit_max_iterations = 0;
        max_y_value_time = 0;
        histogram_in_circle_time = 0;
        in_circle_otsu_time = 0;
        in_circle_otsu_count = 0;
        internal_region_time = 0;
        internal_region_count = 0;
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
    bdvb.region_created = false;
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

    getSizeEst(bdvb, info_in, info_out);

#ifdef BALL_DEBUG
        std::cout << "Diam_size_est: " << bdvb.diam_size_est << "\n";
        std::cout << "Rows: " << bdvb.region->getRows() << " Cols: " << bdvb.region->getCols() << "\n";
#endif //BALL_DEBUG

    /* Sanity check for size */
    /* Checks the ball estimate size is reasonable, and the region is below the field boundary or on the bottom camera.*/
    //std::cout << "Diam size est " << bdvb.diam_size_est << " AND target bbox" << bdvb.region->getBoundingBoxRaw().a.y() << std::endl;
    if (bdvb.diam_size_est < 150 && bdvb.diam_size_est > 50) {
        /* Do a check for the ball's aspect ratio. Depending on the results, we may need to pass it into another
         * ROI like black ROI to locate the ball inside*/
        double aspect_ratio = checkRegionAspectRatio(region, bdvb);

#ifdef BALL_DEBUG
        std::cout << "aspect_ratio: " << aspect_ratio << "\n";
#endif //BALL_DEBUG

        if (aspect_ratio < 0.5){
            // Too tall, but there might be a ball inside.
            // Black ROI might be able to find it inside.

#ifdef BALL_DEBUG
            std::cout << "Tall\n";
#endif //BALL_DEBUG

            blackROI(info_in, region, info_middle, info_out, true, res);
            //circleROI(info_in, region, info_middle, info_out, true, res);
        } else if (aspect_ratio > 2) {
            // Too short/wide, squarify will fix this.
            // We know that is the case, because the size check on width must have passed to get to this point.
            // TODO: Is this just the general case with a location check? Can't we do location check regardless?
            //

#ifdef BALL_DEBUG
            std::cout << "Short\n";
#endif //BALL_DEBUG

            regenerateRegion(bdvb, true);
            rescaleRegion(bdvb);

            res.push_back(bdvb);
            bdvbAdded = true;

        } else {
            // Roughly good in terms of size checks and ratio. This means we get a good region around
            // a ball if it exists.

#ifdef BALL_DEBUG
            std::cout << "Normal\n";
#endif //BALL_DEBUG

            regenerateRegion(bdvb, false);
            rescaleRegion(bdvb);

            res.push_back(bdvb);
            bdvbAdded = true;

        }
    } else if (bdvb.diam_size_est > 40){
        // When the width size check fails so the region is too wide initially to be a ball.
        // Ask blackROI to locate it inside.
        blackROI(info_in, region, info_middle, info_out, true, res);
        //circleROI(info_in, region, info_middle, info_out, true, res);
    } else {
        // Diam size estimate for the ball is less than 5cm, so it is likely to be noise on the field.
        // Ignore these regions.

    }
#ifdef BALL_DETECTOR_USES_VDM
    if (vdm != NULL && vdm->vision_debug_blackboard.values["Draw This Region"] == 1) {
        bdvb.drawBall();
    }
#endif // BALL_DETECTOR_USES_VDM
    return bdvbAdded;
}

bool BallDetector::naiveROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res) {
    bool bdvbAdded = false;

    // If we don't want to use it
    BallDetectorVisionBundle bdvb;
    bdvb.region = &region;
    bdvb.region_created = false;
    bdvb.is_crazy_ball = false;

    getSizeEst(bdvb, info_in, info_out);

    /* Sanity check for size */
    /* Checks the ball estimate size is reasonable, and the region is below the field boundary or on the bottom camera.*/

    //std::cout << "Diam size est " << bdvb.diam_size_est << " AND target bbox" << bdvb.region->getBoundingBoxRaw().a.y() << std::endl;
    if (!doReject ||
            (bdvb.diam_size_est < 130 && bdvb.diam_size_est > 50 &&
                ((bdvb.region->getBoundingBoxRaw().a.y() > FIELD_BOUNDARY_ESTIMATE) || !(bdvb.region->isTopCamera())))) {
        regenerateRegion(bdvb, true);
        rescaleRegion(bdvb);

        bdvb.ball.imageCoords.y() =
        ((bdvb.region->getBoundingBoxRaw().b.y() - bdvb.region->getBoundingBoxRaw().a.y())*0.5
            + bdvb.region->getBoundingBoxRaw().a.y())
        + (!bdvb.region->isTopCamera()) * TOP_IMAGE_ROWS;

        res.push_back(bdvb);
        bdvbAdded = true;
    }

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
    bdvb.region_created = false;
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

    /*/ TODO: Change this to use the colour saliency image instead?
    for(int pixel=0; pixel < cols*rows; ++pixel)
    {
        if (colour_classifier->classifyTop(cur_point.raw()) == cBLACK) {
            //std::cout << x << ", " << y << "\n";
            if (first) {
                min_x = x;
                max_x = x;
                min_y = y;
                max_y = y;

                first = false;
            }
            else {
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
        }

        ++cur_point;
        ++x;
        if(x == cols)
        {
            x = 0;
            ++y;
        }
    }
    //*/
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

        /*
        int width_padding = 1;
        int height_padding = 1;

        newBounds.a.x() -= width_padding;
        newBounds.b.x() += width_padding;
        newBounds.a.y() -= height_padding;
        newBounds.b.y() += height_padding;
        */

        /*newBounds.a.x() = std::max(0, newBounds.a.x());
        newBounds.a.y() = std::max(0, newBounds.a.y());
        newBounds.b.x() = std::min(bdvb.region->getCols(), newBounds.b.x());
        newBounds.b.y() = std::min(bdvb.region->getRows(), newBounds.b.y());*/

        //std::cout << "newbounds a: " << newBounds.a.x() << "," << newBounds.a.y() << "|" <<
            //" b: " << newBounds.b.x() << "," << newBounds.b.y() << "\n";
        /*
        newBounds.a.x() -= 0.2 * (bdvb.diam_expected_size_pixels / bdvb.region->getDensity());
        newBounds.b.x() += 0.2 * (bdvb.diam_expected_size_pixels / bdvb.region->getDensity());
        newBounds.a.y() -= 0.2 * bdvb.diam_expected_size_pixels / bdvb.region->getDensity();
        */

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

bool BallDetector::circleROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle,
        VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res) {
    //std::cout << "CircleROI\n";
    bool bdvbAdded = false;

    // If we don't want to use it
    BallDetectorVisionBundle bdvb;
    bdvb.region = &region;
    bdvb.region_created = false;
    bdvb.is_crazy_ball = true;

    getSizeEst(bdvb, info_in, info_out);

    regenerateRegion(bdvb, false);
    preProcess(bdvb);

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

double getRowAverage(BallDetectorVisionBundle &bdvb, int row){
    int x=0, y=row;
    double average = 0;
    int n = 1;
    RANSACCircle c = bdvb.circle_fit.result_circle;
    float radius_sq = c.radius * c.radius;
    RegionI region = *bdvb.region;

    for (x=0; x < region.getCols(); ++x){
        if (DISTANCE_SQR((float)x, (float)y, c.centre.x(), c.centre.y()) > radius_sq){
            // outside circle.
            continue;
        } else {
            ++n;
            average += *(region.getPixelRaw(x, y));
        }
    }

    return average/(double)n;
}

/* Attempt to normalise the lighting of the ball and the shadow */
void normaliseLighting(BallDetectorVisionBundle &bdvb){
    /*
    double half_radius = bdvb.circle_fit.result_circle.radius / 2.0;
    double average_top = getRowAverage(bdvb, bdvb.circle_fit.result_circle.centre.y() - half_radius);
    double average_bot = getRowAverage(bdvb, bdvb.circle_fit.result_circle.centre.y() + half_radius);

    std::cout << "Top average = " << average_top << std::endl;
    std::cout << "Bot average = " << average_bot << std::endl;

    int diff = (int) average_top - (int) average_bot;
    // Break out if normalising is not needed.
    if ((average_top - average_bot) < 20){
        return;
    }*/

    // Second attepmt at this problem.

#ifdef BALL_DETECTOR_TIMINGS
    timer3.restart();
#endif // BALL_DETECTOR_TIMINGS
    std::vector<int> y_values = getMaxYPerRowInCircle(*bdvb.region, bdvb.circle_fit.result_circle);
#ifdef BALL_DETECTOR_TIMINGS
    max_y_value_time += timer3.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    // Generally this means the radius is small and circle fit failed.
    if (y_values.size() == 0){
        std::vector<int> top_histogram = makeHistogramInCircle(*bdvb.region, bdvb.circle_fit.result_circle, bdvb.otsu_midpoint_, NUM_OTSU_HISTOGRAM_BUCKETS, true);
        std::vector<int> bot_histogram = makeHistogramInCircle(*bdvb.region, bdvb.circle_fit.result_circle, bdvb.otsu_midpoint_, NUM_OTSU_HISTOGRAM_BUCKETS, false);

        bdvb.otsu_top_threshold_ = getThresholdValueOtsu(top_histogram, bdvb.intra_class_var_top_);
        bdvb.otsu_bot_threshold_ = getThresholdValueOtsu(bot_histogram, bdvb.intra_class_var_bot_);

        // Also fill in the normalised histogram array to prevent crashing.
        bdvb.contrast_row_multiplier.assign(bdvb.region->getRows(), 1);

    } else {
#ifdef BALL_DETECTOR_TIMINGS
        timer3.restart();
#endif // BALL_DETECTOR_TIMINGS
        std::vector<int> norm_histogram = makeNormalisedHistogramInCircle(*bdvb.region, bdvb.circle_fit.result_circle, NUM_OTSU_HISTOGRAM_BUCKETS, y_values, bdvb);
#ifdef BALL_DETECTOR_TIMINGS
        histogram_in_circle_time += timer3.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

        double temp;
        int norm_threshold = getThresholdValueOtsu(norm_histogram, temp);
        bdvb.otsu_top_threshold_ = norm_threshold;
        bdvb.otsu_bot_threshold_ = norm_threshold;
        // Note: If we use normalisation, we want to keep the old intra class variance instead of replace them.
        // This is because normalising will always give a good otsu cut.
        //bdvb.intra_class_var_bot_ = bdvb.intra_class_var_top_;
    }
}

void BallDetector::processInCircleOtsu(BallDetectorVisionBundle &bdvb) {
    // TODO: Test this to see if using the original y form colour ROI is better.
    //bdvb.otsu_midpoint_ = bdvb.circle_fit.result_circle.centre.y();
    bdvb.otsu_midpoint_ = 6000;
    //bdvb.otsu_midpoint_ = std::min(bdvb.original_region_base_y_, (int)bdvb.circle_fit.result_circle.centre.y());

#ifdef BALL_DEBUG
    std::cout << "Old Y is : " << bdvb.original_region_base_y_ << std::endl;
    std::cout << "Circle centre : " << bdvb.circle_fit.result_circle.centre.y() << std::endl;
#endif // BALL_DEBUG

    normaliseLighting(bdvb);

    /*/ Threshold at which we differentiate white and black
    std::vector<int> top_histogram = makeHistogramInCircle(*bdvb.region, bdvb.circle_fit.result_circle, bdvb.otsu_midpoint_, NUM_OTSU_HISTOGRAM_BUCKETS, true);
    std::vector<int> bot_histogram = makeHistogramInCircle(*bdvb.region, bdvb.circle_fit.result_circle, bdvb.otsu_midpoint_, NUM_OTSU_HISTOGRAM_BUCKETS, false);

    bdvb.otsu_top_threshold_ = getThresholdValueOtsu(top_histogram, bdvb.intra_class_var_top_);
    bdvb.otsu_bot_threshold_ = getThresholdValueOtsu(bot_histogram, bdvb.intra_class_var_bot_);
    //*/

#ifdef BALL_DEBUG
    std::cout << "processInCircleOtsu\n";
    std::cout << "Otsu top: " << bdvb.otsu_top_threshold_ << " intra_class_var: " << bdvb.intra_class_var_top_ << "\n";
    std::cout << "Otsu bot: " << bdvb.otsu_bot_threshold_ << " intra_class_var: " << bdvb.intra_class_var_bot_ << "\n";
#endif // BALL_DEBUG
}

float BallDetector::confidenceThatRegionIsBall(BallDetectorVisionBundle &bdvb){

    checkPartialRegion(bdvb);

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    preProcess(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    preprocess_count++;
    preprocess_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    float running_confidence = 0;

    // Early exit if otsu failed

    if (!analyseOtsuIntraClassVar(bdvb.intra_class_var_top_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Top intra class var too low: " << bdvb.intra_class_var_top_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuIntraClassVar(bdvb.intra_class_var_bot_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Bot intra class var too low: " << bdvb.intra_class_var_bot_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuThresh(bdvb.otsu_top_threshold_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Top otsu thresh too low: " << bdvb.otsu_top_threshold_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuThresh(bdvb.otsu_bot_threshold_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Bot otsu thresh too low: " << bdvb.otsu_bot_threshold_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    processCircleFit(*bdvb.region, bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    circlefit_count++;
    circlefit_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    bdvb.ball.imageCoords.x() = bdvb.region->getBoundingBoxRaw().a.x() +
        bdvb.circle_fit.result_circle.centre.x() * bdvb.region->getDensity();

    bdvb.ball.imageCoords.y() = bdvb.region->getBoundingBoxRaw().a.y() +
        bdvb.circle_fit.result_circle.centre.y() * bdvb.region->getDensity() +
        (!bdvb.region->isTopCamera()) * TOP_IMAGE_ROWS;

    bdvb.ball.radius = (bdvb.circle_fit.result_circle.radius) * bdvb.region->getDensity();

    // Rescale region.
    //rescaleRegion(bdvb);

    // Pre process some things. This generates the otsu's threshold.
    //preProcess(bdvb);

    //processHOG(*bdvb.region, bdvb);
    //processPattern(*bdvb.region, bdvb);
    // Circle fit needs to run before internal region check because the resulting circle
    // is used inside.

    // Circle fit already happened inside naive ROI.
    // The midpoint where the otsu's top/bottom switches over.

    // Early exit if circle is not found

    if (bdvb.ball.radius < 2) {
        return 0;
    }
#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    processInCircleOtsu(bdvb);
#ifdef BALL_DETECTOR_TIMINGS
    in_circle_otsu_count++;
    in_circle_otsu_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    if (!analyseOtsuIntraClassVar(bdvb.intra_class_var_top_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Top intra class var too low: " << bdvb.intra_class_var_top_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuIntraClassVar(bdvb.intra_class_var_bot_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Bot intra class var too low: " << bdvb.intra_class_var_bot_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuThresh(bdvb.otsu_top_threshold_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Top otsu thresh too low: " << bdvb.otsu_top_threshold_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

    if (!analyseOtsuThresh(bdvb.otsu_bot_threshold_)) {
        running_confidence = 0;
#ifdef BALL_DEBUG
    std::cout << "Bot otsu thresh too low: " << bdvb.otsu_bot_threshold_ << "\n";
#endif // BALL_DEBUG
        return running_confidence;
    }

#ifdef BALL_DETECTOR_TIMINGS
    timer2.restart();
#endif // BALL_DETECTOR_TIMINGS
    processInternalRegions(*bdvb.region, bdvb, bdvb.circle_fit.result_circle, bdvb.internal_regions);
#ifdef BALL_DETECTOR_TIMINGS
    internal_region_count++;
    internal_region_time += timer2.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS
    //processSphereCheck(*bdvb.region, bdvb);

    /*
    // hog.print();
    if (analyseHOG(bdvb.hog)) {
        //std::cout << "HOG argees" << std::endl;
        running_confidence += 0.0;
    }
    // pattern.print();
    if (analysePattern(bdvb.pattern)) {
        // std::cout << "Pattern argees" << std::endl;
        running_confidence += 0.0;
    }*/

    // Circle fit
    /*if (analyseCircleFit(bdvb.circle_fit)) {
        running_confidence += 0.0;
        //std::cout << "Circle fit agrees" << std::endl;
    }*/

    // internal_regions.print();
    if (analyseInternalRegions(bdvb.internal_regions, bdvb)) {
        //std::cout << "Internal Region argees" << std::endl;
        running_confidence += 0.4;
    }

    //*
    if (analyseInternalRegionsTotal(bdvb.internal_regions, bdvb)) {
        //std::cout << "Internal Region argees" << std::endl;
        running_confidence += 0.2;
    }
    //*/

    /*
    if (analyseInternalRegionCircles(internal_region_circles)) {
         //std::cout << "Internal Region circles argees" << std::endl;
        running_confidence += 0.4;
    }
    */
    /*
    // sphere_check.print();
    if (analyseSphereCheck(bdvb.sphere_check)) {
        // std::cout << "Sphere Check argees" << std::endl;
        running_confidence += 0.0;
    }*/

    return running_confidence;
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

    const RegionI *new_region = new RegionI((*bdvb.region), 1 << density_change, DENSITY_DECREASE,
                                                             true, false, false);

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
}

void BallDetector::processHOG(const RegionI &region, BallDetectorVisionBundle &bdvb) {
    // Create a histogram of the orientations of edges
    // We now do this just within the ball, instead of the entire fovea
    // Compare this to a benchline for the ball
    // We use 8 buckets for angles

    HOGFeatures hog_features;
    for (int i = 0; i < NUM_ANGLE_BINS; i++){
        hog_features.angle_counts[i] = 0;
    }
    hog_features.total_edge_magnitude = 0;
    hog_features.edge_count = 0;

    for (RegionI::iterator_fovea it = region.begin_fovea(); it < region.end_fovea(); ++it)
    {
        if (it.edgeMagnitude() < STRONG_EDGE_THRESHOLD)
            continue;

        // Point analysis to figure out the bin that this point belongs to.

        //      _2|1_
        //       3|4
        Point edge = it.edge();
        int quadrant = 0;
        if (edge.x() >= 0){
            quadrant = (edge.y() >= 0) ? 1 : 4;
        } else {
            quadrant = (edge.y() >= 0) ? 2 : 3;
        }

        // Break quadrant up. Note that we need to use abs() for some of the
        // vectors because x, y may be negative in some quadrants.
        if (quadrant == 1){
            if (edge.x() > edge.y()){
                ++hog_features.angle_counts[0];
            } else {
                // edge.y() > edge.x()
                ++hog_features.angle_counts[1];
            }
        } else if (quadrant == 2) {
            if (edge.y() > abs(edge.x())){
                ++hog_features.angle_counts[2];
            } else {
                ++hog_features.angle_counts[3];
            }
        } else if (quadrant == 3) {
            if (abs(edge.x()) > abs(edge.y())){
                ++hog_features.angle_counts[4];
            } else {
                ++hog_features.angle_counts[5];
            }
        } else {
            if (abs(edge.y()) > edge.x()){
                ++hog_features.angle_counts[6];
            } else {
                ++hog_features.angle_counts[7];
            }
        }

        /* Sum the edge magnitude */
        hog_features.total_edge_magnitude += it.edgeMagnitude();
        ++hog_features.edge_count;
    }

    hog_features.opposites[0] = hog_features.angle_counts[0] + hog_features.angle_counts[4];
    hog_features.opposites[1] = hog_features.angle_counts[1] + hog_features.angle_counts[5];
    hog_features.opposites[2] = hog_features.angle_counts[2] + hog_features.angle_counts[6];
    hog_features.opposites[3] = hog_features.angle_counts[3] + hog_features.angle_counts[7];

    int density = region.getFoveaDensity();
    float total_pixels = (float)(region.getCols()*region.getRows()*density);

    //TODO: Is this density squared? Trying to do a fair comparison across
    // regions of different densities.
    hog_features.edge_density = (hog_features.edge_count*density*density)/total_pixels;
    bdvb.hog = hog_features;
}

bool BallDetector::analyseOtsuIntraClassVar(double var) {
    return var >= 1e7;
}

bool BallDetector::analyseOtsuThresh(int thresh) {
    return thresh >= OTSU_THRESH_MINIMUM;
}

bool BallDetector::analyseHOG(HOGFeatures &hog_features)
{
    if (hog_features.edge_density < EDGE_PORTION_THRESHOLD){
        return false;
    }

    /* 30% of the items fall into one bin */
    int max_size_one = hog_features.edge_count*0.30;

    /* 50% of the items fall into opposite bins */
    int max_size_both = hog_features.edge_count*0.50;

    for (int i = 0; i < NUM_ANGLE_BINS; ++i) {
        /* For lines, we would have two opposite bins high and the rest quite low. */
        if (hog_features.angle_counts[i] > max_size_one) {
            return false;
        }
    }

    for (int i = 0; i < NUM_ANGLE_BINS/2; ++i) {
        /* For lines, we would have two opposite bins high and the rest quite low. */
        if (hog_features.opposites[i] > max_size_both) {
            return false;
        }
    }

    return true;
}

void BallDetector::processPattern(const RegionI &region, BallDetectorVisionBundle &bdvb) {
    /* Halfway, quarter and three quarter scan lines respectively */
    int mid_point_x = region.getCols()/2;
    int mid_point_y = region.getRows()/2;

    // Scan down the midpoints looking for breaks in white ROI.

    PatternFeatures pattern_features;
    pattern_features.vertical_swaps = 0;
    pattern_features.horizontal_swaps = 0;

    bool is_white = false;
    for (int y = 0; y < region.getRows(); ++y){
        /* Inspect down the midpoint */
        if ((region.getPixelColour(mid_point_x, y) == cWHITE && !is_white) || (region.getPixelColour(mid_point_x, y) != cWHITE && is_white) ){
            ++y; // Additional increment to y, to move it two pixels down.
            ++pattern_features.vertical_swaps;
            is_white = !is_white;
        }
    }

    is_white = false;
    for (int x = 2; x < region.getRows(); ++x){
        /* Inspect across the midpoint */

        if ((region.getPixelColour(x, mid_point_y) == cWHITE && !is_white) || (region.getPixelGrey(x, mid_point_y) != cWHITE && is_white)){
            ++x;
            ++pattern_features.horizontal_swaps;
            is_white = !is_white;
        }
    }
    bdvb.pattern = pattern_features;
}

bool BallDetector::analysePattern(PatternFeatures &pattern_features)
{
    if (pattern_features.vertical_swaps > 3 || pattern_features.horizontal_swaps > 3){
        return true;
    } else {
        return false;
    }
}

/* Obervation that the lower part of the ball region of interest will be darker than the upper half.
 * This is because of the ball's spherical shape, which casts a shadow on the ground and have an
 * underside which is darker because less light is hitting the surface.
 * This spherical/3d shape is a feature of a ball, but not for field lines.
 * Because we are using grey values, we could get lucky and pick up black ball patches too.
 *
 * Limitations:
 * - May not work in a dynamic lighting environment with directed lighting.
 * - May not work as well in bottom camera, where a robot is standing directly above the ball.
 * - May not work for regions in other robots.
 */
void BallDetector::processSphereCheck(const RegionI& region, BallDetectorVisionBundle &bdvb){
    SphereCheckFeatures sphere_check_features;
    sphere_check_features.num_darker_lower_pixels = 0;
    sphere_check_features.num_pixels_examined = region.getCols();

    int top_quarter = region.getRows()/4;
    int bot_quarter = top_quarter * 3;

    for (int x = 0; x < region.getCols(); ++x){
        // TODO: Could we be using a %difference (eg: 30% darker) instead of a static value.
        // TODO: Switch from reating PixelGrey to reading raw Y values
        if ((region.getPixelGrey(x, top_quarter) - region.getPixelGrey(x, bot_quarter)) > SPHERE_SHADOW_THRESHOLD ){
            ++sphere_check_features.num_darker_lower_pixels;
        }
    }

    bdvb.sphere_check = sphere_check_features;
}

bool BallDetector::analyseSphereCheck(SphereCheckFeatures &features) {
    /* Normalise the score. This is the % of pixels where the bottom section is darker than the top. */
    if (float(features.num_darker_lower_pixels)/features.num_pixels_examined > SPHERE_SHADOW_EXPECTED_PROPORTION)  {
        return true;
    } else {
        return false;
    }
}

inline bool isInsideRadius(int px, int py, RANSACCircle c){
    // TODO: If the circle fit is too loose, we can "shrink" it by subtracting from radius below.
    // This would reduce the risk of connecting dark blobs on the outer edges of the circle.
    if (DISTANCE_SQR((float)px, (float)py, c.centre.x(), c.centre.y()) < (c.radius * c.radius)){
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

inline bool isNotWhiteY(int threshold, uint8_t y){
    if (y >= threshold - EXTRA_Y_SUB){
        return false;
    } else {
        return true;
    }
}

inline bool isWhiteY(int threshold, uint8_t y){
    if (y < threshold - EXTRA_Y_SUB){
        return false;
    } else {
        return true;
    }
}

inline bool isOtsuBlack(int threshold, uint8_t y) {
    return y < threshold;
}

inline bool isInvalidPos(Point &pos, int num_cols, int num_rows) {
    return pos.x() < 0 || pos.x() >= num_cols || pos.y() < 0 || pos.y() >= num_rows;
}

inline bool isEdgePos(Point &pos, int num_cols, int num_rows) {
    return pos.x() == 0 || pos.x() == num_cols - 1 || pos.y() == 0 || pos.y() == num_rows - 1;
}

// If outside of the image, then classify is not white

inline bool isWhiteYSafeEdgeCheck(int threshold, uint8_t y, Point &pos, int num_cols, int num_rows) {
    if (isInvalidPos(pos, num_cols, num_rows)) {
        return false;
    }

    return isWhiteY(threshold, y);
}

/* Check if the region is not white, based on OTSU and the ransac circle */
// Anything outside of the radius will be considered as white, for CCA.

inline bool isNotWhiteAndInside(int threshold, uint8_t y, uint8_t u, uint8_t v, int px, int py, RANSACCircle c) {
    return isNotWhiteY(threshold, y) && isInsideRadius(px, py, c);
}

void printPixel(int threshold, uint8_t y, uint8_t u, uint8_t v, int px, int py, RANSACCircle c) {
    std::string to_print = ". ";

    // WARNING: Assumes ransac circle is vaid when passed in.
    if (isOutsideRadius(px, py, c)){
        to_print = "# ";
    } else if (isWhiteY(threshold, y)) {
        to_print = ". ";
    } else {
        to_print = "D ";
    }
    std::cout << to_print;
}

void BallDetector::preProcess(BallDetectorVisionBundle &bdvb){

    // Otsu
    // Top and bottom half regions
    // We split otsu into two halves to take into account the shaddow that can
    // appear on the bottom half of any ball.
    // When scanning the top half, the threshold is set from the top region
    // when scanning the bottom half, the threshold is set from the bottom region
    RegionI top_half_region = RegionI(*bdvb.region, BBox(Point(0, 0), Point(bdvb.region->getCols(), bdvb.region->getRows()/2)), 1, DENSITY_MAINTAIN);
    RegionI bot_half_region = RegionI(*bdvb.region, BBox(Point(0, bdvb.region->getRows()/2), Point(bdvb.region->getCols(), bdvb.region->getRows())), 1, DENSITY_MAINTAIN);

    // The midpoint where the otsu's top/bottom switches over.
    bdvb.otsu_midpoint_ = bdvb.region->getRows()/2;

    // Threshold at which we differentiate white and black
    std::vector<int> top_histogram = makeHistogram(top_half_region, NUM_OTSU_HISTOGRAM_BUCKETS);
    std::vector<int> bot_histogram = makeHistogram(bot_half_region, NUM_OTSU_HISTOGRAM_BUCKETS);

    bdvb.otsu_top_threshold_ = getThresholdValueOtsu(top_histogram, bdvb.intra_class_var_top_);
    bdvb.otsu_bot_threshold_ = getThresholdValueOtsu(bot_histogram, bdvb.intra_class_var_bot_);

#ifdef BALL_DEBUG
    std::cout << "Otsu top: " << bdvb.otsu_top_threshold_ << " intra_class_var: " << bdvb.intra_class_var_top_ << "\n";
    std::cout << "Otsu bot: " << bdvb.otsu_bot_threshold_ << " intra_class_var: " << bdvb.intra_class_var_bot_ << "\n";
#endif // BALL_DEBUG
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

    // TODO: Change these numbers
    //int min_internal_group_size = result_circle.radius * result_circle.radius * 0.1;
    int min_internal_group_size;
    if (bdvb.is_crazy_ball){
        min_internal_group_size = result_circle.radius * result_circle.radius * 0.05;
    } else {
        min_internal_group_size = result_circle.radius * result_circle.radius * 0.1;
    }
    int max_internal_group_size = result_circle.radius * result_circle.radius * M_PI * 0.5;

    int centre_x = result_circle.centre.x();
    int centre_y = result_circle.centre.y();
    double area_circle = result_circle.radius * result_circle.radius * M_PI;

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

bool BallDetector::analyseInternalRegions(InternalRegionFeatures &internal_region_features, BallDetectorVisionBundle &bdvb) {
    //std::cout << "internalregions: " << internal_region_features.num_internal_regions <<
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
    //std::cout << "Internalregions: " << internal_region_features.num_regions << "\n";
    // Check the size of the internal regions. A really long one is most likely a line.
    //*/ We are quite generous because of blurred balls.
    int max_width = bdvb.circle_fit.result_circle.radius * 0.9;
    for (int i=0; i<internal_region_features.num_regions; ++i){
        if (internal_region_features.groups[i].completely_internal &&
           ((internal_region_features.groups[i].max_x - internal_region_features.groups[i].min_x) > max_width
           || (internal_region_features.groups[i].max_y - internal_region_features.groups[i].min_y > max_width)))
        {
            // Failed the width bound check
            return false;
        }
    }
    //*/

    // Check the number of regions
    if (bdvb.is_partial_region){
        return (internal_region_features.num_regions >= 3 && internal_region_features.num_regions <= 7);
    } else {
        return (internal_region_features.num_regions >= 3 && internal_region_features.num_regions <= 7);
    }
}

bool BallDetector::analyseInternalRegionCircles(std::vector <CircleFitFeatures> &internal_region_circles) {
    return internal_region_circles.size() >= 1;
}

GreyscaleHistogramFeatures BallDetector::processGreyscaleHistogram(const RegionI& region, BallDetectorVisionBundle &bdvb) {
    GreyscaleHistogramFeatures features;
    for (int i = 0; i < NUM_GREYSCALE_HISTOGRAM_BINS; i++) {features.normalised_histogram[i] = 0.0;}
    makeNormalisedHistogram(region, features.normalised_histogram, NUM_GREYSCALE_HISTOGRAM_BINS);
    return features;
}

bool BallDetector::analyseGreyscaleHistogram(GreyscaleHistogramFeatures &feat) {
    feat.print();
    return false;
}

void BallDetector::processCircleFit(const RegionI& region, BallDetectorVisionBundle &bdvb){
    /* Generate the candidate points */

    // Ransac
    //float radius = region.getRows() * 0.5;
    float radius = std::max(region.getRows(), region.getCols()) * 0.5;

#ifdef BALL_DETECTOR_TIMINGS
    timer3.restart();
#endif // BALL_DETECTOR_TIMINGS

    //float radius = region.getRows() * 0.4;
    if (bdvb.is_partial_region){
        if (bdvb.partial_ball_side == BALL_SIDE_TOP || bdvb.partial_ball_side == BALL_SIDE_BOTTOM){
            // Readjust the ball size estimate.
            //radius = region.getRows() * 0.9;
            getCircleCandidatePoints(region, bdvb, false);
        } else {
            //getCircleCandidatePoints(region, bdvb, true);
            getCircleCandidatePoints(region, bdvb, false);
            // Don't do the semicircle fit.
        }
    }
    else {
        getCircleCandidatePoints(region, bdvb, true);
    }
#ifdef BALL_DETECTOR_TIMINGS
    candidate_point_generation_count++;
    candidate_point_generation_time += timer3.elapsed_us();
#endif // BALL_DETECTOR_TIMINGS

    trimRegionBasedOnCircleCandidatePoints(bdvb);

    if (bdvb.is_partial_region) {
        //float radius = diam_expected_size_ / 2;

        //float e = radius * 0.15; // 5.0; // was 15.0
        float e = 2.0;
        uint16_t n = std::max((int) (2.5 * radius), 20);
        float stepsize = std::max((int) (radius * 0.05), 3);

        std::vector<bool> *con, cons_buf[2];
        cons_buf[0].resize(bdvb.circle_fit_points.size());
        cons_buf[1].resize(bdvb.circle_fit_points.size());
        con = &cons_buf[0];

        //findLargestCircleFit(bdvb, radius, &con, cons_buf, e, n, 0.8, stepsize, bdvb.partial_ball_side);
        // TODO: Investigate this. Because partial regions are more expensive to circlefit,
        // (centre of the circle could be on the sides), we consider all regions as non partial
        findLargestCircleFit(bdvb, radius, &con, cons_buf, e, n, 0.8, stepsize, BALL_SIDE_TOTAL);

        /*
        float radius_e = radius * 0.1;
        uint16_t k = 20;
        float e = radius * 0.1; // 5.0; // was 15.0
        //uint16_t n = 20;
        //uint16_t n = std::max((int) (0.5 * bdvb.circle_fit_points.size()), 20);
        uint16_t n = std::max((int) (2.5 * radius), 20);
        unsigned int seed = 42;

        //if (region.isTopCamera()) n += 5;

        std::vector<bool> *con, cons_buf[2];
        cons_buf[0].resize(bdvb.circle_fit_points.size());
        cons_buf[1].resize(bdvb.circle_fit_points.size());
        con = &cons_buf[0];
        bdvb.circle_fit.result_circle = RANSACCircle(PointF(0,0), 0.0);

        bdvb.circle_fit.circle_found = RANSAC::findCircleOfRadius3P(bdvb.circle_fit_points, radius, radius_e, &con, bdvb.circle_fit.result_circle, k, e, n, cons_buf, &seed);

        if (bdvb.circle_fit.circle_found){
            //std::cout << "CIRCLE FOUND" << std::endl;
        }
        */
    }
    else {
        radius = std::min(bdvb.region->getRows(), bdvb.region->getCols()) * 0.5;
        //float e = radius * 0.15; // 5.0; // was 15.0
        float e = 2.0;
        uint16_t n = std::max((int) (2.5 * radius), 10);
        // TODO: stepsize should be created in such a way that circle fit is limited to performing x number of iterations
        float stepsize = std::max((int) (radius * 0.05), 3);

        std::vector<bool> *con, cons_buf[2];
        cons_buf[0].resize(bdvb.circle_fit_points.size());
        cons_buf[1].resize(bdvb.circle_fit_points.size());
        con = &cons_buf[0];

        //findLargestCircleFit(bdvb, radius, &con, cons_buf, e, n, 0.8, stepsize, bdvb.partial_ball_side);
        findLargestCircleFit(bdvb, radius, &con, cons_buf, e, n, 0.8, stepsize, BALL_SIDE_TOTAL);
    }
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
                circlefit_iterations++;
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
    circlefit_max_iterations = std::max(circlefit_max_iterations, curr_iterations);
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
                circlefit_iterations++;
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
                    circlefit_max_iterations = std::max(circlefit_max_iterations, curr_iterations);
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
    circlefit_max_iterations = std::max(circlefit_max_iterations, curr_iterations);
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
    //int rows = region.getRows();
    int below_white_black_threshold = bdvb.otsu_top_threshold_;
    int above_white_black_threshold = bdvb.otsu_top_threshold_;
    int white_black_threshold = bdvb.otsu_top_threshold_;
    int otsu_midpoint = bdvb.otsu_midpoint_;

    // Exclusion zone - an area from the centre of the region, that will
    // be ignored in the candidate point generation.
    /*
    int exclusion_zone = 0.25 * region.getRows();
    Point exclusion_p1 = Point(exclusion_zone, exclusion_zone);
    Point exclusion_p2 = Point(exclusion_zone*3, exclusion_zone*3);*/

    // Generate the candidate points
    for (int y = 1; y < region.getRows()-1; ++y){

        // If we drop to the second half of the image, use the bottom threshold.
        if (y > otsu_midpoint){
            // Get out in the shadow case. Under this situation, we have a large shadow so don't bother
            if (semiCircle) {
                break;
            }

            white_black_threshold = bdvb.otsu_bot_threshold_;
            //if ((bdvb.otsu_top_threshold_ - bdvb.otsu_bot_threshold_) > 50) break;
        }
        if (y - 1 > otsu_midpoint) {
            above_white_black_threshold = bdvb.otsu_bot_threshold_;
        }
        if (y + 1 > otsu_midpoint) {
            below_white_black_threshold = bdvb.otsu_bot_threshold_;
        }


        for (int x = 1; x < region.getCols()-1; ++x){
            // Assuming that the bounding box fits nicely around a circle
            /*
            if ((x >= exclusion_p1.x() && x <= exclusion_p2.x()) &&
                    y >= exclusion_p1.y() && y <= exclusion_p2.y()){
                //std::cout << "- ";
                continue;
            }*/

            // Check to see if this white is adjacent to black.
            // Since getPixelRaw returns a pointer to the YUV422 pixel, we just dereference
            // it to get the y value.
            if (! isWhiteY(white_black_threshold, *(region.getPixelRaw(x, y)))){
                //std::cout << "- ";
                continue;
            }

            if ((! isWhiteY(above_white_black_threshold, *(region.getPixelRaw(x-1, y-1)))) ||
                (! isWhiteY(above_white_black_threshold, *(region.getPixelRaw(x  , y-1)))) ||
                (! isWhiteY(above_white_black_threshold, *(region.getPixelRaw(x+1, y-1)))) ||
                (! isWhiteY(white_black_threshold, *(region.getPixelRaw(x-1, y  )))) ||
                (! isWhiteY(white_black_threshold, *(region.getPixelRaw(x+1, y  )))) ||
                (! isWhiteY(below_white_black_threshold, *(region.getPixelRaw(x-1, y+1)))) ||
                (! isWhiteY(below_white_black_threshold, *(region.getPixelRaw(x  , y+1)))) ||
                (! isWhiteY(below_white_black_threshold, *(region.getPixelRaw(x+1, y+1)))) )
            {
                bdvb.circle_fit_points.push_back(Point(x,y));
                //std::cout << "E ";
            } else {
                //std::cout << "- ";
            }
        }
        //std::cout << std::endl;
    }

#ifdef BALL_DEBUG
    std::cout << "GetCircleCandidatePoints: " << bdvb.circle_fit_points.size() << "\n";
#endif // BALL_DEBUG
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

    preProcess(bdvb);

    //processHOG(region, bdvb);
    //processPattern(region, bdvb);
    //processCircleFit(*bdvb.region, bdvb);
    processInternalRegions(*bdvb.region, bdvb, bdvb.circle_fit.result_circle, bdvb.internal_regions);

    //std::vector <CircleFitFeatures> internal_region_circles = processInternalRegionCircles(region, bdvb);
    //processSphereCheck(region, bdvb);

    std::ostringstream s;

    s << "Density: " << bdvb.region->getDensity()
      << "\nSize est (diam): " << bdvb.diam_size_est << " Expected size (diam): " << bdvb.diam_expected_size
      << "\nBall: " << bdvb.ball.imageCoords.x() << "," << bdvb.ball.imageCoords.y() << " r: " << bdvb.ball.radius
      << "\nOtsu (top): " << bdvb.otsu_top_threshold_ << "\nOtsu (bot): " << bdvb.otsu_bot_threshold_  << "\n"
      << bdvb.hog.getSummary()
      << bdvb.pattern.getSummary()
      << bdvb.internal_regions.getSummary()
      << bdvb.sphere_check.getSummary()
      << bdvb.circle_fit.getSummary();

    return s.str();
}

// **************************** CONNECTED COMPONENT ANALYSIS *******************************************
void BallDetector::connectedComponentAnalysisWhite(const RegionI& base_region,
        BallDetectorVisionBundle &bdvb)
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

    // The x value of the topmost pixel
    group_top_x_.clear();

    // The y value of the topmost pixel
    group_top_y_.clear();

    // Set to USHRT_MAX when there is no neighbour.
    uint16_t top_neighbour = USHRT_MAX;
    uint16_t left_neighbour = USHRT_MAX;
    bool has_neighbour = false;

    int white_black_threshold = bdvb.otsu_top_threshold_;
    int above_white_black_threshold = bdvb.otsu_top_threshold_;
    /*
    for (std::vector<int>::iterator it = top_histogram.begin(); it != top_histogram.end(); ++it) {std::cout << *it << " ";}
    std::cout << std::endl;
    for (std::vector<int>::iterator it = bot_histogram.begin(); it != bot_histogram.end(); ++it) {std::cout << *it << " ";}
    std::cout << std::endl;
    std::cout << "Top threshold: " << top_white_black_threshold << " Bot threshold: " << bot_white_black_threshold << std::endl;
    // */

    // Iterators that move through the region.
    RegionI::iterator_raw cur_point = base_region.begin_raw();
    RegionI::iterator_raw above = base_region.begin_raw();
    RegionI::iterator_raw left = base_region.begin_raw();

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
        if (pixel / cols > rows / 2) {
            white_black_threshold = bdvb.otsu_bot_threshold_;
        }
        if(white_black_threshold == bdvb.otsu_bot_threshold_ && pixel/cols > (rows/2)+1)
            above_white_black_threshold = bdvb.otsu_bot_threshold_;

        /*
        // Add second forward slash to above line and get pretty printouts of the regions in ascii
        printPixel(white_black_threshold, *(cur_point.raw()), u, v, x, y, circle);
        if ((pixel + 1) % cols == 0) {
            std::cout << std::endl;
        }
        // */

        // If this is not a white pixel, group it.

        if (isWhiteY(white_black_threshold, *(cur_point.raw())))
        {

            // Get all neighbours.
            has_neighbour = false;
            if (x != 0 && isWhiteY(white_black_threshold, *(left.raw())))
            {
                left_neighbour = *(group-1);
                has_neighbour = true;
            }
            else
                left_neighbour = USHRT_MAX;

            if(y != 0 && isWhiteY(above_white_black_threshold, *(above.raw())))
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
                group_top_x_.push_back(x);
                group_top_y_.push_back(y);
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

        ++cur_point;
        if(pixel > 0)
            ++left;
        if(pixel >= cols)
            ++above;
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
            if(group_top_y_[group] < group_top_y_[owner]) {
                group_top_y_[owner] = group_top_y_[group];
                group_top_x_[owner] = group_top_x_[group];
            }
        }
    }
}

// **************************** CONNECTED COMPONENT ANALYSIS *******************************************
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

    // The x value of the topmost pixel
    group_top_x_.clear();

    // The y value of the topmost pixel
    group_top_y_.clear();

    // Set to USHRT_MAX when there is no neighbour.
    uint16_t top_neighbour = USHRT_MAX;
    uint16_t left_neighbour = USHRT_MAX;
    bool has_neighbour = false;

    int white_black_threshold = bdvb.otsu_top_threshold_;
    int above_white_black_threshold = bdvb.otsu_top_threshold_;
    int otsu_midpoint = bdvb.otsu_midpoint_;
    /*
    for (std::vector<int>::iterator it = top_histogram.begin(); it != top_histogram.end(); ++it) {std::cout << *it << " ";}
    std::cout << std::endl;
    for (std::vector<int>::iterator it = bot_histogram.begin(); it != bot_histogram.end(); ++it) {std::cout << *it << " ";}
    std::cout << std::endl;
    std::cout << "Top threshold: " << top_white_black_threshold << " Bot threshold: " << bot_white_black_threshold << std::endl;
    // */

    // Iterators that move through the region.
    RegionI::iterator_raw cur_point = base_region.begin_raw();
    RegionI::iterator_raw above = base_region.begin_raw();
    RegionI::iterator_raw left = base_region.begin_raw();

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
        if (pixel / cols > otsu_midpoint) {
            white_black_threshold = bdvb.otsu_bot_threshold_;
        }
        if(white_black_threshold == bdvb.otsu_bot_threshold_ && pixel/cols > (otsu_midpoint)+1)
            above_white_black_threshold = bdvb.otsu_bot_threshold_;

        uint8_t u;
        uint8_t v;
        if(base_region.getDensity() > 1 || pixel%2 == 0)
        {
            u = *(cur_point.raw()+1);
            v = *(cur_point.raw()+3);
        }
        else
        {
            u = *(cur_point.raw()-1);
            v = *(cur_point.raw()+1);
        }

        /*
        // Add second forward slash to above line and get pretty printouts of the regions in ascii
        printPixel(white_black_threshold, *(cur_point.raw()), u, v, x, y, circle);
        if ((pixel + 1) % cols == 0) {
            std::cout << std::endl;
        }
        // */

        // If this is not a white pixel, group it.

        if (isNotWhiteAndInside(white_black_threshold, NORMALISE_PIXEL(*(cur_point.raw()), bdvb.contrast_row_multiplier[y]),
                    u, v, x, y, circle))
        {
            uint8_t lu;
            uint8_t lv;
            uint8_t au;
            uint8_t av;
            if(base_region.getDensity() > 1 || pixel%2 == 0)
            {
                lu = *(left.raw()+1);
                lv = *(left.raw()+3);
            }
            else
            {
                lu = *(left.raw()-1);
                lv = *(left.raw()+1);
            }
            if(base_region.getDensity() > 1 || pixel%2 == 0)
            {
                au = *(above.raw()+1);
                av = *(above.raw()+3);
            }
            else
            {
                au = *(above.raw()-1);
                av = *(above.raw()+1);
            }

            // Get all neighbours.
            has_neighbour = false;
            if (x != 0 && isNotWhiteAndInside(white_black_threshold, NORMALISE_PIXEL(*(left.raw()), bdvb.contrast_row_multiplier[y]),
                        lu, lv, x - 1, y, circle))
            {
                left_neighbour = *(group-1);
                has_neighbour = true;
            }
            else
                left_neighbour = USHRT_MAX;

            if(y != 0 && isNotWhiteAndInside(above_white_black_threshold, NORMALISE_PIXEL(*(above.raw()), bdvb.contrast_row_multiplier[y-1]),
                        au, av, x, y - 1, circle))
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
                group_top_x_.push_back(x);
                group_top_y_.push_back(y);
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

        ++cur_point;
        if(pixel > 0)
            ++left;
        if(pixel >= cols)
            ++above;
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
            if(group_top_y_[group] < group_top_y_[owner]) {
                group_top_y_[owner] = group_top_y_[group];
                group_top_x_[owner] = group_top_x_[group];
            }
        }
    }
}

bool BallDetector::shouldRunCrazyBallDetector(const VisionInfoIn& info_in)
{
    return(info_in.behaviour_crazy_ball_override ||
        (info_in.currentRole == 2 && last_normal_ball_ < 100 &&
                          last_normal_ball_ > 5 && last_ball_distance_ < 1000));
}
