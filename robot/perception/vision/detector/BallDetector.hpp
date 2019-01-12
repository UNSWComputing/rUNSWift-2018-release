#ifndef PERCEPTION_VISION_DETECTOR_BALLDETECTOR_H_
#define PERCEPTION_VISION_DETECTOR_BALLDETECTOR_H_

#include <iostream>

#include "perception/vision/detector/DetectorInterface.hpp"
#include "perception/vision/Region.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/GroupLinks.hpp"
#include "perception/vision/other/GMM_classifier.hpp"

#include "types/RansacTypes.hpp"

#define MAX_RES TOP_IMAGE_COLS*TOP_IMAGE_ROWS

// #define BALL_DETECTOR_USES_VDM
#ifdef BALL_DETECTOR_USES_VDM

#include "perception/vision/VisionDebuggerInterface.hpp"
#include "soccer.hpp"

#endif // BALL_DETECTOR_USES_VDM

enum PartialBallSide {
    BALL_SIDE_LEFT = 0,
    BALL_SIDE_TOP,
    BALL_SIDE_RIGHT,
    BALL_SIDE_BOTTOM,
    BALL_SIDE_TOTAL
};

enum RegionAspectType {
    NORMAL = 0,
    SHORT = 1,
    TALL = 2,
    UNDEFINED = 3,
};

struct InternalRegion {
    int num_pixels;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    bool completely_internal;

    void print() {
        std::cout << "- num_pixels: " << num_pixels << " completely_internal: " << completely_internal << std::endl;
        std::cout << "  p1: (" << min_x << ", " << min_y << "), p2: (" << max_x << ", " << max_y << ")" << std::endl;
    }

    std::string getSummary() {
        std::ostringstream s;

        s << "- num_pixels: " << num_pixels << " completely_internal: " << completely_internal << std::endl;
        s << "  p1: (" << min_x << ", " << min_y << "), p2: (" << max_x << ", " << max_y << ")" << std::endl;
        return s.str();
    }
};

struct InternalRegionFeatures {
    int num_internal_regions;
    int num_regions;
    double max_internal_region_prop;
    double max_region_density_number;
    std::vector<InternalRegion> groups;

    void print(){
        std::cout << "== Internal Regions ==" << std::endl;
        std::cout << "Num Regions: " << num_regions << " Num Internal: " << num_internal_regions << std::endl;
        for (std::vector<InternalRegion>::iterator it = groups.begin(); it != groups.end(); ++it) { it->print(); }
    }

    std::string getSummary() {
        std::ostringstream s;

        s << "== Internal Regions ==" << std::endl;
        s << "Num Regions: " << num_regions << " Num Internal: " << num_internal_regions << std::endl;
        return s.str();
    }
};

struct CircleFitFeatures {
    bool circle_found;
    float error;
    RANSACCircle result_circle;

    std::string getSummary() {
        std::ostringstream s;

        s << "== CircleFit ==" << std::endl;
        s << "Circle found: " << circle_found << " Centre: " << result_circle.centre.x() << "," << result_circle.centre.y()
          << " Radius: " << result_circle.radius << std::endl;;
        return s.str();
    }


#ifdef BALL_DETECTOR_USES_VDM
    void drawBall(VisionPainter *p) {
        vdm->msg << getSummary() << std::endl;
        if (circle_found) {
            p->drawCircle(
                result_circle.centre.x(), result_circle.centre.y(), result_circle.radius,
                VisionPainter::PINK
            );
        }
    }

#endif // BALL_DETECTOR_USES_VDM
};

struct Triangle {
    std::vector <Point> vertices;
    Triangle() : vertices(3) {}
};

struct RegionTriangleFeatures {
    std::vector <Point> region_centres;
    std::vector <Triangle> region_triangles;
};

struct BallDetectorVisionBundle {
    const RegionI* region;
    const RegionI* original_region;
    const RegionI* modelRegion;
    bool region_created;
    bool model_region_created;

    BallInfo ball;

    int original_region_base_y_;

    std::vector<double> contrast_row_multiplier;

    CircleFitFeatures circle_fit;
    InternalRegionFeatures internal_regions;
    RegionTriangleFeatures region_triangle_features;

    std::vector <Point> circle_fit_points;
    std::vector <Point> circle_center_points;

    double diam_size_est;
    double diam_expected_size;
    double diam_expected_size_pixels;

    // Partial regions are regions that are on the edge of the frame.
    bool is_partial_region;
    bool is_crazy_ball;

    PartialBallSide partial_ball_side;

    // Avg raw brightness of pixels known to be white from orginal saliancy
    float avg_brightness;

    // Values used in adaptive thresholding the ball region
    int window_size;
    int percentage;

    // Used to check if potential ball has the variance requireds
    int max_y_value;
    int min_y_value;

    // Region aspect type decided by comboROI
    RegionAspectType region_aspect_type;

#ifdef BALL_DETECTOR_USES_VDM
    void drawBall() {
        if (vdm != NULL) {
            VisionPainter *p = vdm->getGivenRegionOverlayPainter(*region);

            if (region_created) {
                vdm->msg << "Region Created" << std::endl;
            } else {
                vdm->msg << "Region Not Created" << std::endl;
            }
            vdm->msg << std::endl;
            vdm->msg << "Diam Size Estimated: " << diam_size_est << std::endl;
            vdm->msg << "Diam Size Expected:  " << diam_expected_size;
            vdm->msg << " (" << diam_expected_size_pixels << " pixels)" << std::endl;
            vdm->msg << "Is Partial Region? " << is_partial_region << std::endl;
            vdm->msg << std::endl;
            vdm->msg << std::endl;
            vdm->msg << "Offnao: " << offNao << std::endl;

            circle_fit.drawBall(p);

            vdm->setDebugMessage();
        }
    }

#endif // BALL_DETECTOR_USES_VDM
};

class BallDetector: public Detector {
    public:

        BallDetector() : crazy_ball_cycle_(0), last_normal_ball_(0),
                         estimator(ball) {}

        /**
         * detect implementation of abstract infterface function
         */
        void detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

        /**
         * Determines whether the region is just a simple white blob, or
         * actually contains internal dots that could be ball spots.
         */
        bool checkSimpleBlob(BallDetectorVisionBundle& bdvb);

        void processCircleFitSizeEst(const RegionI& region, BallDetectorVisionBundle &bdvb);

        /* Generate the candidate points */
        void getCircleCandidatePoints(const RegionI& region, BallDetectorVisionBundle &bdvb, bool semiCircle);

        void findCircleKenji(BallDetectorVisionBundle &bdvb);

        void preProcessAdaptive(BallDetectorVisionBundle &bdvb);

        std::string getFeatureSummary(VisionInfoOut& info_out, BallDetectorVisionBundle &bdvb);

        bool analyseCircleFit(CircleFitFeatures &feat);

        void regenerateRegion(BallDetectorVisionBundle &bdvb, bool aspectCheck);
        void regenerateRegionFromCircleFit(BallDetectorVisionBundle &bdvb, RANSACCircle &circle);

        void trimRegionBasedOnCircleCandidatePoints(BallDetectorVisionBundle &bdvb);

        void rescaleRegion(BallDetectorVisionBundle &bdvb);

        void getSizeEst(BallDetectorVisionBundle &bdvb, const VisionInfoIn& info_in, VisionInfoOut& info_out);

        void processInternalRegions(const RegionI &baseRegion, BallDetectorVisionBundle &bdvb,
            RANSACCircle &result_circle, InternalRegionFeatures &internal_regions);

        void processInternalRegionsROI(const RegionI &baseRegion, BallDetectorVisionBundle &bdvb,
            InternalRegionFeatures &internal_regions);

        bool blackROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res);

        bool blobROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out, std::vector <BallDetectorVisionBundle> &res, RegionAspectType region_aspect_type);

        bool circleROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res);

        bool comboROI(const VisionInfoIn& info_in, const RegionI& region, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out, bool doReject, std::vector <BallDetectorVisionBundle> &res);

        // I know this is ugly copied code. But unless we template it (so the compiler knows about it and can inline)
        // Not sure how to make this clean AND efficient
        void connectedComponentAnalysisNotWhiteAndInside(const RegionI& base_region,
            BallDetectorVisionBundle &bdvb,RANSACCircle &circle);
        void connectedComponentAnalysisNotWhite(const RegionI& base_region, BallDetectorVisionBundle &bdvb);

        bool checkPartialRegion(BallDetectorVisionBundle &bdvb);

        void findLargestCircleFit(BallDetectorVisionBundle &bdvb, float max_radius, std::vector<bool> **cons, std::vector <bool> cons_buf[2], float e, unsigned int n,
            float min_radius_prop, float step_size, PartialBallSide partial_ball_side);
        void findBestCircleFit(BallDetectorVisionBundle &bdvb, float max_radius, std::vector<bool> **cons, std::vector <bool> cons_buf[2], float e, unsigned int n,
            float min_radius_prop, float step_size, PartialBallSide partial_ball_side);

        void getAverageBrightness(BallDetectorVisionBundle &bdvb);

        void calculateAdaptiveValuesForCircleFitting(BallDetectorVisionBundle &bdvb);
        void calculateAdaptiveValuesForInternalRegions(BallDetectorVisionBundle &bdvb);
        void calculateAdaptiveValuesForBlobROI(BallDetectorVisionBundle &bdvb);

        void findRegionTriangles(BallDetectorVisionBundle &bdvb, InternalRegionFeatures &internal_regions, RegionTriangleFeatures &region_triangle_features);

        bool equilateralTriangle(BallDetectorVisionBundle &bdvb, std::vector<Point> triangle_points);

        void combinations(int offset, int k, std::vector <Point> region_centres, int key_reg_num);


        void ballRawRange(BallDetectorVisionBundle &bdvb);
    private:
        bool isHeadTiltedForward(VisionInfoOut& info_out);

        float getDiamInImage(VisionInfoOut& info_out, Point p);

        bool analyseInternalRegions(InternalRegionFeatures &internal_region_features, BallDetectorVisionBundle &bdvb);
        bool analyseInternalRegionsTotal(InternalRegionFeatures &internal_region_features, BallDetectorVisionBundle &bdvb);
        bool analyseInternalRegionCircles(std::vector <CircleFitFeatures> &internal_region_circles);
        bool analyseInternalRegionTriangles(RegionTriangleFeatures region_triangle_features);
        bool analyseBallRawRange(BallDetectorVisionBundle &bdvb);

        bool shouldRunCrazyBallDetector(const VisionInfoIn& info_in);
        bool inspectBall(BallDetectorVisionBundle &bdvb);

        void printRegionsAndCentres(const RegionI& region, std::vector<Point>& centres);

        int calculateCircleLeft(int y, int cols, RANSACCircle c);
        int calculateCircleRight(int y, int cols, RANSACCircle c);

        // The set of links between groups generated during CCA. Here to avoid
        // reallocation.
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

        int crazy_ball_cycle_;
        int last_normal_ball_;
        int last_ball_distance_;

        // For tringle combinations
        std::vector <Point> combo_;
        std::vector <Triangle> tri_combos_;
        bool key_reg_combo_;

        // GMM ball classifier
        Estimator estimator;
};
#endif
