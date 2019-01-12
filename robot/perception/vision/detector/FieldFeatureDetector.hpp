#ifndef PERCEPTION_VISION_DETECTOR_FIELDFEATUREDETECTOR_H_
#define PERCEPTION_VISION_DETECTOR_FIELDFEATUREDETECTOR_H_

#include "perception/vision/detector/DetectorInterface.hpp"

#include "types/VisionInfoOut.hpp"
#include "perception/vision/Region.hpp"
#include "types/VisionInfoIn.hpp"
#include "perception/vision/other/Ransac.hpp"

// Parameters to manage cluster based culling.
#define DENSE_POINT_CLUSTER 160
#define BOX_SIZE_POWERS 10
#define BOX_SIZE (1 << BOX_SIZE_POWERS)
#define BOXES_X ((FIELD_LENGTH/BOX_SIZE)+(FIELD_LENGTH%BOX_SIZE != 0))
#define BOXES_Y ((FIELD_WIDTH/BOX_SIZE)+(FIELD_WIDTH%BOX_SIZE != 0))

class FieldFeatureDetector : public Detector {
public:
    FieldFeatureDetector();
    /**
     * detect implementation of abstract infterface function
     */
     void detect(const VisionInfoIn& info_in, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

private:
    std::vector<FieldFeatureInfo> field_features_; // final feature list
    std::vector<FieldFeatureInfo> temp_features_; // temp feature list
    std::vector<FieldLinePointInfo> field_line_points_; // all candidate points found
    std::vector<Point> landmarks_; // landmarks around the field to look at
    int num_circle_landmarks_;
    const CameraToRR *convertRR_;

    // Temporary storage for points.
    std::vector<FieldLinePointInfo> field_line_points_temp_;
    int clusters[BOXES_X*BOXES_Y];

    // find candidate points and put them in top_candidate_points_ or bot_candidate_points_ depending on localisation
    // also put them in field_line_points_ for offnao
    void findPointsInRegion(VisionInfoOut& info_out, const RegionI& region);

    void findFeatures(VisionInfoOut &frame, unsigned int *seed, int type = 0);

    void findFieldLinesAndCircles(
        const std::vector<FieldLinePointInfo> &points,
        std::vector<FieldFeatureInfo> *features,
        unsigned int *seed);

    void findFieldLines(
        const std::vector<FieldLinePointInfo> &points,
        std::vector<FieldFeatureInfo> *lines,
        unsigned int *seed);

    void findIntersections(
        std::vector<FieldFeatureInfo> &lines,
        std::vector<FieldFeatureInfo> *features);

    void mergeFeatures(std::vector<FieldFeatureInfo> &features);

    void findParallelLines(std::vector<FieldFeatureInfo>* lines);

    void circleOrientation(std::vector<FieldFeatureInfo> *features);

    void findPenaltySpot();

    void reset(bool full);

    void checkLine(const std::vector<FieldLinePointInfo> &points, LineInfo *l1);

    Point intersect(LineInfo l1, LineInfo l2);

    bool perpendicular(LineInfo l1, LineInfo l2);

    bool isBadCorner(LineInfo l1, LineInfo l2);

    float findTAngle(Point p, LineInfo l);

    float findCAngle(Point p, LineInfo l1, LineInfo l2);

    bool possibleT(LineInfo l1, LineInfo l2);

    void findCornerEndpoints(LineInfo l1, LineInfo l2, Point& e1, Point& e2);

    bool closeToEdge(const Fovea &fovea, FieldFeatureInfo f);

    float findGradient(LineInfo l, Point p);

    bool possibleGoalBoxCorner(CornerInfo c, TJunctionInfo t);

    bool isLeftGoalBoxCorner(CornerInfo c, TJunctionInfo t);

    bool parallelPair(
        LineInfo l1,
        LineInfo l2,
        RRCoord* r);

    struct cmpPoints {
       bool operator()(const FieldLinePointInfo &p1,
                 const FieldLinePointInfo &p2) const {
           if (p1.rrp.x() == p2.rrp.x()) return (p1.rrp.y() < p2.rrp.y());
           return (p1.rrp.x() < p2.rrp.x());
        }
    };

};

#endif
