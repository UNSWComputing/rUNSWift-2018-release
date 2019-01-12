/*
* This is a detector that finds groups of colour roi's and defines a cluster of these as a robot.
* It WILL pick up things that are NOT robots, for time sake they have just been treated as robots when passing
* it into Blackboard.
* Please contact Kirsten Hendriks for further information.
*/

#include "perception/vision/detector/ClusterDetector.hpp"

// #define ROBOT_DEBUG 1
#define CLUSTER_SIZE_THRESHOLD 100000
// The number of regions required in the cluster
#define MIN_CLUSTER_SIZE 4
#define CLUSTER_DISTANCE_THRESHOLD 10000

void ClusterDetector::detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
    const std::vector<RegionI>& regions = info_middle.roi;
    int boundary_highest_y = NULL;

    // this makes the assumption that if there are more than one boundaries, the first one is the most important
    if (info_out.boundaries.size() > 0) {
        int p1_y = info_out.boundaries[0].imageBoundary.p1.y();
        int p2_y = info_out.boundaries[0].imageBoundary.p2.y();
        boundary_highest_y = p1_y < p2_y ? p1_y : p2_y;
    }

    int num_regions = regions.size();
    std::vector<Cluster> candidate_clusters;

    Cluster current;
    resetCluster(current);
    for (int i = 0; i < num_regions; i++){
        // calculate the slope and intercept of the diagonal from the region
        if (boundary_highest_y == NULL || boundary_highest_y <= regions[i].getBoundingBoxRaw().b.y()) {
            if (current.canAddRegionToCluster(regions[i])) {
                current.addRegionToCluster(regions[i]);
            } else {
                candidate_clusters.push_back(current);
                resetCluster(current);
            }
        }
    }

    // step through candidate clusters and combine them, if a cluster has been combined
    // delete it from the not_yet_merged vector
    std::vector<Cluster> merged;
    std::vector<Cluster> not_yet_merged = candidate_clusters;
    bool didMerge = false;
    for (std::vector<Cluster>::iterator it = candidate_clusters.begin(); it != candidate_clusters.end(); it++) {
        for (std::vector<Cluster>::iterator it_to_merge = not_yet_merged.begin(); it_to_merge != not_yet_merged.end();) {
            // TODO theoretically a cluster should not merge with a cluster before it
            // should be able to reduce checks somehow by using that
            if (*it != *it_to_merge && it->canAddClusterToCluster(*it_to_merge)) {
                it->addClusterToCluster(*it_to_merge);
                didMerge = true;
                not_yet_merged.erase(it_to_merge);
            } else {
                it_to_merge++;
            }
        }
        if (didMerge) {
            merged.push_back(*it);
            didMerge = false;
        }
    };

    int num_candidates = merged.size();
    for (int i = 0; i < num_candidates; i++) {
        findRobot(merged[i], info_in, info_out);
#ifdef ROBOT_DEBUG
    std::cout << "# of regions in cluster: " << candidate_clusters[i].regions.size() << std::endl;
    std::cout << "top: " << candidate_clusters[i].top;
    std::cout << " bottom: " << candidate_clusters[i].bottom;
    std::cout << " left: " << candidate_clusters[i].left;
    std::cout << " right: " << candidate_clusters[i].right << std::endl;
#endif
    }

#ifdef ROBOT_DEBUG
    std::cout << "Found # robots: " << robots_.size() << std::endl;
#endif
    info_out.robots = robots_;
    robots_.clear();
}

void ClusterDetector::findRobot(Cluster& candidate, const VisionInfoIn& info_in, const VisionInfoOut& info_out) {
    BBox bb = BBox(Point(candidate.left, candidate.top), Point(candidate.right, candidate.bottom));
    int width = bb.width();
    int height = bb.height();
    if (width <= height && width * height > CLUSTER_SIZE_THRESHOLD &&
            candidate.isMostlyRegions() &&
            candidate.regions.size() > MIN_CLUSTER_SIZE) {
        // clusters should not extend over both top and bottom cameras so we can just check the first region
        Point pointForRR = Point(bb.b.x() - bb.a.x(),
            bb.b.y() + (!candidate.regions[0].isTopCamera()) * TOP_IMAGE_ROWS);

        int robot_height = 0;

        Point feet(bb.width() / 2 + bb.a.x(), bb.b.y());
        RRCoord rr_feet  = info_out.cameraToRR->convertToRR(feet, false);

        Point b = info_out.cameraToRR->pose.imageToRobotXY(pointForRR, robot_height);

        float diff = 190*tan(info_in.latestAngleX);

        b.y() -= diff;

        RRCoord rr;
        rr.distance() = hypotf(b.y(), b.x());


//        if (rr.distance() <= CLUSTER_DISTANCE_THRESHOLD) {

            //frame.cameraToRR.convertToRR(saliency.mapFoveaToImage(feet), false);

            RobotInfo::Type type = RobotInfo::rUnknown;

            robots_.push_back(RobotInfo(rr_feet, type, bb, RobotInfo::TOP_CAMERA));
  //      }
    }

}

void ClusterDetector::resetCluster(Cluster& cluster) {
    cluster.regions.clear();
    cluster.top = 0;
    cluster.bottom = 0;
    cluster.left = 0;
    cluster.right = 0;
}
