#ifndef PERCEPTION_VISION_DETECTOR_CLUSTERDETECTOR_H_
#define PERCEPTION_VISION_DETECTOR_CLUSTERDETECTOR_H_

#include "perception/vision/detector/DetectorInterface.hpp"

// Minumum sum of the region required to be counted
#define CLUSTER_REGION_RATIO_THRESHOLD 0.4

// Minumum sum of the region required to be counted in the bottom half. It should be mostly white because because of the feet.
#define CLUSTER_REGION_RATIO_BOTTOM_THRESHOLD 0.6

// Maximum distance between regions in pixels, to be merged in.
#define CLUSTER_MERGE_THRESHOLD 20

// This will reject wide, short lines.
#define MAX_WIDTH_TO_MERGE 4
// Reject regions that take up too much of the frame. eg: 1/3 of the frame.
#define MAX_WIDTH_REGION_ACCEPTANCE 3

#define MIN_CLUSTER_SIZE 4

struct Cluster {
    std::vector<RegionI> regions;
    // all the values are raw
    int top;
    int bottom;
    int left;
    int right;

    bool canAddRegionToCluster(const RegionI& region) {
        BBox bb = region.getBoundingBoxRaw();
        // Reject very thin regions. These are usually field lines that expend the cluster size outwards
        // and dilutes the density.
        if ((bb.width() / bb.height() > MAX_WIDTH_TO_MERGE) && (bb.width() > IMAGE_COLS/MAX_WIDTH_REGION_ACCEPTANCE)){
            return false;
        }
        if (regions.size() == 0) {
            return true;
        } else {
            return (bb.a.x() - CLUSTER_MERGE_THRESHOLD <= right) &&
            (bb.b.x() + CLUSTER_MERGE_THRESHOLD >= left) &&
            (bb.a.y() - CLUSTER_MERGE_THRESHOLD <= bottom) &&
            (bb.b.y() + CLUSTER_MERGE_THRESHOLD >= top);
        }
    };

    void addRegionToCluster(const RegionI& region) {
        bool firstRegion = regions.size() == 0 ? true : false;
        regions.push_back(region);
        BBox bb = region.getBoundingBoxRaw();
        if (top > bb.a.y() || firstRegion) {
            top = bb.a.y();
        }
        if (bottom < bb.b.y() || firstRegion) {
            bottom = bb.b.y();
        }
        if (left > bb.a.x() || firstRegion){
            left = bb.a.x();
        }
        if (right < bb.b.x() || firstRegion) {
            right = bb.b.x();
        }
    };

    // clusters can be added to each other from a top to bottom manner
    bool canAddClusterToCluster(Cluster& cluster) {
        return cluster.top <= bottom && cluster.bottom >= top && cluster.left <= right && cluster.right >= left;
    };

    void addClusterToCluster(Cluster& cluster) {
        regions.insert(regions.end(), cluster.regions.begin(), cluster.regions.end());
        if (cluster.top < top) {
            top = cluster.top;
        }
        if (cluster.bottom > bottom) {
            bottom = cluster.bottom;
        }
        if (cluster.left < left){
            left = cluster.left;
        }
        if (cluster.right > right) {
            right = cluster.right;
        }
    };

    bool isMostlyRegions() {
        int cluster_size = (bottom - top) * (right - left);
        int cluster_size_bottom_half = cluster_size / 3;
        int region_size = 0;
        int region_size_bottom_half = 0;
        int midpoint = (bottom + top)*2 / 3;
        BBox current;
        for (std::vector<RegionI>::iterator it = regions.begin(); it != regions.end(); it++) {
            current = it->getBoundingBoxRaw();
            int size = current.height() * current.width();

            region_size += size;
            if (current.b.y() > midpoint){
                region_size_bottom_half += size;
            }
        }
        float percentage = (float) region_size / (float) cluster_size;
        float percentage_bottom_half = (float) region_size_bottom_half / float (cluster_size_bottom_half);
        //std::cout << "Percent " << percentage << " bot: " << percentage_bottom_half << std::endl;
        return (percentage > CLUSTER_REGION_RATIO_THRESHOLD) &&
            (percentage_bottom_half > CLUSTER_REGION_RATIO_BOTTOM_THRESHOLD);
    };

    bool operator== (Cluster& other) {
        return other.top == top && other.bottom == bottom && other.left == left && other.right == right;
    };

    bool operator!= (Cluster& other) {
        return other.top != top || other.bottom != bottom || other.left != left || other.right != right;
    }
};

class ClusterDetector : public Detector {
public:
    /**
     * detect implementation of abstract infterface function
     */
     void detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

private:
    /* final list of robots detected */
    std::vector<RobotInfo> robots_;

    void findRobot(Cluster& candidate, const VisionInfoIn& info_in, const VisionInfoOut& info_out);
    void resetCluster(Cluster& cluster);
};

#endif
