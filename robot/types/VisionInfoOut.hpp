#ifndef VISION_INFO_OUT_HPP
#define VISION_INFO_OUT_HPP

#include <vector>

#include "perception/vision/camera/CameraToRR.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/BallInfo.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/PostInfo.hpp"
#include "types/RobotInfo.hpp"
#include "perception/vision/Region.hpp"

struct VisionInfoOut {
    std::vector<BallInfo> balls;
    std::vector<BallInfo> uncertain_balls;
    std::vector<FieldBoundaryInfo> boundaries;
    std::vector<FieldFeatureInfo> features;
    std::vector<PostInfo> posts;
    std::vector<RobotInfo> robots;

    const CameraToRR *cameraToRR;

    int topStartScanCoords[TOP_IMAGE_COLS];
    int botStartScanCoords[BOT_IMAGE_COLS];
    std::vector<RegionI> regions;
};

#endif
