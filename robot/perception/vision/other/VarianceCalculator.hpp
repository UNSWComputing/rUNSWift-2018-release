#ifndef PERCEPTION_VISION_VARIANCECALCULATOR_H_
#define PERCEPTION_VISION_VARIANCECALCULATOR_H_

#include <vector>

#include "types/BallInfo.hpp"
#include "types/PostInfo.hpp"
 #include "types/RobotInfo.hpp"
 #include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"

class VarianceCalculator {
public:
   static void setVariance(std::vector<BallInfo>         &balls);
   static void setVariance(std::vector<PostInfo>         &posts);
   static void setVariance(std::vector<RobotInfo>        &robots);
   static void setVariance(std::vector<FieldBoundaryInfo>    &fieldBoundaries);
   static void setVariance(std::vector<FieldFeatureInfo> &fieldFeatures);

private:
   static void setDistanceHeadingVariance(RRCoord &rr);
   static void setDistanceHeadingOrientationVariance(RRCoord &rr);
};

#endif
