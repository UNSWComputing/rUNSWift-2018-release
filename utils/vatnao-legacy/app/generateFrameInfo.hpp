#include <vector>

#include "../../../robot/blackboard/Blackboard.hpp"
#include "../../../robot/types/BBox.hpp"

#include "vatnaoFrameInfo.hpp"

VatnaoFrameInfo generateFrameInfo(Blackboard* blackboard, 
        Fovea &fovea_top, Fovea &fovea_bot);

FrameRect rectFromBoundingBox(BBox bb, bool top, string label);

std::vector<FrameRect> retrieveRegions(Blackboard* blackboard);
std::vector<FrameRect> retrieveBalls(Blackboard* blackboard);
std::vector<FrameLine> retrieveFieldBoundaries(Blackboard* blackboard);
std::vector<FrameLine> retrieveFieldLines(Blackboard* blackboard);
std::vector<FramePoint> retrieveFieldPoints(Blackboard* blackboard);
std::vector<FramePoint> retrieveFieldLinePoints(Blackboard* blackboard);
std::vector<FrameRect> retrieveGoalPosts(Blackboard* blackboard);
std::vector<FrameRect> retrieveRobots(Blackboard* blackboard);

