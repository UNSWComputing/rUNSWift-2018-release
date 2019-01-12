#include <vector>
#include <string>

#include "../../../../robot/blackboard/Blackboard.hpp"
#include "../../../../robot/types/BBox.hpp"

#include "../vatnaoFrameInfo.hpp"
#include "../VatnaoQuery.hpp"

#include "../ImageMethods/RgbImg.hpp"
#include "../VatnaoDebugMiddleware/VatnaoDebugMiddleware.hpp"

class FrameInfoGenerator {
    public:
        FrameInfoGenerator();

        VatnaoFrameInfo generateFrameInfo(Blackboard* blackboard, VatnaoQuery query, VatnaoDebugMiddleware *vdm);

        RgbImg* getPreviewRegion();
        RgbImg* getAnnotationRegion();

        void setPreviewRegion(RgbImg* img);
        void setAnnotationRegion(RgbImg* img);
        void setFrameMessage(std::string msg);
    private:
        RgbImg top_frame_;
        RgbImg bot_frame_;
        RgbImg* region_preview_;
        RgbImg* region_annotated_;
        std::string frame_message_;
};

void fillFrameInfo(VatnaoFrameInfo& frame_info, Blackboard* blackboard, VatnaoQuery query, VatnaoDebugMiddleware *vdm);

FrameRect rectFromBoundingBox(BBox bb, bool top, string label);

std::vector<FrameRect> retrieveRegions(Blackboard* blackboard);
std::vector<FrameRect> retrieveBalls(Blackboard* blackboard);
std::vector<FrameLine> retrieveFieldBoundaries(Blackboard* blackboard);
std::vector<FrameLine> retrieveFieldLines(Blackboard* blackboard);
std::vector<FramePoint> retrieveFieldPoints(Blackboard* blackboard);
std::vector<FramePoint> retrieveFieldLinePoints(Blackboard* blackboard);
std::vector<FrameRect> retrieveGoalPosts(Blackboard* blackboard);
std::vector<FrameRect> retrieveRobots(Blackboard* blackboard);
