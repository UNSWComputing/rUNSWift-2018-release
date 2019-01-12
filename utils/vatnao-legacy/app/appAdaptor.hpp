#pragma once

#include <string>

#include "../../../robot/blackboard/Blackboard.hpp"
#include "../../../robot/perception/vision/VisionAdapter.hpp"
#include "../../../robot/perception/vision/Vision.hpp"

#include "appStatus.hpp"
#include "vatnaoFrameInfo.hpp"
#include "world/world.hpp"
#include "world/dumpParser.hpp"

using namespace std;

class AppAdaptor{
    public:
        AppAdaptor(string path);

        // Move forward or back the given number of frames
        // Return the number of frames actually moved
        int forward(int numFrames = 1);
        int back(int numFrames = 1);

        void reload();

        void appGenerateFrameInfo();

        AppStatus getStatus();

        VatnaoFrameInfo getFrameInfo() { return frame_info_; };

    private:
        DumpParser dumpParser;
        World world;
        VisionAdapter *runswiftVisionAdapter;
        Vision *runswift_vision_;
        string colour_cal_top_;
        string colour_cal_bot_;

        CameraToRR camera_to_rr_;
        boost::shared_ptr<CombinedFrame> combined_frame_;

        // Bounding boxes for full regions
        BBox bbox_top_;
        BBox bbox_bot_;

        CombinedFovea combined_fovea_;

        VatnaoFrameInfo frame_info_;
};
