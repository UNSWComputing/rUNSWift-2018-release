#pragma once

#include <string>
#include <vector>

#include "../../../robot/blackboard/Blackboard.hpp"
#include "../../../robot/perception/vision/VisionAdapter.hpp"
#include "../../../robot/perception/vision/Vision.hpp"

#include "infoGeneration/generateFrameInfo.hpp"
#include "appStatus.hpp"
#include "VatnaoQuery.hpp"
#include "vatnaoFrameInfo.hpp"
#include "world/world.hpp"
#include "world/dumpParser.hpp"

#include "VatnaoDebugMiddleware/VatnaoDebugMiddleware.hpp"

using namespace std;

class AppAdaptor{
    public:
        AppAdaptor(string path, string colour_cal_top, string colour_cal_bot);

        // Move forward or back the given number of frames
        // Return the number of frames actually moved
        int forward(int numFrames = 1);
        int back(int numFrames = 1);
        void process();

        void setVatnaoQuery(VatnaoQuery query);
        VatnaoQuery getVatnaoQuery();

        VatnaoFrameInfo getFrameInfo();

        AppStatus getStatus();

        //
        // Methods for use by VatnaoDebugMiddleware
        //
        void addVatnaoOption(VatnaoOption option);
        void set_debug_message(string msg);

        void addNewOptions(std::vector<VatnaoOption> new_options);

    private:

        DumpParser dumpParser;
        World world;
        VisionAdapter *runswiftVisionAdapter;
        Vision *runswift_vision_;
        VatnaoDebugMiddleware *vatnao_vdm_;
        FrameInfoGenerator frame_info_generator_;
        VatnaoQuery query_;

        vector<VatnaoOption> vatnao_options_;
        string vatnao_debug_message_;
};
