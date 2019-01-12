#include "appAdaptor.hpp"

#include "../../../robot/perception/vision/VisionDefinitions.hpp"
#include "generateFrameInfo.hpp"

AppAdaptor::AppAdaptor(string path):
    dumpParser(path),
    world(&dumpParser),
    bbox_top_(BBox(Point(0,0), Point(TOP_SALIENCY_COLS, TOP_SALIENCY_ROWS))),
    bbox_bot_(BBox(Point(0,0), Point(BOT_SALIENCY_COLS, BOT_SALIENCY_ROWS))),
    combined_fovea_(CombinedFovea(
        new Fovea(bbox_top_, TOP_SALIENCY_DENSITY, true, true),
        new Fovea(bbox_bot_, BOT_SALIENCY_DENSITY, false, true)
    ))
{
    offNao = false;

    runswiftVisionAdapter = new VisionAdapter(world.blackboard);
    runswift_vision_ = &runswiftVisionAdapter->vision_;
    
    combined_frame_ = boost::shared_ptr<CombinedFrame>();
}

int AppAdaptor::forward(int numFrames){
    int actualNumFrames = world.forward(numFrames);
    runswiftVisionAdapter->tickProcess();
    return actualNumFrames;
}

int AppAdaptor::back(int numFrames){
    int actualNumFrames = world.back(numFrames);
    runswiftVisionAdapter->tickProcess();
    return actualNumFrames;
}

void AppAdaptor::reload(){
    runswiftVisionAdapter->tickProcess();
}

void AppAdaptor::appGenerateFrameInfo(){
    Blackboard *blackboard = world.blackboard;
    
    camera_to_rr_.pose = readFrom(motion, pose);
    camera_to_rr_.updateAngles(readFrom(kinematics, sensorsLagged));
    camera_to_rr_.findEndScanValues();

    //boost::shared_ptr<CombinedFrame> combined_frame_;
    combined_frame_ = boost::shared_ptr<CombinedFrame>(new CombinedFrame(
        readFrom(vision, topFrame),
        readFrom(vision, botFrame),
        camera_to_rr_,
        combined_frame_
    ));

    combined_fovea_.generate(*(combined_frame_.get()), 
                             ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP,
                             ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP,
                             ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT,
                             ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT);

    frame_info_ = generateFrameInfo(world.blackboard, *combined_fovea_.top_, *combined_fovea_.bot_);
}

AppStatus AppAdaptor::getStatus(){
    AppStatus status;
    status.numTopCameraCols = TOP_IMAGE_COLS;
    status.numTopCameraRows = TOP_IMAGE_ROWS;
    status.numBotCameraCols = BOT_IMAGE_COLS;
    status.numBotCameraRows = BOT_IMAGE_ROWS;
    return status;
}
