#include "appAdaptor.hpp"

#include "../../../robot/perception/vision/VisionDefinitions.hpp"
#include "../../../robot/soccer.hpp"

AppAdaptor::AppAdaptor(string path, string colour_cal_top, string colour_cal_bot):
    dumpParser(path),
    world(&dumpParser)
{
    query_.region_index = 0;
    query_.subregion_index = 0;

    vatnao_vdm_ = new VatnaoDebugMiddleware(this);
    vdm = vatnao_vdm_;
    runswiftVisionAdapter = new VisionAdapter(world.blackboard);
    runswift_vision_ = &runswiftVisionAdapter->vision_;
    // runswift_vision_->loadNnmc(true, colour_cal_top);
    // runswift_vision_->loadNnmc(false, colour_cal_bot);
}

int AppAdaptor::forward(int numFrames){
    int actualNumFrames = world.forward(numFrames);
    query_ = query_.withResetRegions();
    process();
    return actualNumFrames;
}

int AppAdaptor::back(int numFrames){
    int actualNumFrames = world.back(numFrames);
    query_ = query_.withResetRegions();
    process();
    return actualNumFrames;
}

void AppAdaptor::process() {
    vatnao_vdm_->resetTopFrame();
    vatnao_vdm_->resetBotFrame();

    runswiftVisionAdapter->tickProcess();
    frame_info_generator_.setPreviewRegion(&vatnao_vdm_->getRegionPreview());
    frame_info_generator_.setAnnotationRegion(&vatnao_vdm_->getRegionAnnotated());
    frame_info_generator_.setFrameMessage(vatnao_debug_message_);
}

void AppAdaptor::addVatnaoOption(VatnaoOption option) {
    vatnao_options_.push_back(option);
    if (option.is_numeric) {
        query_ = query_.withUpdatedNumericOption(option.option_name, option.default_numeric_option());
    } else {
        query_ = query_.withUpdatedOption(option.option_name, option.default_option());
    }
}

void AppAdaptor::setVatnaoQuery(VatnaoQuery query) {
    query_ = query;
}

VatnaoQuery AppAdaptor::getVatnaoQuery() {
    return query_;
}

void AppAdaptor::set_debug_message(string msg) {
    vatnao_debug_message_ = msg;
}

VatnaoFrameInfo AppAdaptor::getFrameInfo(){
    return frame_info_generator_.generateFrameInfo(world.blackboard, query_, vatnao_vdm_);
}

AppStatus AppAdaptor::getStatus(){
    AppStatus status;
    status.numTopCameraCols = TOP_IMAGE_COLS;
    status.numTopCameraRows = TOP_IMAGE_ROWS;
    status.numBotCameraCols = BOT_IMAGE_COLS;
    status.numBotCameraRows = BOT_IMAGE_ROWS;

    status.region_index = query_.region_index;
    status.subregion_index = query_.subregion_index;

    status.options = vatnao_options_;
    status.debug_message = vatnao_debug_message_;
    return status;
}
