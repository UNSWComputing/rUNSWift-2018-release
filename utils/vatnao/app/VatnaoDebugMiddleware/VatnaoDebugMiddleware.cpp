#include "VatnaoDebugMiddleware.hpp"

#include <iostream>

#include "../appStatus.hpp"
#include "../appAdaptor.hpp"

VatnaoDebugMiddleware::VatnaoDebugMiddleware(AppAdaptor* app_adaptor):
    region_preview_image_(),   // just use the default
    region_annotated_image_(), // values for these
    top_frame_overlay_(),
    bot_frame_overlay_(),
    annotation_painter_(region_annotated_image_),
    top_frame_painter_(top_frame_overlay_),
    bot_frame_painter_(bot_frame_overlay_)
{
    app_adaptor_ = app_adaptor;
}

void VatnaoDebugMiddleware::addOption(std::string option_name) {
    VatnaoOption option;
    option.option_name = option_name;
    option.is_numeric = false;
    app_adaptor_->addVatnaoOption(option);
}

void VatnaoDebugMiddleware::addOption(std::string option_name, std::vector<std::string> options) {
    VatnaoOption option;
    option.option_name = option_name;
    option.options = options;
    option.is_numeric = false;
    app_adaptor_->addVatnaoOption(option);
}

void VatnaoDebugMiddleware::addNumericOption(std::string option_name) {
    VatnaoOption option;
    option.option_name = option_name;
    option.is_numeric = true;
    app_adaptor_->addVatnaoOption(option);
}

VisionDebugQuery VatnaoDebugMiddleware::getQuery() {
    VatnaoQuery query = app_adaptor_->getVatnaoQuery();
    VisionDebugQuery retval;
    retval.region_index = query.region_index;
    retval.subregion_index = query.subregion_index;
    retval.options = query.custom_options;
    retval.numeric_options = query.numeric_options;

    return retval;
}

void VatnaoDebugMiddleware::setDebugMessage(std::string msg) {
    app_adaptor_->set_debug_message(msg);
}

VisionPainter* VatnaoDebugMiddleware::getGivenRegionOverlayPainter(const RegionI& region) {
    region_preview_image_.fromRegion(region);
    region_annotated_image_.fromRegion(region);
    return &annotation_painter_;
}

VisionPainter* VatnaoDebugMiddleware::getFrameOverlayPainter(int density, bool top) {
    if (top) {
        top_frame_overlay_.setDensity(density);
        return &top_frame_painter_;
    } else {
        bot_frame_overlay_.setDensity(density);
        return &bot_frame_painter_;
    }
}

RgbImg& VatnaoDebugMiddleware::getRegionPreview() {
    return region_preview_image_;
}

RgbImg& VatnaoDebugMiddleware::getRegionAnnotated() {
    return region_annotated_image_;
}

RgbImg& VatnaoDebugMiddleware::getTopFrameOverlay() {
    return top_frame_overlay_;
}

RgbImg& VatnaoDebugMiddleware::getBotFrameOverlay() {
    return bot_frame_overlay_;
}

void VatnaoDebugMiddleware::resetTopFrame() {
    top_frame_overlay_.blank(top_frame_overlay_.getCols(), top_frame_overlay_.getRows());
}

void VatnaoDebugMiddleware::resetBotFrame() {
    bot_frame_overlay_.blank(bot_frame_overlay_.getCols(), bot_frame_overlay_.getRows());
}
