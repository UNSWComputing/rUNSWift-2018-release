#ifndef UTILS_VATNAO_APP_VATNAODEBUGMIDDLEWARE_VATNAODEBUGMIDDLEWARE_HPP
#define UTILS_VATNAO_APP_VATNAODEBUGMIDDLEWARE_VATNAODEBUGMIDDLEWARE_HPP

#include <string>
#include <map>
#include <vector>

#include "OverlayPainter.hpp"
#include "../appStatus.hpp"
#include "../VatnaoQuery.hpp"

#include "../../../../robot/perception/vision/VisionDebuggerInterface.hpp"

// Just pre-define AppAdaptor here instead of including it. Avoids circular
// references and the like... ughhhh.
class AppAdaptor;

class VatnaoDebugMiddleware:public VisionDebugModule {
    public:
    VatnaoDebugMiddleware(AppAdaptor *app_adaptor);

    // Runswift interraction endpoints
    void addOption(std::string option_name);
    void addOption(std::string option_name, std::vector<std::string> options);
    void addNumericOption(std::string option_name);

    VisionDebugQuery getQuery();

    void setDebugMessage(std::string msg);

    VisionPainter* getGivenRegionOverlayPainter(const RegionI& region);

    VisionPainter* getFrameOverlayPainter(int density, bool top);

    // App Adaptor interaction endpoints
    RgbImg& getRegionPreview();
    RgbImg& getRegionAnnotated();
    RgbImg& getTopFrameOverlay();
    RgbImg& getBotFrameOverlay();
    void resetTopFrame();
    void resetBotFrame();

    private:
    AppAdaptor *app_adaptor_;

    RgbImg region_preview_image_;
    RgbImg region_annotated_image_;
    RgbImg top_frame_overlay_;
    RgbImg bot_frame_overlay_;
    OverlayPainter annotation_painter_;
    OverlayPainter top_frame_painter_;
    OverlayPainter bot_frame_painter_;
};

#endif
