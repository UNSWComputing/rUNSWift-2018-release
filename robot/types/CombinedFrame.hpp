#ifndef COMBINED_FRAME_HPP
#define COMBINED_FRAME_HPP

//#include <stdint.h>

#include <boost/shared_ptr.hpp>

#include "perception/vision/camera/CameraToRR.hpp"
#include "perception/vision/camera/CameraDefinitions.hpp"

struct CombinedFrame {
    const uint8_t *top_frame_;
    const uint8_t *bot_frame_;
    const CameraToRR &camera_to_rr_;
    boost::shared_ptr<CombinedFrame> last_;

    CombinedFrame(const uint8_t *top_image, const uint8_t *bot_image,
            const CameraToRR &camera_to_rr,
            boost::shared_ptr<CombinedFrame> last) :
        top_frame_(top_image), bot_frame_(bot_image),
        camera_to_rr_(camera_to_rr), last_(last) {}

    CombinedFrame(const uint8_t *top_image, const uint8_t *bot_image) :
        top_frame_(top_image), bot_frame_(bot_image),
        camera_to_rr_(CameraToRR()),
        last_(boost::shared_ptr<CombinedFrame>()) {}
};


#endif
