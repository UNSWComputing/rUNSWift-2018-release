#ifndef COMBINED_FOVEA_HPP
#define COMBINED_FOVEA_HPP

#include "perception/vision/Fovea.hpp"

struct CombinedFovea {

public:
    CombinedFovea(Fovea* top, Fovea* bot) : top_(top), bot_(bot) {}
    ~CombinedFovea() { free(top_); free(bot_); }
    void generate(const CombinedFrame& this_frame, 
                  const int top_window_size, 
                  const int top_percentage, 
                  const int bot_window_size, 
                  const int bot_percentage) {
        top_->generate(this_frame, top_window_size, top_percentage, true);
        bot_->generate(this_frame, bot_window_size, bot_percentage, true);
    }
    Fovea* top_;
    Fovea* bot_;
};

#endif
