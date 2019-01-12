#ifndef VATNAO_APP_VATNAOQUERY_HPP_
#define VATNAO_APP_VATNAOQUERY_HPP_

#include <map>
#include <string>

enum Saliency {
    RAW = 0
};

struct VatnaoQuery {
    unsigned int region_index;
    unsigned int subregion_index;

    Saliency saliency;

    std::map<std::string, std::string> custom_options;
    std::map<std::string, double> numeric_options;

    // Immutable Modifier Helpers for VatnaoQuery
    // Copies and returns the copy
    VatnaoQuery withUpdatedOption(std::string option_name, std::string option) {
        VatnaoQuery retval = *this;
        retval.custom_options[option_name] = option;
        return retval;
    }

    VatnaoQuery withUpdatedNumericOption(std::string option_name, double value) {
        VatnaoQuery retval = *this;
        retval.numeric_options[option_name] = value;
        return retval;
    }

    VatnaoQuery withUpdatedSaliency(Saliency value) {
        VatnaoQuery retval = *this;
        retval.saliency = saliency;
        return retval;
    }

    VatnaoQuery withUpdatedRegionIndex(int change) {
        VatnaoQuery retval = *this;
        retval.region_index += change;
        return retval;
    }

    VatnaoQuery withUpdatedSubregionIndex(int change) {
        VatnaoQuery retval = *this;
        retval.subregion_index += change;
        return retval;
    }

    VatnaoQuery withResetRegions() {
        VatnaoQuery retval = *this;
        retval.region_index = 0;
        retval.subregion_index = 0;
        return retval;
    }
};

#endif
