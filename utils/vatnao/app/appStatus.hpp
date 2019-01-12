#pragma once

#include <string>
#include <vector>

struct VatnaoOption {
    std::string option_name;
    std::vector<std::string> options;
    bool is_numeric;

    bool is_boolean_option() {
        return is_numeric == false && options.size() == 0;
    }

    static std::string string_from_boolean(bool val) {
        if (val) {
            return "true";
        } else {
            return "false";
        }
    }

    double default_numeric_option() {
        return 0;
    }

    std::string default_option() {
        if (is_boolean_option()) {
            return string_from_boolean(false);
        } else if (is_numeric) {
            throw "Cannot call for default option if is_numeric. Must be default_numeric_option()";
        } else {
            return options[0];
        }
    }

    bool operator==(const VatnaoOption& other) {
        if (option_name != other.option_name) {
            return false;
        }
        if (options.size() != other.options.size()) {
            return false;
        }
        for (size_t i = 0; i < options.size(); i++) {
            if (options[i] != other.options[i]) {
                return false;
            }
        }

        return true;
    }

    bool operator!=(const VatnaoOption& other) {
        return !(operator==(other));
    }
};

struct AppStatus {
    int numTopCameraCols;
    int numTopCameraRows;
    int numBotCameraCols;
    int numBotCameraRows;

    unsigned int region_index;
    unsigned int subregion_index;

    std::vector<VatnaoOption> options;
    std::string debug_message;
};
