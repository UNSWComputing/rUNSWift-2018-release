#include <iterator>
#include <map>
#include <string>

#include <linux/videodev2.h>  // V4L2_CID stuff comes from here ;)


// C++0x or C++11 syntax would be nicer:
//std::map<char, int> CAMERA_CONTROL_NAME_MAP = {
//  {'a', 1},
//  {'b', 3},
//  {'c', 5}
//};
std::map<std::string, int> create_camera_control_name_map() {
    std::map<std::string, int> m;
    // There's a whole universe out there in
    // linux/videodev2.h that Aldebaran neglected to document,
    // that might contain useful stuff :)
    // See also: http://www.onsemi.com/pub_link/Collateral/MT9M114-D.PDF

    // Controls that depend on each other, as setting them
    // in the wrong order causes runswift to hang!
    m["V4L2_CID_EXPOSURE_AUTO"] = V4L2_CID_EXPOSURE_AUTO;   // (V4L2_CID_CAMERA_CLASS_BASE+1)
    m["V4L2_CID_BRIGHTNESS"] = V4L2_CID_BRIGHTNESS;         // (V4L2_CID_BASE+0)
    m["V4L2_CID_EXPOSURE"] = V4L2_CID_EXPOSURE;             // (V4L2_CID_BASE+17)
    m["V4L2_CID_GAIN"] = V4L2_CID_GAIN;                     // (V4L2_CID_BASE+19)

    m["V4L2_CID_AUTO_WHITE_BALANCE"] = V4L2_CID_AUTO_WHITE_BALANCE;  // (V4L2_CID_BASE+12)
    m["V4L2_CID_DO_WHITE_BALANCE"] = V4L2_CID_DO_WHITE_BALANCE;  // (V4L2_CID_BASE+13)    
    m["V4L2_CID_WHITE_BALANCE_TEMPERATURE"] = V4L2_CID_WHITE_BALANCE_TEMPERATURE;  // (V4L2_CID_BASE+26)

    // Controls that seem to be independent of other settings
    m["V4L2_CID_HFLIP"] = V4L2_CID_HFLIP;                   // (V4L2_CID_BASE+20)
    m["V4L2_CID_VFLIP"] = V4L2_CID_VFLIP;                   // (V4L2_CID_BASE+21)

    m["V4L2_CID_CONTRAST"] = V4L2_CID_CONTRAST;             // (V4L2_CID_BASE+1)
    m["V4L2_CID_SATURATION"] = V4L2_CID_SATURATION;         // (V4L2_CID_BASE+2)
    m["V4L2_CID_HUE"] = V4L2_CID_HUE;                       // (V4L2_CID_BASE+3)
    m["V4L2_CID_RED_BALANCE"] = V4L2_CID_RED_BALANCE;       // (V4L2_CID_BASE+14)
    m["V4L2_CID_GAMMA"] = V4L2_CID_GAMMA;                   // (V4L2_CID_BASE+16)
    m["V4L2_CID_SHARPNESS"] = V4L2_CID_SHARPNESS;           // (V4L2_CID_BASE+27)
    m["V4L2_CID_BACKLIGHT_COMPENSATION"] = V4L2_CID_BACKLIGHT_COMPENSATION;  // (V4L2_CID_BASE+28)

    // Additional controls
    m["V4L2_CID_AUTOEXPOSURE"] = V4L2_CID_AUTOEXPOSURE;     // (V4L2_CID_BASE+32)
    m["V4L2_CID_CAM_INIT"] = V4L2_CID_CAM_INIT;             // (V4L2_CID_BASE+33)
    m["V4L2_CID_EXPOSURE_CORRECTION"] = V4L2_CID_EXPOSURE_CORRECTION;  // (V4L2_CID_BASE+34)
    m["V4L2_CID_EXPOSURE_ALGORITHM"] = V4L2_CID_EXPOSURE_ALGORITHM;     // (V4L2_CID_CAMERA_CLASS_BASE+17)
    m["V4L2_CID_POWER_LINE_FREQUENCY"] = V4L2_CID_POWER_LINE_FREQUENCY;      // (V4L2_CID_BASE+24)    

    // New Driver Controls
    m["V4L2_MT9M114_FADE_TO_BLACK"] = V4L2_CID_PRIVATE_BASE;      // (V4L2_CID_PRIVATE_BASE)
    m["V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA"] = V4L2_CID_PRIVATE_BASE+1;      // (V4L2_CID_PRIVATE_BASE+1)
    m["V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA_DARK"] = V4L2_CID_PRIVATE_BASE+2;      // (V4L2_CID_PRIVATE_BASE+2)
    m["V4L2_MT9M114_AE_TARGET_GAIN"] = V4L2_CID_PRIVATE_BASE+3;      // (V4L2_CID_PRIVATE_BASE+3)
    m["V4L2_MT9M114_AE_MIN_VIRT_DGAIN"] = V4L2_CID_PRIVATE_BASE+4;      // (V4L2_CID_PRIVATE_BASE+4)
    m["V4L2_MT9M114_AE_MAX_VIRT_DGAIN"] = V4L2_CID_PRIVATE_BASE+5;      // (V4L2_CID_PRIVATE_BASE+5)
    m["V4L2_MT9M114_AE_MIN_VIRT_AGAIN"] = V4L2_CID_PRIVATE_BASE+6;      // (V4L2_CID_PRIVATE_BASE+6)
    m["V4L2_MT9M114_AE_MAX_VIRT_AGAIN"] = V4L2_CID_PRIVATE_BASE+7;      // (V4L2_CID_PRIVATE_BASE+7)
    return m;
}
std::map<std::string, int> CAMERA_CONTROL_NAME_MAP = create_camera_control_name_map();


std::map<int, std::string> create_camera_control_value_map() {
    std::map<int, std::string> m;
    // TODO: C++0x / C++11 map
    typedef std::map<std::string, int>::iterator it_type;
    for(it_type iterator = CAMERA_CONTROL_NAME_MAP.begin(); iterator != CAMERA_CONTROL_NAME_MAP.end(); iterator++) {
        // iterator->first = key
        // iterator->second = value
        m[iterator->second] = iterator->first;
    }
    return m;
}
std::map<int, std::string> CAMERA_CONTROL_VALUE_MAP = create_camera_control_value_map();