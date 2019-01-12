#pragma once

#include <map>
#include <string>

#define V4L2_MT9M114_FADE_TO_BLACK (V4L2_CID_PRIVATE_BASE)
#define V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA (V4L2_CID_PRIVATE_BASE+1)
#define V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA_DARK (V4L2_CID_PRIVATE_BASE+2)
#define V4L2_MT9M114_AE_TARGET_GAIN (V4L2_CID_PRIVATE_BASE+3)
#define V4L2_MT9M114_AE_MIN_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+4)
#define V4L2_MT9M114_AE_MAX_VIRT_DGAIN (V4L2_CID_PRIVATE_BASE+5)
#define V4L2_MT9M114_AE_MIN_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+6)
#define V4L2_MT9M114_AE_MAX_VIRT_AGAIN (V4L2_CID_PRIVATE_BASE+7)

extern std::map<std::string, int> create_camera_control_name_map;
extern std::map<std::string, int> CAMERA_CONTROL_NAME_MAP;


extern std::map<int, std::string> create_camera_control_value_map;
extern std::map<int, std::string> CAMERA_CONTROL_VALUE_MAP;