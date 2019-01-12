#ifndef PERCEPTION_VISION_CAMERA_CAMERADEFINITIONS_H_
#define PERCEPTION_VISION_CAMERA_CAMERADEFINITIONS_H_

typedef enum {
   WHICH_CAMERA_ERROR = -1,
   TOP_CAMERA         =  0,
   BOTTOM_CAMERA      =  1,
   NUM_CAMERAS        =  2
} WhichCamera;

/*
 * Arbitary focal length of the camera
 **/
#define FOCAL_LENGTH 1
/**
 * Pixel size calculated from FOV of camera and the arbitary focal length
 **/

//#define CAMERA_FOV_W 0.8098327729
#define CAMERA_FOV_W 1.06290551
#define TOP_PIXEL_SIZE (tan(CAMERA_FOV_W / 2) / 640)
#define BOT_PIXEL_SIZE (tan(CAMERA_FOV_W / 2) / 320)

// NOT SURE IF THESE BELOW ARE USED

/**
 * Straight line distance from the neck to the camera
 **/
#define NECK_TO_TOP_CAMERA 86.5847
#define NECK_TO_BOTTOM_CAMERA 53.7234
/**
 * Angle of the camera from the neck
 **/
#define ANGLE_OFFSET_TOP_CAMERA -0.82567
#define ANGLE_OFFSET_BOTTOM_CAMERA -0.33652

#define ANGLE_BOTTOM_CAMERA 0.69813



#endif

