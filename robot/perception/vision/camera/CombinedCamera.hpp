#ifndef PERCEPTION_VISION_CAMERA_COMBINEDCAMERA_H_
#define PERCEPTION_VISION_CAMERA_COMBINEDCAMERA_H_

#include <string>

#include "Camera.hpp"
#include "types/CombinedFrame.hpp"
#include "types/CombinedCameraSettings.hpp"

using std::string;

/**
 * CombinedCamera
 * 
 * CombinedCamera abstracts the top and bottom camera
 *
 * The cameras must be set at some point during initialization:
 *
 *     CombinedCamera::topCamera = new SomeCamera(...);
 *     CombinedCamera::botCamera = new SomeCamera(...);
 *
 */
class CombinedCamera {
    public:
        /**
         * CombinedCamera()
         *
         * Pass dumpframes, dumprate and dumpfile if you want to create
         * a yuv recording for both cameras
         */
        CombinedCamera(
            bool dumpframes = false,
            int dumprate = 0,
            string dumpfile = ""
        );
        ~CombinedCamera();

        Camera* getCamera();

        /**
         * getFrame()
         *
         * Return a CombinedFrame object containing the current image
         * from both the top and bottom camera
         */
        const uint8_t * getFrameTop();
        const uint8_t * getFrameBottom();
        static Camera* getCameraTop();
        static Camera* getCameraBot();
        static void setCameraTop(Camera* camera);
        static void setCameraBot(Camera* camera);

        /**
         * getCameraSettings()
         *
         * Return a CombinedCameraSettings object containing the current setttings
         * of the top and the bottom camera
         */
        CombinedCameraSettings getCameraSettings();

    private:
        static Camera *top_camera_;
        static Camera *bot_camera_;
};

#endif
