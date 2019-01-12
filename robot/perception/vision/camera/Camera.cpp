#include "perception/vision/camera/Camera.hpp"
#include "utils/Timer.hpp"
#include "utils/Logger.hpp"

using namespace std;

__u32 controlIds[NUM_CONTROLS] =
{V4L2_CID_HFLIP, V4L2_CID_VFLIP,
 V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST,
 V4L2_CID_SATURATION, V4L2_CID_HUE,
 V4L2_CID_SHARPNESS,
 V4L2_CID_AUTO_WHITE_BALANCE,
 V4L2_CID_BACKLIGHT_COMPENSATION,
 V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_EXPOSURE, V4L2_CID_GAIN,
 V4L2_CID_DO_WHITE_BALANCE};

const int NUM_CAMERAS = 2;
__s32 controlValues_lights[NUM_CAMERAS][NUM_CONTROLS] =
{ {1, 1,
   1,
   248, 60,
   130, 0,
   2,
   1, 0x00,
   0,
   50, 250,
   2800},
  {0, 1,
   1,
   250, 64,
   13, 0,
   -55,
   0, 0x00,
   248,
   60, 180,
   0}
};

__s32 (*controlValues)[NUM_CONTROLS] = controlValues_lights;

Camera::Camera() : dumpFile(0) {
  // imageSize = IMAGE_WIDTH * IMAGE_HEIGHT * 2;
}

bool Camera::startRecording(const char *filename, uint32_t frequency_ms) {
   this->frequency_ms = frequency_ms;
   if (dumpFile != NULL) {
      fclose(dumpFile);
   }
   dumpFile = fopen(filename, "w");
   llog(INFO) << "Starting camera dump to file: " << filename << endl;
   return dumpFile != NULL;
}

void Camera::stopRecording() {
   if (dumpFile != NULL) {
      fclose(dumpFile);
      dumpFile = NULL;
   }
   llog(INFO) << "Finishing camera dump to file" << endl;
}

void Camera::writeFrame(const uint8_t*& imageTop,const uint8_t*& imageBot) {
   static Timer t;
   int topSize = 1280*960;
   int botSize = 640*480;
   if (dumpFile != NULL && imageTop != NULL && imageBot != NULL) {
      if (t.elapsed_ms() >= frequency_ms) {
         t.restart();
         llog(DEBUG3) << "Writing frame to dumpFile" << endl;
         int written = fwrite(imageTop, topSize*2, 1, dumpFile);
         llog(DEBUG3) << "wrote " << written << " top frames" << endl;
         fflush(dumpFile);
         written = fwrite(imageBot, botSize*2, 1, dumpFile);
         llog(DEBUG3) << "wrote " << written << " bot frames" << endl;
         fflush(dumpFile);
      }
   }
}
