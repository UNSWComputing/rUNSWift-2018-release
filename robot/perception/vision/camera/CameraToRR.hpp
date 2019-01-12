#ifndef PERCEPTION_VISION_CAMERA_CAMERATORR_H_
#define PERCEPTION_VISION_CAMERA_CAMERATORR_H_

#include <stdint.h>
#include <math.h>

#include "types/RRCoord.hpp"
#include "types/SensorValues.hpp"
#include "types/RRCoord.hpp"
#include "types/SensorValues.hpp"
#include "types/Point.hpp"
#include "types/RansacTypes.hpp"

#include "perception/vision/camera/Camera.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/camera/CameraDefinitions.hpp"
#include "perception/kinematics/Pose.hpp"


/**
 * CameraToRR
 *
 * Functions for generating "Robot Relative" measurements and conversions.
 *
 * All constant measurements are in mm or radians
 **/

class CameraToRR {
   public:
      CameraToRR();
      ~CameraToRR();

      // Decide if it's worthwhile respecting "private data" for values
      void updateAngles(SensorValues values);
      SensorValues values;

      RRCoord convertToRR(int16_t i, int16_t j, bool isBall) const;
      RRCoord convertToRR(const Point &p, bool isBall) const;
      Point convertToRRXY(const Point &p) const;
      RANSACLine convertToRRLine(const RANSACLine &l) const;
      Point convertToImageXY(const Point &p) const;

      Pose pose;
      float pixelSeparationToDistance(int pixelSeparation, int realSeparation) const;
      bool isRobotMoving() const;

      /**
       * Finds the saliency scan coordinates where vertical scans
       * should be stopped to avoid considering the robots own body
       * The coordinates in the array returned are image co-ords
       **/
      void findEndScanValues();

      int getTopEndScanCoord(int index) const {
            return topEndScanCoords_[index];
      }
      int getBotEndScanCoord(int index) const {
            return botEndScanCoords_[index];
      }

    private:
      int topEndScanCoords_[IMAGE_COLS];
      int botEndScanCoords_[IMAGE_COLS];
};

#endif
