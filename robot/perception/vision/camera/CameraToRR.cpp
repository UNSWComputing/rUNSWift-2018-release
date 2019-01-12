#include "perception/vision/camera/CameraToRR.hpp"

#include "utils/SPLDefs.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"

using namespace std;

CameraToRR::CameraToRR()
{
   for (int i = 0; i < IMAGE_COLS; i++) {
      topEndScanCoords_[i] = TOP_IMAGE_ROWS;
      botEndScanCoords_[i] = BOT_IMAGE_ROWS;
   }
}

CameraToRR::~CameraToRR()
{
}

void CameraToRR::updateAngles(SensorValues val)
{
   values = val;
}

RRCoord CameraToRR::convertToRR(const Point &p, bool isBall) const
{
   return convertToRR(p.x (), p.y (), isBall);
}

RRCoord CameraToRR::convertToRR(int16_t i, int16_t j, bool isBall) const
{
   RRCoord myloc = pose.imageToRobotRelative(i, j, isBall ? 35 : 0);
   return myloc;
}

Point CameraToRR::convertToRRXY(const Point &p) const
{
   Point myloc = pose.imageToRobotXY(p, 0);
   return myloc;
}

RANSACLine CameraToRR::convertToRRLine(const RANSACLine &l) const
{
   return RANSACLine(convertToRRXY(l.p1), convertToRRXY(l.p2));
}

Point CameraToRR::convertToImageXY(const Point &p) const
{
   Point myloc = pose.robotToImageXY(p, 0);
   return myloc;
}

// TODO: Sean make this not just use top pixel size
float CameraToRR::pixelSeparationToDistance(int pixelSeparation,
      int realSeparation) const
{
   return (FOCAL_LENGTH * realSeparation) /
      (TOP_PIXEL_SIZE * pixelSeparation);
}

bool CameraToRR::isRobotMoving() const
{
   return true;
}

void CameraToRR::findEndScanValues() {
   const int exclusionRes = Pose::EXCLUSION_RESOLUTION;
   const int16_t *topPoints = pose.getTopExclusionArray();
   const int16_t *botPoints = pose.getBotExclusionArray();
   for (int i = 0; i < TOP_IMAGE_COLS; ++i) {
      if (topPoints[(i * exclusionRes)/TOP_IMAGE_COLS] < TOP_IMAGE_ROWS) {
         topEndScanCoords_[i] = topPoints[(i * exclusionRes)/TOP_IMAGE_COLS];
      } else {
         topEndScanCoords_[i] = TOP_IMAGE_ROWS;
      }
   }
   for (int i = 0; i < BOT_IMAGE_COLS; ++i) {
      if (botPoints[(i * exclusionRes)/BOT_IMAGE_COLS] < BOT_IMAGE_ROWS) {
         botEndScanCoords_[i] = botPoints[(i * exclusionRes)/BOT_IMAGE_COLS];
      } else {
         botEndScanCoords_[i] = BOT_IMAGE_ROWS; 
      }
   }
}
