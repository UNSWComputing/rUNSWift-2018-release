#ifndef PERCEPTION_VISION_MIDDLEINFOPROCESSOR_FIELD_BOUNDARY_FINDER
#define PERCEPTION_VISION_MIDDLEINFOPROCESSOR_FIELD_BOUNDARY_FINDER

#include <vector>
#include <utility>

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/camera/CameraToRR.hpp"
#include "perception/vision/Fovea.hpp"

#include "types/VisionInfoIn.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/Point.hpp"

#include "MiddleInfoProcessorInterface.hpp"

class FieldBoundaryFinder : public MiddleInfoProcessor
{
   public:
      /**
       * Points to be ransaced
      **/
      std::vector<Point> boundaryPointsTop;
      std::vector<Point> boundaryPointsBot;

      /**
       * The coordinates of the top of the field in the image
       * The coordinates are given in image coordinates
       **/
      int topStartScanCoords[IMAGE_COLS];
      int botStartScanCoords[IMAGE_COLS];

      void find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out);

      /**
       * Find coordinates of points that may be at the boundary
       * of the field by using the saliency scan
       * @param frame      Current vision frame
       * @param fovea      Current fovea to be searched
       **/
      void fieldBoundaryPoints(const VisionInfoIn &info_in,
                           const Fovea &fovea,
                           bool top);

      /**
       * Find up to two lines formed by field boundary points
       * using the RANSAC algorithm
       **/
      void fieldBoundaryLines(unsigned int *seed,
                          const CameraToRR *convRR,
                          bool top);

      /**
       * Fills the startScanCoords array to find the coordinates
       * in the saliency scan where the field starts
       **/
      void findStartScanCoords(const VisionInfoIn &info_in,
                               VisionInfoOut &info_out,
                               const Fovea &fovea);

      explicit FieldBoundaryFinder();

      std::vector<FieldBoundaryInfo> fieldBoundaries;

   private:
      static const int consecutive_green;

      void lsRefineLine(
            RANSACLine               &line,
            const std::vector<Point> &points,
            const std::vector<bool>  &cons);

      /**
       * A cummulative count of green pixels occuring
       * at the top of the image
       */
      int greenTops[TOP_SALIENCY_COLS];
      int totalGreens;
};

#endif
