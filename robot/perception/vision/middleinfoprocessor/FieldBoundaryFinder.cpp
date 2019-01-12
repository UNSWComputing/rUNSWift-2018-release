#include "FieldBoundaryFinder.hpp"

#include <limits>

#include "perception/vision/other/Ransac.hpp"
#include "perception/vision/VisionDefinitions.hpp"

#include "utils/Logger.hpp"
#include "utils/LeastSquaresLine.hpp"
#include "utils/basic_maths.hpp"
#include "utils/angles.hpp"
#include "utils/SPLDefs.hpp"

// 30 Degrees -> Radians
#define MIN_ANGLE_BETWEEN_BOUNDARY_LINES 0.523599

using namespace std;

unsigned int seed = 42;
const int VERTICAL_OFFSET = 50;

const int FieldBoundaryFinder::consecutive_green = 2;

FieldBoundaryFinder::FieldBoundaryFinder() {
   // reserve space for the maximum number of field boundary points we may have
   // note, this is a lot faster than letting the vector resize itself
   boundaryPointsTop.reserve(TOP_SALIENCY_COLS);
   boundaryPointsBot.reserve(BOT_SALIENCY_COLS);
}

void FieldBoundaryFinder::find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
   const Fovea* topFovea;
   const Fovea* botFovea;

   // Assume we have two full regions
   // Rather than assume an ordering, check if first is top
   if (info_middle.full_regions[0].isTopCamera()) {
      topFovea = info_middle.full_regions[0].getInternalFovea();
      botFovea = info_middle.full_regions[1].getInternalFovea();
   }
   else {
      topFovea = info_middle.full_regions[1].getInternalFovea();
      botFovea = info_middle.full_regions[0].getInternalFovea();
   }

   // Reset Variables
   boundaryPointsBot.clear();
   boundaryPointsTop.clear();
   fieldBoundaries.clear ();

   // Run on top fovea
   fieldBoundaryPoints(info_in, *topFovea, true);
   fieldBoundaryLines(&seed, &info_in.cameraToRR, true);

   findStartScanCoords(info_in, info_out, *topFovea);

   // Repeat on bottom fovea
   fieldBoundaryPoints(info_in, *botFovea, false);
   fieldBoundaryLines(&seed, &info_in.cameraToRR, false);

   findStartScanCoords(info_in, info_out, *botFovea);

   info_out.boundaries = fieldBoundaries;
}

/**
 * Boundary Point Detection
 *
 * 1. Divide frame into two sections: top and bottom.
 *    top is defined as the projected width of a field line
 *    at the top of the image using camera to RR
 *
 * 2. Detect boundaries
 * psuedo code for bottom of image
 * for each pixel in column starting from top {
 *    if (`consecutive_green' pixels below are green) {
 *       add point
 *    }
 *    if (pixels below contain `consecutive_green' green pixels
 *     && contain non-consecutive white or unclassified pixels)
 *    {
 *       add point
 *    }
 *    if (current pixel is white and either of above conditions are true) {
 *       add point
 *    }
 * }
 *
 * psuedo code for top of image
 * for each pixel in column starting from top {
 *    if (so far only white and unclassified pixels have been seen) {
 *       continue
 *    }
 *
 *    as above
 *
 * }
 */
void FieldBoundaryFinder::fieldBoundaryPoints(
      const VisionInfoIn &info_in,
      const Fovea &fovea,
      bool top)
{

   /* Old parameter hax */
   const std::pair<int, int> &horizon = info_in.pose.getHorizon();
   /* End hax */

   std::pair<int, int> horizon_adj = horizon;
   horizon_adj.first += VERTICAL_OFFSET;
   horizon_adj.second += VERTICAL_OFFSET;

   const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   const int ROWS = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

   const Pose &pose = info_in.pose;

   float gradient = (horizon_adj.second - horizon_adj.first) / float(COLS) / fovea.getDensity();

   int intercept = horizon_adj.first / fovea.getDensity();

   int i, j, start;
   int colour_j;
   int image_end, fovea_end;
   int horizon_ave;
   int fieldline_width_at_top;
   int green_count, white_count, overshoot;
   bool possible_fieldline;


   /* Calculate width of field line if it were to be present at the top of
    * the field.
    */
   Point topRR, topImage, topFovea;

   horizon_ave = (horizon_adj.first + horizon_adj.second) / 2 / fovea.getDensity();
   if (horizon_ave < 0) {
      horizon_ave = 0;
   }

   /* Only calculate fieldline width if the horizon is not at the
    * top of the image.
    */
   if (horizon_ave == 0) {
//      if (top) {
         topRR = pose.imageToRobotXY(Point(COLS/ 2, horizon_ave));
//      } else {
//         topRR = pose.imageToRobotXY(
//            Point(IMAGE_WIDTH / 2, horizon_ave + IMAGE_ROWS));
//      }

      static const float width_tol = 1.5;
      topRR.x() -= (FIELD_LINE_WIDTH * width_tol);
      if (topRR.x() < 0) {
         fieldline_width_at_top = 0;
      } else {
         topImage = pose.robotToImageXY(topRR);
         topImage.y();
         if (topImage.y() > ROWS) {
            topImage.y() = ROWS;
         }
         topFovea = fovea.mapImageToFovea (topImage);
         fieldline_width_at_top = topFovea.y();
      }
   } else {
      fieldline_width_at_top = 0;
   }

   int cols = TOP_SALIENCY_COLS;
   if (!top) cols = BOT_SALIENCY_COLS;
   for (i = 0; i < cols; ++i) {
      if (i != 0) {
         greenTops[i] = greenTops[i - 1];
      } else {
         greenTops[0] = 0;
      }

      float horizonIntercept = gradient * i + intercept;
      if (!top) horizonIntercept -= ROWS / fovea.getDensity();
      start = std::min(std::max(0.0f, horizonIntercept), (float)(ROWS / fovea.getDensity())); //start of the scan must not got below the image

      if (top) {
         image_end = info_in.cameraToRR.getTopEndScanCoord(
                     fovea.mapFoveaToImage(Point(i, 0)).x());
      } else {
         image_end = info_in.cameraToRR.getBotEndScanCoord(
                     fovea.mapFoveaToImage(Point(i, 0)).x());
      }

      fovea_end = fovea.mapImageToFovea(Point(0, image_end)).y();

      white_count = 0;
      overshoot   = 0;

      j = start;
      colour_j = j;

      /* Test for fieldline at top of image */
      possible_fieldline = false;
      for (; j < fieldline_width_at_top; ++ j, ++ colour_j) {
         if (fovea.getFoveaColour(i, colour_j) == cWHITE) {
            possible_fieldline = true;
            break;
         }
      }

      /* Now search for the field boundary */
      green_count =  0;
      white_count =  0;
      overshoot   = -1;

      colour_j -= (j - start);
      j  = start;

      // NOTE: REMOVE THIS AFTER BRAZIL, THIS IS A SPECIAL CASE FOR THE NICE BLACK BORDER WE GET
      //bool seenBlack = false;
      for (; j < fovea_end; ++ j, ++ colour_j) {
         Colour c = fovea.getFoveaColour(i, colour_j);
         //if (*c == cBLACK) seenBlack = true;
         //if (!seenBlack) continue;

         if (c == cGREEN) {
            ++ green_count;
            ++ overshoot;
            white_count = 0;

            if (green_count == consecutive_green) {
               j -= overshoot;

               /* Check that we didn't detect a fieldline */
               if (possible_fieldline && j < start + fieldline_width_at_top) {
                  break;
               }
               if (j != 0) {
                  if (top) {
                     boundaryPointsTop.push_back(Point(i, j) * fovea.getDensity());
                  } else {
                     Point p = Point(i, j) * fovea.getDensity();
                     p.y() += ROWS;
                     boundaryPointsBot.push_back(p);
                  }
               } else {
                  ++ greenTops[i];
               }

               break;
            }
         } else if ((c == cWHITE || c == cBACKGROUND)
                 && (white_count < 1 && green_count != 0))
         {
            /* Allow for white between two green pixels */
            ++ overshoot;
            ++ white_count;
         } else if (c == cWHITE && green_count == 0) {
            /* first pixel can be white */
            overshoot   = 0;
            white_count = 1;
         } else {
            green_count =  0;
            white_count =  0;
            overshoot   = -1;
         }
      }
   }
}

void FieldBoundaryFinder::fieldBoundaryLines(unsigned int *seed,
                                        const CameraToRR *convRR,
                                        bool top)
{

   // Set up some constants based on the camera
   const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   const int ROWS = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
   int density = (top) ? TOP_SALIENCY_DENSITY : BOT_SALIENCY_DENSITY;
   int cols = (top) ? TOP_SALIENCY_COLS : BOT_SALIENCY_COLS;

   // Modified RANSAC parameters for better field boundaries
   // original k = 40, e = 8, n = 140/density
   // For first line, I doubled e
   // turns out, that's all that's necessary

   /* some magic constants (see RANSAC documentation) */
   static const unsigned int k = 50;
   static const float        e = 8.0;
   static const unsigned int n = (140 / density);

   /* If there are n or greater green tops, which usually fall below
    * the field boundary, above a field line, invalidate it
    */
   static const int invalidateIfNTops  = 40  / density;

   unsigned int i;

   std::vector<Point> *boundaryPoints;
   if(top) boundaryPoints = &boundaryPointsTop;
   else boundaryPoints = &boundaryPointsBot;

   std::vector<bool> *cons, consBuf[2];
   consBuf[0].resize(cols * 2);
   consBuf[1].resize(cols * 2);

   std::vector<RANSACLine> lines(1);
   RANSACLine result;

   RANSAC::Generator<RANSACLine> g;
   RANSAC::Ransac<RANSACLine> ransac;

   if (ransac(g, *boundaryPoints, &cons, lines[0], k, 2*e, n, consBuf, seed)) {

      /**
       * Line found, now try find a second line.
       * First remove points belonging to the first line
       * Then run RANSAC again
       */
      std::vector<Point> boundaryPoints2;
      for (i = 0; i < boundaryPoints->size(); ++i) {
         if (! (*cons)[i]) {
            boundaryPoints2.push_back(boundaryPoints->at(i));
         }
      }

      lines.resize(2);
      if (! ransac(g, boundaryPoints2, &cons, lines[1], k, e, n, consBuf, seed)) {
         lines.resize(1);
      }

   } else {
      lines.resize(0);
   }

   /* Invalidate any line with too much green above it */
   std::vector<RANSACLine>::iterator line;
   for (line = lines.begin(); line != lines.end(); ++ line) {
      int n;
      if (line->t1 == 0) {
         n = greenTops[cols - 1];
      } else {
         /* Solve for x when y = SALIENCY_DENSITY. At y = 0 aliased green
          * pixels from a legitimate field boundary can be along the top of the
          * image
          *
          * t1x + t2y + t3 = 0
          * x = -(t3 + t2*SALIENCY_DENSITY) / t1
          */
         int x;
         if (top) {
            x = -(line->t3 + line->t2 * density) / line->t1;
         } else {
            x = -(line->t3 + line->t2*(density+ROWS)) /line->t1;
         }

         /* Convert to saliency image resolution */
         x /= density;

         if (x < 0 || x >= cols) {
            n = greenTops[cols - 1];
         } else {
            bool positiveSlope = (line->t1 >= 0) == (line->t2 >= 0);
            if (positiveSlope) {
               n = greenTops[x];
            } else {
               n = greenTops[cols - 1] - greenTops[x];
            }
         }
      }
      if (n >= invalidateIfNTops) {
         line = lines.erase(line) - 1;
      }
   }

   if (lines.size() == 2) {
      /**
       * begin sanity checks, sort lines by gradient
       */
      RANSACLine *l1;
      RANSACLine *l2;

      if (abs(lines[0].t2 * lines[1].t1) < abs(lines[1].t2 * lines[0].t1)) {
         l1 = &lines[0];
         l2 = &lines[1];
      } else {
         l1 = &lines[1];
         l2 = &lines[0];
      }

      /**
       * sanity check 1: avoid lines that intersect outside the image
       */
      llog(DEBUG2) << "sanity check 1: intersect inside frame" << endl;
      float x_intercept = (l1->t3*l2->t2 - l2->t3*l1->t2) /
                          (float)(l1->t2*l2->t1 - l1->t1*l2->t2);
      float y_intercept = (l1->t3*l2->t1 - l2->t3*l1->t1)/
                          (float)(l1->t1*l2->t2 - l2->t1*l1->t2);

      if (x_intercept < 0 || x_intercept > COLS
       || y_intercept < 0 || y_intercept > (2*ROWS))
      {
         lines.pop_back();
      }

      if (lines.size() == 2) {
         /**
          * sanity check 2: avoid lines that are close to being parallel
          */
         llog(DEBUG2) << "sanity check 2: angle between lines" << endl;
         RANSACLine rrl1 = convRR->convertToRRLine(*l1);
         RANSACLine rrl2 = convRR->convertToRRLine(*l2);
         float m1 = -rrl1.t1 / (float) rrl1.t2;
         float m2 = -rrl2.t1/ (float) rrl2.t2;
         float theta = atan(abs((m1 - m2) / (1 + m1 * m2)));
         // float deviation = M_PI / 2 - theta;
         // if (deviation > MAX_BOUNDARY_DEVIATION_FROM_PERPENDICULAR)
         if (theta < MIN_ANGLE_BETWEEN_BOUNDARY_LINES)
         {
            lines.pop_back();
            llog(DEBUG2) << "Failed sanity check 2" << std::endl;
            llog(DEBUG2) << "m1: " << m1 << ", m2: " << m2 << ", theta: " << RAD2DEG(theta) << std::endl;
         }
      }
   }

   for (i = 0; i < lines.size(); ++ i) {
      RRCoord r1, r2;

      r1 = convRR->convertToRR(lines[i].p1, false);
      r2 = convRR->convertToRR(lines[i].p2, false);

      int32_t x1 = r1.distance() * cos(r1.heading());
      int32_t y1 = r1.distance() * sin(r1.heading());
      int32_t x2 = r2.distance() * cos(r2.heading());
      int32_t y2 = r2.distance() * sin(r2.heading());

      // Find perpendicular distance of this line from the origin
      float dist = abs((x2-x1)*y1 - (y2-y1)*x1)/
         sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

      float var = SQUARE(1000) + dist + lines[i].var * 1000;
      RANSACLine rrLine(Point(x1, y1), Point(x2, y2), var);
      //std::cout << lines[i].p1 << std::endl << lines[i].p2 << std::endl;
      fieldBoundaries.push_back(FieldBoundaryInfo(rrLine, lines[i]));
   }
}

void FieldBoundaryFinder::findStartScanCoords(const VisionInfoIn &info_in,
    VisionInfoOut &info_out, const Fovea &fovea) {

   const int COLS = (fovea.getTop()) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   const int ROWS = (fovea.getTop()) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
   int cols = (fovea.getTop()) ? TOP_SALIENCY_COLS : BOT_SALIENCY_COLS;
   const std::pair<int, int> &horizon = info_in.pose.getHorizon();
   float gradient = (horizon.second - horizon.first) / float(COLS);
   int intercept = horizon.first / fovea.getDensity();

   uint16_t i;
   int32_t intersec;

   /* Calculate the number of field boundaries in this image */
   int numBoundaries = 0;
   for (vector<FieldBoundaryInfo>::const_iterator it = fieldBoundaries.begin();
        it != fieldBoundaries.end(); ++ it) {
      if ((*it).imageBoundary.p1.y() < ROWS && fovea.getTop()) {
         numBoundaries++;
      } else if ((*it).imageBoundary.p1.y() > ROWS && !fovea.getTop()) {
         numBoundaries++;
      }
   }

   /* If there are no lines, work out if the entire image consists of
    * the field or the entire image is of the background.
    */
   if (numBoundaries == 0) {
      int numGreen = 0;
      for (int x = 0; x < cols; x++) {
         if (fovea.getFoveaColour(x,0) == cGREEN) {
            numGreen++;
         }
      }
      if (numGreen < MIN_GREEN_THRESHOLD) {
         for (int x = 0; x < COLS; x++) {
            // There is no field seen, so give all the array values that
            // far exceed the number of rows in the saliency scan so that
            // no scan lines are used
            if (fovea.getTop()) {
               info_out.topStartScanCoords[x] = ROWS;
            } else {
               info_out.botStartScanCoords[x] = ROWS*2;
            }
         }
      } else {
         if (fovea.getTop()) {
            for (int x = 0; x < COLS; x++) {
               int horizonIntercept = gradient * x + intercept;
               info_out.topStartScanCoords[x] = std::max(0, horizonIntercept);
            }
         } else {
            for (int x = 0; x < COLS; x++) {
               info_out.botStartScanCoords[x] = ROWS;
            }
         }
      }
   } else {
      for (int x = 0; x < COLS; x++) {
         int maxVal = -1;
         for (i = 0; i < fieldBoundaries.size(); ++ i) {
            const RANSACLine &line = fieldBoundaries[i].imageBoundary;
            if (line.p1.y() < ROWS && fovea.getTop()) {
               intersec = (-line.t3 - (x * line.t1))/line.t2;
               if (intersec > maxVal) {
                  maxVal = intersec;
               }
            } else if (line.p1.y() >= ROWS && !fovea.getTop()) {
               intersec = (-line.t3 - (x * line.t1))/line.t2;
               if (intersec > maxVal) {
                  maxVal = intersec;
               }
            }
         }
         if (maxVal < 0) {
            maxVal = 0;
         }
         if (fovea.getTop()) {
            int horizonIntercept = gradient * x + intercept;
            info_out.topStartScanCoords[x] = std::max(std::max(0, std::min(ROWS, maxVal)), horizonIntercept);
         } else {
            info_out.botStartScanCoords[x] =
                  std::max(ROWS, std::min(ROWS*2, maxVal));
         }
      }
   }
}

void FieldBoundaryFinder::lsRefineLine(
      RANSACLine               &line,
      const std::vector<Point> &points,
      const std::vector<bool>  &cons)
{
   LeastSquaresLine ls;
   std::vector<Point>::const_iterator p = points.begin();
   std::vector<bool> ::const_iterator c = cons.begin();

   for (; p != points.end(); ++ p, ++ c) {
      if (*c) {
         ls.addPoint(*p);
      }
   }

   ls.getLineABC(&line.t1, &line.t2, &line.t3);
}
