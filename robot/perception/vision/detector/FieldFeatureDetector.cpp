#include "perception/vision/detector/FieldFeatureDetector.hpp"

#include "utils/Logger.hpp"
#include "utils/Timer.hpp"
#include "utils/SPLDefs.hpp"

#include <cmath>

#define MIN_LINE_WIDTH                     (0*0)
#define MAX_LINE_WIDTH                     (75*75)
#define MIN_LINE_LENGTH                    (400*400)
#define SHORT_LINE_LENGTH                  (300*300)
#define PERPENDICULAR_THRESHOLD DEG2RAD    (10)
#define EXCLUSION_LENGTH                   (6)
#define FOVEA_EDGE_THRESHOLD_BASE          75
#define MIN_POINT_DISTANCE                 (120*120)
#define ALLOWED_POINT_PROXIMITY 16/TOP_SALIENCY_DENSITY
#define PARALLEL_LINE_THRESHOLD DEG2RAD    (10)
#define PARALLEL_LINE_DISTANCE_THRESHOLD   (200)
#define MIN_PARALLEL_LENGTH                (500*500)

#define BOTH    0
#define LINES   1
#define CIRCLES 2

//#define DEBUG_FIELD_FEATURE_ASCII_ART
// #define DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

FieldFeatureDetector::FieldFeatureDetector() {
    // Setup landmarks
    // Centre circle
    landmarks_.push_back(Point(-750, 0));   // Centre Circle
    landmarks_.push_back(Point(750, 0));    // Centre Circle
    num_circle_landmarks_ = 2;

    // Our half
    landmarks_.push_back(Point(-4500, 1100));     // Goalbox T-int
    landmarks_.push_back(Point(-4500, -1100));    // Goalbox T-int
    landmarks_.push_back(Point(-3900, 1100));     // Goalbox corner
    landmarks_.push_back(Point(-3900, -1100));    // Goalbox corner
    landmarks_.push_back(Point(-4500, 2000));     // Field corner
    landmarks_.push_back(Point(-4500, -2000));    // Field corner

    // Their half
    landmarks_.push_back(Point(4500, 1100));      // Goalbox T-int
    landmarks_.push_back(Point(4500, -1100));     // Goalbox T-int
    landmarks_.push_back(Point(3900, 1100));      // Goalbox corner
    landmarks_.push_back(Point(3900, -1100));     // Goalbox corner
    landmarks_.push_back(Point(4500, 2000));      // Field corner
    landmarks_.push_back(Point(4500, -2000));     // Field corner

    // Halfway
    landmarks_.push_back(Point(0, 2000));         // Halfway T-int
    landmarks_.push_back(Point(0, -2000));        // Halfway T-int
}

void FieldFeatureDetector::detect(const VisionInfoIn& info_in, const VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

    // the seed set is for RANSAC, as we are not doing cryptography this does not need to be random
    unsigned int seed = 42;
    Timer t;
    convertRR_ = &info_in.cameraToRR;

    const std::vector<RegionI>& regions = info_middle.full_regions;

    // reset all found points and features for a new frame
    reset(true);

    t.restart();
    for(unsigned int i=0; i < regions.size(); ++i)
    {
        findPointsInRegion(info_out, regions[i]);
    }
    FieldFeatureInfo f = FieldFeatureInfo(RRCoord(0,0), field_line_points_);
    field_features_.push_back(f);

    llog(VERBOSE) << "Finding field feature candidate points took " <<
                                           t.elapsed_us() << " us" << std::endl;

    // TODO make this work with current region points found
    t.restart();

    int type = BOTH;

    // If we are the goalie, don't detect circles
    if (info_in.my_player_number == 1) {
        type = LINES;
    }

    findFeatures(info_out, &seed, type);
    field_features_.insert(field_features_.end(),
                         temp_features_.begin(),
                         temp_features_.end());
    llog(VERBOSE) << "Finding basic field features took " << t.elapsed_us() <<
                                                             " us" << std::endl;

    // Count the number of lines and circles we've found so far
    t.restart();
    std::vector<FieldFeatureInfo>::const_iterator it;
    int num_circles = 0;
    int num_lines = 0;
    for (it = field_features_.begin(); it != field_features_.end(); ++it) {
        FieldFeatureInfo::Type type = it->type;
        if (type == FieldFeatureInfo::fCentreCircle) {
           ++num_circles;
        } else if (type == FieldFeatureInfo::fLine) {
           ++num_lines;
        }
    }
    llog(VERBOSE) << "Counting lines and circles took " << t.elapsed_us();
    llog(VERBOSE) << " us" << std::endl;

    // Look for parallel lines if we haven't seen the centre circle, otherwise do circle orientation if we've seen 1 circle and 1 line.
    // VWong readded parallel line detection that was once removed because if we saw the centre circle as parallel lines, localisation would perform poorly
    t.restart();
    if (num_circles == 0) {
        //findParallelLines(&field_features_);
        //llog(VERBOSE) << "Looking for parallel lines took " << t.elapsed_us();
        //llog(VERBOSE) << " us" << std::endl;
    } else if (num_circles == 1 && num_lines == 1) {
        circleOrientation(&field_features_);
        llog(VERBOSE) << "Circle orientation took " << t.elapsed_us();
        llog(VERBOSE) << " us" << std::endl;
    }

    // Find penalty spot if player is goalie
    t.restart();
    // TODO get player number if we still need it
    // if(playerNum == 1){
    //     // findPenaltySpot(info_out, topFovea);
    // }
    llog(VERBOSE) << "Penalty spot detection took " << t.elapsed_us();
    llog(VERBOSE) << " us" << std::endl;

    // For localisation, since FieldLinePointInfo's are found on the ball remove all of these
    // TODO get player number if we still need it
    // if(playerNum == 1){
    //     std::vector<FieldFeatureInfo>::iterator it;
    //     for (it = field_features_.begin(); it != field_features_.end(); ) {
    //         if (it->type == FieldFeatureInfo::fFieldLinePoint) {
    //             it = field_features_.erase(it);
    //         } else {
    //             it ++;
    //         }
    //     }
    // }

   // TODO high res feature finding if necessary
   // TODO use previous found feautures if possible given that Colour ROI already reduces space searched
   info_out.features = field_features_;
}

// TODO: check if info_out is actually used
void FieldFeatureDetector::findPointsInRegion(VisionInfoOut& info_out, const RegionI& region)
{
    // int field_top;
    const int cols = region.getCols();
    const int rows = region.getRows();

    // The left point.
    Point left = Point(-1, -1);

    // The right point.
    Point right = Point(-1, -1);

#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
    int candidates=0;
    int colourRejections=0;
    int distanceRejections=0;
    int proximityRejections=0;
    int closeRejections=0;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

    // The top and bottom point for each row/column.
    std::vector<Point> tops(cols, Point(-1, -1));
    std::vector<Point> bots(cols, Point(-1, -1));

    // The most recent x and y locations at which a point was added. Prevents
    // adding too many points in roughly the same place.
    std::vector<int> recentColY(cols, -1000);
    std::vector<int> recentRowX(rows, -1000);

    // The scan start coordinates.
    int col_starts[TOP_SALIENCY_ROWS];
    int endPoint = TOP_IMAGE_ROWS*region.isTopCamera() +
                                         BOT_IMAGE_ROWS*(!region.isTopCamera());
    for(int x_raw=0, x=0; x_raw<endPoint; x_raw+=region.getDensity(), ++x)
    {
        if(region.isTopCamera())
            col_starts[x] = info_out.topStartScanCoords[x_raw];
        else
            col_starts[x] = info_out.botStartScanCoords[x_raw];
    }

    // Reset the point list.
    field_line_points_temp_.clear();

    // Horizontal Scan
    int curr_col = 0;
    int curr_row = 0;
    int curr_row_raw = TOP_IMAGE_ROWS*(!region.isTopCamera());
    RegionI::iterator_fovea end = region.end_fovea();
    for(RegionI::iterator_fovea it = region.begin_fovea(); it<end; ++it,
                                                                     ++curr_col)
    {
        // The directional edge magnitudes of the pixel.
        // const Point dxdy = it.edge();

        // Keep track of the current coordinates.
   	    if(curr_col == cols)
        {
            curr_col = 0;
            left[0] = -1;
            left[1] = -1;
            right[0] = -1;
            right[1] = -1;
            ++curr_row;
            curr_row_raw += region.getDensity();

#ifdef DEBUG_FIELD_FEATURE_ASCII_ART
            if(region.isTopCamera() && curr_row%4 == 0)
                std::cout << std::endl;
#endif // DEBUG_FIELD_FEATURE_ASCII_ART
   	    }

        // Check if this pixel is below the field boundary.
        // TURNED OFF AS THE VALUES GIVEN ARE ALL BELOW THE BOTTOM.

#ifdef DEBUG_FIELD_FEATURE_ASCII_ART
        if(region.isTopCamera() && curr_col%4 == 0 && curr_row%4 == 0)
        {
            if(dxdy.x() > FOVEA_EDGE_THRESHOLD_BASE && dxdy.y() >
                                                      FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "B ";
            else if(dxdy.x() > FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "H ";
            else if(dxdy.y() > FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "V ";
            else if(dxdy.x() < -FOVEA_EDGE_THRESHOLD_BASE && dxdy.y() <
                                                     -FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "b ";
            else if(dxdy.x() < -FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "h ";
            else if(dxdy.y() < -FOVEA_EDGE_THRESHOLD_BASE)
                std::cout << "v ";
            else
                std::cout << "- ";
        }
#endif // DEBUG_FIELD_FEATURE_ASCII_ART

        //std::cout << "field_top: " << field_top << std::endl;
   		if(curr_row_raw < col_starts[curr_col])
   			continue;

        // // The left edge should be a strong dark to light change.
   		// if(dxdy.x() < -FOVEA_EDGE_THRESHOLD_BASE)
        // {
        //     left[0] = curr_col;
        //     left[1] = curr_row;
        // }

        // // The right edge should be a strong light to dark change.
        // else if(dxdy.x() > FOVEA_EDGE_THRESHOLD_BASE && left.x() != -1)
        // {
        //     right[0] = curr_col;
        //     right[1] = curr_row;
        // }

        // // The top edge should be a strong dark to light change.
   		// if(dxdy.y() < -FOVEA_EDGE_THRESHOLD_BASE)
        // {
        //     tops[curr_col][0] = curr_col;
        //     tops[curr_col][1] = curr_row;
        // }

        // // The bottom edge should be a strong light to dark change.
        // else if(dxdy.y() > FOVEA_EDGE_THRESHOLD_BASE &&
        //                                                tops[curr_col].x() != -1)
        // {
        //     bots[curr_col][0] = curr_col;
        //     bots[curr_col][1] = curr_row;
        // }

        // If this is not a strong edge, and we have both left and right edges,
        // it may be worth adding as a feature point.
        if(right.x() != -1)
        {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
            ++candidates;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

            // Get the coordinates of the new feature point.
   			int x = (left.x() + right.x()) / 2;
   			int y = (left.y() + right.y()) / 2;

            // The distance between points.
            int dist;

            // Sanity checks
            //*
            // Check colour of pixel in image.
   			Colour c = region.getPixelColour(x, y);
   			if(c != cWHITE)
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++colourRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

                left[0] = -1;
                left[1] = -1;
                right[0] = -1;
                right[1] = -1;
                continue;
            }
            //*/
            //*
            // Check if this point is far enough from nearby points.
            if(!(x - recentRowX[y] > ALLOWED_POINT_PROXIMITY) ||
                                 !(y - recentColY[x] > ALLOWED_POINT_PROXIMITY))
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++proximityRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                left[0] = -1;
                left[1] = -1;
                right[0] = -1;
                right[1] = -1;
                continue;
            }

            // Update the recent points.
            recentRowX[y] = x;
            recentColY[x] = y;

			// Check distance between left and right point
   			Point imageLeft = left * region.getDensity();
   			Point imageRight = right * region.getDensity();
            imageLeft.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;
            imageRight.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;

            Point leftRR = convertRR_->pose.imageToRobotXY(imageLeft);
   			Point rightRR = convertRR_->pose.imageToRobotXY(imageRight);
   			dist = DISTANCE_SQR(leftRR.x(), leftRR.y(), rightRR.x(),
                                                                   rightRR.y());
   			if((dist < MIN_LINE_WIDTH) || (dist > MAX_LINE_WIDTH))
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++distanceRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

   				left = right;
	   			right = Point(-1,-1);
	   			continue;
	   		}

	   		Point middle = Point(x, y) * region.getDensity();
            middle.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;
	   		Point p = convertRR_->pose.imageToRobotXY(middle);

            // Check point isn't too close (and likely a nao part).
	   		dist = p.x()*p.x() + p.y()*p.y();
	   		if(dist < MIN_POINT_DISTANCE)
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++closeRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

	   			left[0] = -1;
                left[1] = -1;
                right[0] = -1;
                right[1] = -1;
	   			continue;
	   		}

            // Check point isn't outside the field.
            if(p.x() < -FIELD_LENGTH/2 || p.x() > FIELD_LENGTH/2 ||
                                p.y() < -FIELD_WIDTH/2 || p.y() > FIELD_WIDTH/2)
            {
                left[0] = -1;
                left[1] = -1;
                right[0] = -1;
                right[1] = -1;
                continue;
            }

	   		FieldLinePointInfo f = FieldLinePointInfo(middle, p);
	   		field_line_points_temp_.push_back(f);

            // Reset search.
            left[0] = -1;
            left[1] = -1;
            right[0] = -1;
            right[1] = -1;
	   	}
        if(bots[curr_col].x() != -1)
        {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
            ++candidates;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

            // Get the coordinates of the new feature point.
            int x = (tops[curr_col].x() + bots[curr_col].x()) / 2;
            int y = (tops[curr_col].y() + bots[curr_col].y()) / 2;

            // The distance between points.
            int dist;

            // Sanity checks
            //*
            // Check colour of pixel in image.
            Colour c = region.getPixelColour(x, y);
            if(c != cWHITE)
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++colourRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

                tops[curr_col][0] = -1;
                tops[curr_col][1] = -1;
                bots[curr_col][0] = -1;
                bots[curr_col][1] = -1;
                continue;
            }
            //*/
            //*
            // Check if this point is far enough from nearby points.
            if(!(x - recentRowX[y] > ALLOWED_POINT_PROXIMITY) ||
                                 !(y - recentColY[x] > ALLOWED_POINT_PROXIMITY))
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++proximityRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                tops[curr_col][0] = -1;
                tops[curr_col][1] = -1;
                bots[curr_col][0] = -1;
                bots[curr_col][1] = -1;
                continue;
            }

            // Update the recent points.
            recentRowX[y] = x;
            recentColY[x] = y;

            // Check distance between top and bottom point
            Point imageTop = tops[curr_col] * region.getDensity();
            Point imageBot = bots[curr_col] * region.getDensity();
            imageTop.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;
            imageBot.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;

            Point topRR = convertRR_->pose.imageToRobotXY(imageTop);
            Point botRR = convertRR_->pose.imageToRobotXY(imageBot);
            dist = DISTANCE_SQR(topRR.x(), topRR.y(), botRR.x(), botRR.y());
            if((dist < MIN_LINE_WIDTH) || (dist > MAX_LINE_WIDTH))
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++distanceRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

                tops[curr_col] = right;
                bots[curr_col] = Point(-1,-1);
                continue;
            }

            Point middle = Point(x, y) * region.getDensity();
            middle.y() += (!region.isTopCamera()) * TOP_IMAGE_ROWS;
            Point p = convertRR_->pose.imageToRobotXY(middle);

            // Check point isn't too close (and likely a nao part).
            dist = p.x()*p.x() + p.y()*p.y();
            if(dist < MIN_POINT_DISTANCE)
            {
#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
                ++closeRejections;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS

                tops[curr_col][0] = -1;
                tops[curr_col][1] = -1;
                bots[curr_col][0] = -1;
                bots[curr_col][1] = -1;
                continue;
            }

            // Check point isn't outside the field.
            if(p.x() < -FIELD_LENGTH/2 || p.x() > FIELD_LENGTH/2 ||
                                p.y() < -FIELD_WIDTH/2 || p.y() > FIELD_WIDTH/2)
            {
                tops[curr_col][0] = -1;
                tops[curr_col][1] = -1;
                bots[curr_col][0] = -1;
                bots[curr_col][1] = -1;
                continue;
            }

            FieldLinePointInfo f = FieldLinePointInfo(middle, p);
            field_line_points_temp_.push_back(f);

            // Reset search.
            tops[curr_col][0] = -1;
            tops[curr_col][1] = -1;
            bots[curr_col][0] = -1;
            bots[curr_col][1] = -1;
        }
	}

    /*
    This step culls dense groups of points, with the aim of eliminating robots.
    If a single cell has over a certain threshold in points all points in that
    cell are eliminated. Threshold and box size should be set such that only
    when the whole box contains many points (as with naos) are points
    eliminated.

    The underlying array is a 1D array representing the 2D LengthXWidth array.
    The current equation works as (l0w0, l0w1, ..., l0wn, l1w0, ..., lnwn).
    The equation is broken down:

    Current length position divided by the size of a box (field relative points
    are such that 0 is the field centre, hence the offset:
    ((field_line_points_temp_[point].p.x() + FIELD_LENGTH/2) >> BOX_SIZE_POWERS)

    Offset to the correct width position:
    + ((field_line_points_temp_[point].p.y() + FIELD_WIDTH/2) >>
                                                                BOX_SIZE_POWERS)

    Skip that number of full rows:
     * BOXES_X

    Note that bit shift is used instead of divide for speed.
    */

    // Cull clusters of points in a small area to get rid of robots.
    for(int cluster=0; cluster<BOXES_X*BOXES_Y; ++cluster)
        clusters[cluster] = 0;

    // Throw points into their relevant boxes.
    for(unsigned int point=0; point<field_line_points_temp_.size(); ++point)
    {
        ++clusters[((field_line_points_temp_[point].p.x() + FIELD_LENGTH/2)
            >> BOX_SIZE_POWERS) + ((field_line_points_temp_[point].p.y() +
                                  FIELD_WIDTH/2) >> BOX_SIZE_POWERS) * BOXES_X];
    }

    // Points that are not in large clusters are permitted.
    for(unsigned int point=0; point<field_line_points_temp_.size(); ++point)
    {
        if(clusters[((field_line_points_temp_[point].p.x() + FIELD_LENGTH/2)
                >> BOX_SIZE_POWERS) + ((field_line_points_temp_[point].p.y() +
                FIELD_WIDTH/2) >> BOX_SIZE_POWERS) * BOXES_X] <
                                                            DENSE_POINT_CLUSTER)
            field_line_points_.push_back(field_line_points_temp_[point]);
    }

#ifdef DEBUG_FIELD_FEATURE_ASCII_ART
    std::cout << std::endl;
#endif // DEBUG_FIELD_FEATURE_ASCII_ART

#ifdef DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
    std::cout << "Base candidates: " << candidates << std::endl;
    std::cout << "Colour rejections: " << colourRejections << std::endl;
    std::cout << "Proximity rejections: " << proximityRejections << std::endl;
    std::cout << "Distance rejections: " << distanceRejections << std::endl;
    std::cout << "Close rejections: " << closeRejections << std::endl;
    std::cout << "Field line points found: " << field_line_points_.size() <<
                                                                      std::endl;
#endif // DEBUG_OPTIMISE_FIELD_FEATURE_POINTS
}

void FieldFeatureDetector::findFeatures(VisionInfoOut& info_out, unsigned int *seed, int type){
    field_features_.reserve(MAX_FIELD_FEATURES);
    temp_features_.reserve(MAX_FIELD_FEATURES);
Timer t;
t.restart();
    if (type == BOTH) {
       findFieldLinesAndCircles(field_line_points_, &temp_features_, seed);
    } else if (type == LINES) {
       findFieldLines(field_line_points_, &temp_features_, seed);
    }

    findIntersections(temp_features_, &temp_features_);
    mergeFeatures(temp_features_);
}

void FieldFeatureDetector::findFieldLinesAndCircles(
        const std::vector<FieldLinePointInfo> &points,
        std::vector<FieldFeatureInfo> *lines,
        unsigned int *seed) {
    std::vector<Point> field_points1;
    std::vector<Point> field_points2;
    field_points1.reserve(points.size());
    field_points2.reserve(points.size());

    // RANSAC variables
    uint16_t k = 40;
    float e = 15.0;
    uint16_t n = 15;
    std::vector<bool> *con, consBuf[2];
    consBuf[0].resize(points.size());
    consBuf[1].resize(points.size());
    con = &consBuf[0];
    RANSACLine resultLine = RANSACLine(Point(0,0), Point(0,0));
    RANSACCircle resultCircle = RANSACCircle(PointF(0,0), 0.0);
    bool line = true;

    std::vector<bool> takenPoints;
    takenPoints.reserve(points.size());
    for (std::vector<FieldLinePointInfo>::const_iterator i = points.begin();
          i != points.end(); i++) {
       takenPoints.push_back(false);
       field_points1.push_back(i->rrp);
    }

    std::vector<FieldLinePointInfo> onLinePoints;
    onLinePoints.reserve(points.size());
//std::cout << "finding features" << std::endl;
int totalObj = 0;
    // Use RANSAC to find field lines
    for (uint16_t i = 0; i < 100; i++) {
       consBuf[0].clear();
       consBuf[1].clear();
       resultLine = RANSACLine(Point(0,0), Point(0,0));
       resultCircle = RANSACCircle(PointF(0,0), 0.0);
       if (RANSAC::findLinesAndCircles
             (field_points1, CENTER_CIRCLE_DIAMETER/2, &con, resultLine, resultCircle,
              k, e, n, consBuf, seed)) {
//std::cout << "object found" << std::endl;
++totalObj;
          uint16_t b = 0;
          onLinePoints.clear();
          Point max = resultLine.p1;
          Point min = resultLine.p2;
          bool horizontal = false;
          if (abs(min.x() - max.x()) < 50 && abs(min.y() - max.y()) > 50) {
             horizontal = true;
          }
          if (min.x() == 0 && min.y() == 0 && max.x() == 0 && max.y() == 0) {
             line = false;
          }

          // Loop through and remove all the points on the line/circle
          for (uint16_t a = 0; a < field_points1.size(); a++, b++){
             while (takenPoints[b] == true) b++;
             if (con[0][a] == true) {
                onLinePoints.push_back(points[b]);
                takenPoints[b] = true;

                // Keep track of endpoints
                if (horizontal) {
                   if (field_points1[a].y() < min.y()) {
                      min = field_points1[a];
                   } if (field_points1[a].y() > max.y()) {
                      max = field_points1[a];
                   }
                } else {
                   if (field_points1[a].x() < min.x()) {
                      min = field_points1[a];
                   } if (field_points1[a].x() > max.x()) {
                      max = field_points1[a];
                   }
                }
             } else {
                field_points2.push_back(field_points1[a]);
             }
          }
          if ((resultCircle.var <= -10) &&
             ((resultCircle.var) < resultLine.var)) {

             // Calculate RRCoord to centre and save circle info
             PointF centre = resultCircle.centre;
             float distance = sqrt(centre.x() * centre.x() +
                                   centre.y() * centre.y());
             float heading = atan2 (centre.y(), centre.x());
             RRCoord c = RRCoord(distance,heading);
             CentreCircleInfo l = CentreCircleInfo();
             FieldFeatureInfo f = FieldFeatureInfo(c, l);
             lines->push_back(f);

          } else if (line) {

             // Check line and save info
             LineInfo l (resultLine.p1, resultLine.p2);
             l.p1 = min;
             l.p2 = max;

             // Check the line isn't too short
             int dist2 = DISTANCE_SQR(min.x(), min.y(), max.x(), max.y());
             if ( dist2 > MIN_LINE_LENGTH || (dist2>SHORT_LINE_LENGTH && resultLine.var <=-16.f) ) {

                // Trim down the line if it is too long
                sort (onLinePoints.begin(), onLinePoints.end(),
                      cmpPoints());
                checkLine(onLinePoints, &l);

                // Check the line still isn't too short
                dist2 = DISTANCE_SQR(l.p1.x(), l.p1.y(), l.p2.x(), l.p2.y());
                if (dist2 > MIN_LINE_LENGTH || (dist2>SHORT_LINE_LENGTH && resultLine.var <=-16.f) ) {
                   RRCoord c (0,0);
                   FieldFeatureInfo f = FieldFeatureInfo(c, l);
                   lines->push_back(f);

                }
             }
          }

          field_points1.clear();
          field_points1 = field_points2;
          field_points2.clear();
       }

       // If RANSAC didn't find anything, don't bother trying again.
       else {/*
       std::cout << "totalObj: " << totalObj << std::endl;
       std::cout << "Lines: " << lines->size() << std::endl;//*/
          return;
       }
    }
}

void FieldFeatureDetector::findFieldLines(
        const std::vector<FieldLinePointInfo> &points,
        std::vector<FieldFeatureInfo> *lines,
        unsigned int *seed) {
    std::vector<Point> field_points1;
    std::vector<Point> field_points2;
    field_points1.reserve(points.size());
    field_points2.reserve(points.size());

    // RANSAC variables
    uint16_t k = 40;
    float e = 15.0;
    uint16_t n = 20;
    std::vector<bool> *con, consBuf[2];
    consBuf[0].resize(points.size());
    consBuf[1].resize(points.size());
    RANSACLine resultLine = RANSACLine(Point(0,0), Point(0,0));

    std::vector<bool> takenPoints;
    takenPoints.reserve(points.size());
    for (std::vector<FieldLinePointInfo>::const_iterator i = points.begin();
          i != points.end(); i++) {
       takenPoints.push_back(false);
       field_points1.push_back(i->rrp);
    }

    std::vector<FieldLinePointInfo> onLinePoints;
    onLinePoints.reserve(points.size());

    // Use RANSAC to find field lines
    for (uint16_t i = 0; i < 100; i++) {
       consBuf[0].clear();
       consBuf[1].clear();
       resultLine = RANSACLine(Point(0,0), Point(0,0));
       if (RANSAC::findLine (field_points1, &con, resultLine,
              k, e, n, consBuf, seed)) {

          uint16_t b = 0;
          onLinePoints.clear();
          Point max = resultLine.p1;
          Point min = resultLine.p2;
          bool horizontal = false;
          if (abs(min.x() - max.x()) < 50 && abs(min.y() - max.y()) > 50) {
             horizontal = true;
          }

          // Loop through and remove all the points on the line/circle
          for (uint16_t a = 0; a < field_points1.size(); a++, b++){
             while (takenPoints[b] == true) b++;
             if (con[0][a] == true) {
                onLinePoints.push_back(points[b]);
                takenPoints[b] = true;

                // Keep track of endpoints
                if (horizontal) {
                   if (field_points1[a].y() < min.y()) {
                      min = field_points1[a];
                   } if (field_points1[a].y() > max.y()) {
                      max = field_points1[a];
                   }
                } else {
                   if (field_points1[a].x() < min.x()) {
                      min = field_points1[a];
                   } if (field_points1[a].x() > max.x()) {
                      max = field_points1[a];
                   }
                }
             } else {
                field_points2.push_back(field_points1[a]);
             }
          }

          // Check line and save info
          LineInfo l (resultLine.p1, resultLine.p2);
          l.p1 = min;
          l.p2 = max;
          RRCoord c (0,0);
          FieldFeatureInfo f = FieldFeatureInfo(c, l);
          lines->push_back(f);

          field_points1.clear();
          field_points1 = field_points2;
          field_points2.clear();
       }

       // If RANSAC didn't find anything, don't bother trying again.
       else {
          return;
       }
    }
}

void FieldFeatureDetector::findIntersections(std::vector<FieldFeatureInfo> &lines, std::vector<FieldFeatureInfo> *features) {
    if (lines.size() < 2) return;
    for (uint16_t i = 0; i < (lines.size()); i++) {
       while (lines[i].type != FieldFeatureInfo::fLine) {
          i++;
          if (i >= lines.size()) {
             return;
          }
       }
       LineInfo l1 = lines[i].line;
       for (uint16_t j = i+1; j < (lines.size()); j++) {
          while (lines[j].type != FieldFeatureInfo::fLine) {
             j++;
             if (j >= lines.size()) {
                return;
             }
          }
          LineInfo l2 = lines[j].line;

          Point p = Point(0,0);
          p = intersect(l1, l2);
          if ((p.x() != 0) && perpendicular(l1,l2)) {
             FieldFeatureInfo f;

             // If X intersection
             if (possibleT(l1, l2) &&
                 possibleT(l2, l1)) {
                f = FieldFeatureInfo();
             }

             // If T intersection
             else if (possibleT(l1, l2)) {
                float angle = findTAngle(p, l2);
                float distance = sqrt(p.x() * p.x() + p.y() * p.y());
                float heading = atan2(p.y(), p.x());
                RRCoord r = RRCoord(distance, heading, -angle);
                p = convertRR_->convertToImageXY(p);
                TJunctionInfo t = TJunctionInfo(p);
                f = FieldFeatureInfo(r, t);
             }
             // If T intersection
             else if (possibleT(l2, l1)) {
                float angle = findTAngle(p, l1);
                float distance = sqrt(p.x() * p.x() + p.y() * p.y());
                float heading = atan2(p.y(), p.x());
                RRCoord r = RRCoord(distance, heading, -angle);
                p = convertRR_->convertToImageXY(p);
                TJunctionInfo t = TJunctionInfo(p);
                f = FieldFeatureInfo(r, t);
             }
             // If corner
             else {
                float angle = findCAngle(p, l1, l2);
                if (isBadCorner(l1,l2)) {
                   //cout << "bad corner" << endl;
                   continue;
                }
                Point e1, e2;
                findCornerEndpoints(l1, l2, e1, e2);
                float distance = sqrt(p.x() * p.x() + p.y() * p.y());
                float heading = atan2(p.y(), p.x());
                RRCoord r = RRCoord(distance, heading, -angle);
                p = convertRR_->convertToImageXY(p);
                e1 = convertRR_->convertToImageXY(e1);
                e2 = convertRR_->convertToImageXY(e2);
                CornerInfo c = CornerInfo(p, e1, e2);
                f = FieldFeatureInfo(r, c);
             }
            // TODO: find out if checking close to edge is still relevant with colour ROI
            //  if (!closeToEdge(fovea, f)) {
                features->push_back(f);
                lines[i].lineUsed = true;
                lines[j].lineUsed = true;
            //  } else {
                //std::cout << "CLOSE CORNER FAIL :(" << std::endl;
            //  }
             break;
          }
 /*
          else if (p.x() != 0) {
             RRCoord r = RRCoord(0,0);
             if (parallelPair(l1, l2, &r)) {
                lines[i].lineUsed = true;
                lines[j].lineUsed = true;
                r = RRCoord(0,0);
                ParallelLinesInfo l = ParallelLinesInfo(l1,l2);
                FieldFeatureInfo f = FieldFeatureInfo(r, l);
                //features->clear();
                features->push_back(f);
             } else {
 //               cout << "clear" << endl;
                //features->clear();
                return;
             }
          }
 */
       }
    }
}

void FieldFeatureDetector::mergeFeatures(std::vector<FieldFeatureInfo> &features) {
    for(uint32_t i=0; i<features.size(); i++){
        /* This can be expaneded to merge in more than just corner + Tjunction*/
        if (features[i].type != FieldFeatureInfo::fCorner) continue;
        for(uint32_t j=0; j<features.size(); j++){
           if (features[j].type != FieldFeatureInfo::fTJunction) continue;

           if (possibleGoalBoxCorner(features[i].corner, features[j].tjunction)){
               /* Make the feature and push back. For simplicity, use the rr and point from corner */
               bool left_corner = isLeftGoalBoxCorner(features[i].corner, features[j].tjunction);
               GoalBoxCornerInfo h = GoalBoxCornerInfo(features[i].corner.p, left_corner);
               // Differentiate left and right here by checking the angles/lines
               // from the field features;
               FieldFeatureInfo f = FieldFeatureInfo(features[i].rr, h);
               features.push_back(f);
           }

        }
    }
}

void FieldFeatureDetector::findParallelLines(std::vector<FieldFeatureInfo>* lines) {
    for (unsigned int i = 0; i < lines->size(); ++i) {
        if ((*lines)[i].type == FieldFeatureInfo::fLine) {
            for (unsigned int j = i; j < lines->size(); ++j) {
                if ((*lines)[j].type == FieldFeatureInfo::fLine) {
                    RRCoord r = RRCoord(0,0);
                    LineInfo l1 = (*lines)[i].line;
                    LineInfo l2 = (*lines)[j].line;
                    if (parallelPair(l1, l2, &r)) {
                        (*lines)[i].lineUsed = true;
                        (*lines)[j].lineUsed = true;
                        r = RRCoord(0,0);
                        ParallelLinesInfo l = ParallelLinesInfo(l1,l2);
                        FieldFeatureInfo f = FieldFeatureInfo(r, l);
                        lines->push_back(f);
                        return;
                    }
                }
            }
        }
    }
}

void FieldFeatureDetector::circleOrientation(std::vector<FieldFeatureInfo> *features) {
    LineInfo line = LineInfo();
    FieldFeatureInfo circle = FieldFeatureInfo();

    // find line and circle from list of features
    for (uint16_t i = 0; i < features->size(); i++) {
       if ((*features)[i].type == FieldFeatureInfo::fLine) {
          line = (*features)[i].line;
       } else if ((*features)[i].type == FieldFeatureInfo::fCentreCircle) {
          circle = (*features)[i];
       }
    }
    // maybe add check here to ensure line and circle are set
    Point c = circle.rr.toCartesian();
    float denom = sqrt (line.t1*line.t1 + line.t2*line.t2);
    float dist = (line.t1 * c.x() + line.t2 * c.y() + line.t3) / denom;
    if (fabs(dist) > 200) return; // check line is within 20cm of circle centre

    // check intersection
    Point centreRight = line.p1;
    Point centreLeft = line.p2;
    if (centreRight.y() > centreLeft.y()) {
       centreRight = line.p2;
       centreLeft = line.p1;
    }
    float centreLineAngle = atan2(centreRight.y() - centreLeft.y() ,
                              centreRight.x() - centreLeft.x());

    Point directionRight = Point(0,0);
    Point directionLeft = circle.rr.toCartesian();
    if (directionRight.y() > directionLeft.y()) {
       directionRight = circle.rr.toCartesian();
       directionLeft = Point(0,0);
    }
    float robotCentreAngle = atan2(directionRight.y() - directionLeft.y(),
                                   directionRight.x() - directionLeft.x());

    float angle = robotCentreAngle - centreLineAngle;
    //check nan?

    if (angle < 0) {
       angle += M_PI;
    }

    circle.rr.orientation() = angle;

    // TODO: Sean maybe remove the clear from  here?
    //features->clear();
    features->push_back(circle);

}

void FieldFeatureDetector::findPenaltySpot() {

}

void FieldFeatureDetector::reset(bool full) {
   field_line_points_.clear();
   temp_features_.clear();
   // linePoints.clear();
   // lineEdges.clear();
   // circlePoints.clear();
   if (full) {
      field_features_.clear();
   }
}

void FieldFeatureDetector::checkLine(
            const std::vector<FieldLinePointInfo> &points,
            LineInfo *l1) {

   std::vector<FieldLinePointInfo>::const_iterator it;
   std::vector<FieldLinePointInfo>::const_iterator start = points.end();
   std::vector<FieldLinePointInfo>::const_iterator end = points.end();
   std::vector<FieldLinePointInfo>::const_iterator bestStart = points.begin();
   std::vector<FieldLinePointInfo>::const_iterator bestEnd = points.end();

   int min = 300 * 300;

   Point prev = Point(0,0);
   for (it = points.begin(); it != points.end(); ++it) {
      if (prev.x() != 0) {
         Point curr = it->rrp;
         int dist = DISTANCE_SQR(curr.x(), curr.y(), prev.x(), prev.y());
         if (dist > min) {
            if (start == points.end()) {
               start = it;
               bestEnd = it;
            } else if (end == points.end()) {
               end = it;
               if (end - start > bestEnd - bestStart) {
                  bestStart = start;
                  bestEnd = end;
               }
               start = it;
               end = points.end();
            }
         }
      }
      prev = it->rrp;
   }
   // Case for the end of the vector
   end = it;
   if (end - start > bestEnd - bestStart) {
      bestStart = start;
      bestEnd = end;
   }
   bestEnd--;

  l1->p1 = bestStart->rrp;
  l1->p2 = bestEnd->rrp;

}

Point FieldFeatureDetector::intersect(LineInfo l1, LineInfo l2) {
   Eigen::Matrix2f A;
   Eigen::Vector2f b, x;
   Point p = Point(0,0);

   A << l1.t1, l1.t2,
        l2.t1, l2.t2;
   b << -l1.t3, -l2.t3;

   // ALTERED WITH EIGEN UPDATE. Untested, may need a fix.
   x = A.lu().solve(b);
   p = Point(x.x(), x.y());

   return p;
}

bool FieldFeatureDetector::perpendicular(LineInfo l1, LineInfo l2) {

   float theta = atan2f(
              (-1*(l1.t1)/(float)(l1.t2) + (l2.t1)/(float)(l2.t2)),
              (1 + (l1.t1)/(float)(l1.t2) * (l2.t1)/(float)(l2.t2)));
   theta = fabs(theta) - DEG2RAD(90);
   if (isnan(theta)) return false;
   if ((fabs(theta) < PERPENDICULAR_THRESHOLD)) {
      return true;
   }
   return false;
}



bool FieldFeatureDetector::isBadCorner(LineInfo l1, LineInfo l2) {
   float denom = sqrt(l1.t1*l1.t1 + l1.t2*l1.t2);
   float distl1p1 = (l1.t1*l2.p1.x() + l1.t2*l2.p1.y() + l1.t3) / denom;
   float distl1p2 = (l1.t1*l2.p2.x() + l1.t2*l2.p2.y() + l1.t3) / denom;

   if ((fabs(distl1p1) > 200.f) && (fabs(distl1p2) > 200.f)) {
      return true;
   }
   denom = sqrt(l2.t1*l2.t1 + l2.t2*l2.t2);
   float distl2p1 = (l2.t1*l1.p1.x() + l2.t2*l1.p1.y() + l2.t3) / denom;
   float distl2p2 = (l2.t1*l1.p2.x() + l2.t2*l1.p2.y() + l2.t3) / denom;
   if ((fabs(distl2p1) > 200.f) && (fabs(distl2p2) > 200.f)) {
      return true;
   }
   return false;
}

float FieldFeatureDetector::findTAngle(Point p, LineInfo l) {
   float angle = findGradient(l, p);
   float theta = atan2(p.x(), p.y());

   if (angle > 0) {
      angle = theta + DEG2RAD(180) - angle;
   } else {
      angle = theta - (DEG2RAD(180) + angle);
   }

   if (angle > M_PI) {
      angle = NORMALISE(angle);
   }
   return angle;
}


float FieldFeatureDetector::findCAngle(Point p, LineInfo l1, LineInfo l2) {
   float g1 = findGradient(l1, p);
   float g2 = findGradient(l2, p);
   float angle = (g1+g2)/2;
   float roughQuadrantLimit = DEG2RAD(80);
   if ((g1 > 0) && (g2 < 0) &&
         (g1 > roughQuadrantLimit) && (g2 < -roughQuadrantLimit)) {
      g2 += 2*M_PI;
      angle = (g1+g2)/2;
   }
   if ((g2 > 0) && (g1 < 0) &&
         (g2 > roughQuadrantLimit) && (g1 < -roughQuadrantLimit)) {
      g1 += 2*M_PI;
      angle = (g1+g2)/2;
   }
   if (angle > M_PI) angle -= 2*M_PI;

   float theta = atan2(p.x(), p.y());

   if (angle > 0) {
      angle = theta + DEG2RAD(180) - angle;
   } else {
      angle = theta - (DEG2RAD(180) + angle);
   }

   if (angle > M_PI) {
      angle = NORMALISE(angle);
   }
   return angle;
}


bool FieldFeatureDetector::possibleT(LineInfo l1,
                                   LineInfo l2) {

   // Return true if l1 could be the top line in the T

   // Check endpoints of l1 are sufficiently far away from l2
   float denom  = sqrt(l2.t1*l2.t1 + l2.t2*l2.t2);
   float distp1 = (l2.t1*l1.p1.x() + l2.t2*l1.p1.y() + l2.t3) / denom;
   float distp2 = (l2.t1*l1.p2.x() + l2.t2*l1.p2.y() + l2.t3) / denom;

   if ((distp1 < 0.f) ^ (distp2 < 0.f)) {
       // enforce at least 10cm of distance between endpoints, this was previously 4cm which was creating false positives on corners
      if (fabs(distp1) > 100.f && fabs(distp2) > 100.f) {
         // Check at least one endpoint of l2 is near l1
         denom = sqrt(l1.t1*l1.t1 + l1.t2*l1.t2);
         distp1 = (l1.t1*l2.p1.x() + l1.t2*l2.p1.y() + l1.t3) / denom;
         if (fabs(distp1) < 200.f) return true;
         distp2 = (l1.t1*l2.p2.x() + l1.t2*l2.p2.y() + l1.t3) / denom;
         if (fabs(distp2) < 200.f) return true;
      }
   }

   return false;
}

void FieldFeatureDetector::findCornerEndpoints(LineInfo l1, LineInfo l2,
        Point& e1, Point& e2) {
    // Just the distance formula folks. Corner endpoints are the ones that are
    // the furtherst away from each other. We already checked for right angle.

    double max_dist_sqr = 0.0;
    if (DISTANCE_SQR(l1.p1.x(), l1.p1.y(), l2.p1.x(), l2.p1.y()) > max_dist_sqr){
        e1 = l1.p1;
        e2 = l2.p1;
    }

    if (DISTANCE_SQR(l1.p1.x(), l1.p1.y(), l2.p2.x(), l2.p2.y()) > max_dist_sqr){
        e1 = l1.p1;
        e2 = l2.p2;
    }

    if (DISTANCE_SQR(l1.p2.x(), l1.p2.y(), l2.p1.x(), l2.p1.y()) > max_dist_sqr){
        e1 = l1.p2;
        e2 = l2.p1;
    }

    if (DISTANCE_SQR(l1.p2.x(), l1.p2.y(), l2.p2.x(), l2.p2.y()) > max_dist_sqr){
        e1 = l1.p2;
        e2 = l2.p2;
    }
}

bool FieldFeatureDetector::closeToEdge(const Fovea &fovea, FieldFeatureInfo f) {
   Point p;
   if (f.type == FieldFeatureInfo::fCorner) {
      p = fovea.mapImageToFovea(f.corner.p);
   } else if (f.type == FieldFeatureInfo::fTJunction) {
      p = fovea.mapImageToFovea(f.tjunction.p);
   } else {
      return true;
   }

   if (p.x() < EXCLUSION_LENGTH) return true;
   if (p.y() < EXCLUSION_LENGTH) return true;
   if (p.x() > (fovea.getBBox().width() - EXCLUSION_LENGTH)) return true;
   if (p.y() > (fovea.getBBox().height() - EXCLUSION_LENGTH)) return true;
   return false;
}

float FieldFeatureDetector::findGradient(LineInfo l, Point p) {
   // Work out which direction to find gradient in
   float distp1 = DISTANCE_SQR(p.x(), p.y(), l.p1.x(), l.p1.y());
   float distp2 = DISTANCE_SQR(p.x(), p.y(), l.p2.x(), l.p2.y());
   Point far;
   Point close;
   if (distp1 > distp2) {
      far = l.p1;
      close = l.p2;
   } else {
      far = l.p2;
      close = l.p1;
   }
   return (atan2(far.x() - close.x(), far.y() - close.y()));
}

bool FieldFeatureDetector::possibleGoalBoxCorner(CornerInfo c, TJunctionInfo t){
    /* The side of a T should be sufficiently close to the corner c.
     * We enforce a minimum so that no false positives occur when vision slips
     * up and sees the the T junction as a corner and a T.
     */
    Point real_c = convertRR_->convertToRRXY(c.p);
    Point real_t = convertRR_->convertToRRXY(t.p);
    float dist = DISTANCE_SQR(real_c.x(), real_c.y(), real_t.x(), real_t.y());
    /* 600 is the goal box width, 50 is the error*/
    //std::cout << "DIST " << dist << std::endl;
    if (dist > 400 * 400 && dist < 700 * 700){
        return true;
    }
    return false;
}


/* Checks if this is a left or right goal box corner. Returns true if left.
 * This is from the viewpoint of the STRIKER.
 * Either the left or the right enedpoint of the corner should be near the
 * T junction. The other one won't be. We inspect that point to determine
 * our left and rights.
*/
bool FieldFeatureDetector::isLeftGoalBoxCorner(CornerInfo c, TJunctionInfo t){

    Point t_point, end_point;
    if (DISTANCE_SQR(c.e1.x(), c.e1.y(), t.p.x(), t.p.y()) >
        DISTANCE_SQR(c.e2.x(), c.e2.y(), t.p.x(), t.p.y()))
    {
        // e1 is further away to the TJunction point. Use this to determine
        // orientation.
        t_point = c.e2;
        end_point = c.e1;

    } else {
        t_point = c.e1;
        end_point = c.e2;
    }

    // Simply inspect if this point is to the left or right of Tjunction point.
    // This is the "orientation of three ordered points" fromula.
    int result = (c.p.y() - t_point.y()) * (end_point.x() - c.p.x()) -
          (c.p.x() - t_point.x()) * (end_point.y() - c.p.y());
    if (result > 0){
        // Counterclockwise points. Thss forms a right goal.
        return true;
    } else {
        // Points go clockwise, so this will be the right goal box corner from
        // striker's POV
        return false;
    }
}


bool FieldFeatureDetector::parallelPair(LineInfo l1,
                                      LineInfo l2,
                                      RRCoord *r) {
   bool parallel = false;

   float angle1 = atan2 (l1.p1.x() - l1.p2.x(), l1.p1.y() - l1.p2.y());
   float angle2 = atan2 (l2.p1.x() - l2.p2.x(), l2.p1.y() - l2.p2.y());
   if (fabs(angle1-angle2) < PARALLEL_LINE_THRESHOLD) {
      parallel = true;
   }

   // Make sure we try swapping p1 and p2, incase they were the wrong way
   float angle3 = atan2 (l2.p2.x() - l2.p1.x(), l2.p2.y() - l2.p1.y());
   if (fabs(angle1-angle3) < PARALLEL_LINE_THRESHOLD) {
      parallel = true;
   }
   if (!parallel) return false;

   float lenSqr1 = DISTANCE_SQR(l1.p1.x(), l1.p1.y(), l1.p2.x(), l1.p2.y());
   float lenSqr2 = DISTANCE_SQR(l2.p1.x(), l2.p1.y(), l2.p2.x(), l2.p2.y());
   if (lenSqr1 < MIN_PARALLEL_LENGTH || lenSqr2 < MIN_PARALLEL_LENGTH) return false;

   float dist1 = fabs(l1.t3) / sqrt(SQUARE(l1.t1) + SQUARE(l1.t2));
   float dist2 = fabs(l2.t3) / sqrt(SQUARE(l2.t1) + SQUARE(l2.t2));
   if (fabs (fabs(dist1-dist2) - GOAL_BOX_LENGTH) < PARALLEL_LINE_DISTANCE_THRESHOLD) {
      if (dist1 < dist2) {
         float theta = atan2f (((float)-l1.t1) * ((float)l1.t3),
                               ((float)l1.t2)  * ((float)l1.t3));
         *r = RRCoord(dist1, theta);
      } else {
         float theta = atan2f (((float)-l2.t1) * ((float)l2.t3),
                               ((float)l2.t2)  * ((float)l2.t3));
         *r = RRCoord(dist2, theta);
      }
      return true;
   }
   return false;
}
