#include "NaiveHorizonFieldBoundaryFinder.hpp"

#include "perception/vision/VisionDefinitions.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/Point.hpp"
#include "types/RansacTypes.hpp"

// VERTICAL_OFFSET is the amount we can subtract from the horizon and
// still have the field (almost) guaranteed to be within the boundry
// It's a conservative calculation based on the angle between the horizon
// and the opposite corner of the field where the Nao is standing on a
// field corner.
// Some field overlap may occur when the Nao's head is tilted. Otherwise
// it will definitely overestimate (but not as much as the horizon by
// iteslf).
const int VERTICAL_OFFSET = 50;

void NaiveHorizonFieldBoundaryFinder::find(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
    for(unsigned int region=0; region<info_middle.full_regions.size(); ++region)
        detect(info_in, info_middle.full_regions[region], info_out);
}

void NaiveHorizonFieldBoundaryFinder::detect(const VisionInfoIn& info_in, const RegionI& region, VisionInfoOut& info_out) {
    // The horizon values represent the y coordinates of the horizon at
    //   x = 0  and x = TOP_IMAGE_COLS
    // as per the Pose.hpp file
    //
    // We subtract the VERTICAL_OFFSET from the y coordinates of the
    // horizon for two reasons:
    //  a) to reduce the unecessary off field space
    //  b) so it is possible to calculate a robot relative boundry
    std::pair<int, int> horizon = info_in.pose.getHorizon();
    Point horizon_left(0, horizon.first + VERTICAL_OFFSET);
    Point horizon_right(TOP_IMAGE_COLS, horizon.second + VERTICAL_OFFSET);

    RANSACLine imageBoundary(horizon_left, horizon_right);
    RANSACLine robotRelativeBoundary(info_in.pose.imageToRobotXY(horizon_left), info_in.pose.imageToRobotXY(horizon_right));

    info_out.boundaries.push_back(FieldBoundaryInfo(robotRelativeBoundary, imageBoundary));

    findStartScanCoords(region, info_out);

}

void NaiveHorizonFieldBoundaryFinder::findStartScanCoords(const RegionI& region, VisionInfoOut& info_out) {

        const int COLS = (region.isTopCamera()) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
        const int ROWS = (region.isTopCamera()) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

        int cols = (region.isTopCamera()) ? TOP_SALIENCY_COLS : BOT_SALIENCY_COLS;
        const std::pair<int, int> &horizon = info_out.cameraToRR->pose.getHorizon();
        float gradient = (horizon.second - horizon.first) / float(COLS);
        int intercept = horizon.first / region.getDensity();

        uint16_t i;
        int32_t intersec;

        /* Calculate the number of field boundaries in this image */
        int numBoundaries = 0;
        for (std::vector<FieldBoundaryInfo>::const_iterator it = info_out.boundaries.begin();
        it != info_out.boundaries.end(); ++ it) {
            if ((*it).imageBoundary.p1.y() < ROWS && region.isTopCamera()) {
                numBoundaries++;
            } else if ((*it).imageBoundary.p1.y() > ROWS && !region.isTopCamera()) {
                numBoundaries++;
            }
        }

        /* If there are no lines, work out if the entire image consists of
        * the field or the entire image is of the background.
        */
        if (numBoundaries == 0) {
            int numGreen = 0;
            for (int x = 0; x < cols; x++) {
                if (region.getPixelColour(x,0) == cGREEN) {
                    numGreen++;
                }
            }
            if (numGreen < MIN_GREEN_THRESHOLD) {
                for (int x = 0; x < COLS; x++) {
                    // There is no field seen, so give all the array values that
                    // far exceed the number of rows in the saliency scan so that
                    // no scan lines are used
                    if (region.isTopCamera()) {
                        info_out.topStartScanCoords[x] = ROWS;
                    } else {
                        info_out.botStartScanCoords[x] = ROWS*2;
                    }
                }
            } else {
                if (region.isTopCamera()) {
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
                for (i = 0; i < info_out.boundaries.size(); ++ i) {
                    const RANSACLine &line = info_out.boundaries[i].imageBoundary;
                    if (line.p1.y() < ROWS && region.isTopCamera()) {
                        intersec = (-line.t3 - (x * line.t1))/line.t2;
                        if (intersec > maxVal) {
                            maxVal = intersec;
                        }
                    } else if (line.p1.y() >= ROWS && !region.isTopCamera()) {
                        intersec = (-line.t3 - (x * line.t1))/line.t2;
                        if (intersec > maxVal) {
                            maxVal = intersec;
                        }
                    }
                }
                if (maxVal < 0) {
                    maxVal = 0;
                }
                if (region.isTopCamera()) {
                    info_out.topStartScanCoords[x] = std::max(0, std::min(ROWS, maxVal));
                } else {
                    info_out.botStartScanCoords[x] =
                    std::max(ROWS, std::min(ROWS*2, maxVal));
                }
            }
        }
    }
