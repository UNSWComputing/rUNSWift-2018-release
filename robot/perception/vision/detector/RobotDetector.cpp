
#include "RobotDetector.hpp"
#include <iostream>
#include <fstream>

RobotDetector::RobotDetector() {
//#ifdef ROBOT_DETECTOR_USES_VATNAO
//    debugger_ = new RobotDetectorDebugger();
//#endif
	regionFinder = new RobotColorROI();
}

void RobotDetector::detect(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {
#ifdef ROBOT_DETECTOR_TIMER
    std::vector<float> timings;
    Timer timer;
    timer.restart();
#endif

    // Declare a new info_middle so we don't edit the actual one
    VisionInfoMiddle editable_middle = info_middle;

    // ColorRoi modifies info_out. We don't want to add my regions to the base regions so give it a dummy.
    VisionInfoOut newOut;

    // Step 1) - Rethreshold downsampled full regions
	runThresholding(info_in, editable_middle, newOut);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif

    // Step 2) - Rerun a modified colorROI to generate regions
	runColorRoi(info_in, editable_middle, info_out);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif

    // Step 3) - Use density based clustering to generate candidates
    clusters = DBSCAN(editable_middle.roi, 70, 1);
    //clusters = createCandidateRobots(editable_middle.roi);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    timer.restart();
#endif
    // Step 40 Run the classifier
    findRobots(editable_middle, info_out, clusters);
#ifdef ROBOT_DETECTOR_TIMER
    timings.push_back(timer.elapsed_us());
    debugger_->outputTimings(timings, editable_middle.roi.size(), clusters.size());
#endif
//#ifdef ROBOT_DETECTOR_USES_VATNAO
//    debugger_->highlightCandidates(clusters);
//#endif
#ifdef OUTPUT_DATA
    debugger_->outputToFile(clusters);
#endif

    delete(newTop_);
    //delete(newBot_);

    return;
}

void RobotDetector::runThresholding(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

    newTop_ = new RegionI(info_middle.full_regions[0],4, DENSITY_INCREASE, true, ATR_WINDOW,
                                                          ATR_MEAN_PERCENT);
	//newBot_ = new RegionI(info_middle.full_regions[1],1, DENSITY_INCREASE, true, ATR_WINDOW,
    //                                                      ATR_MEAN_PERCENT);

    info_middle.full_regions[0] = *newTop_;
    //info_middle.full_regions[1] = *newBot_;

    return;
}

void RobotDetector::runColorRoi(const VisionInfoIn& info_in, VisionInfoMiddle& info_middle, VisionInfoOut& info_out) {

	//ensure to clear out existing rois
	info_middle.roi.clear();

	// Run the region finder. This will update info_middle
	regionFinder->find(info_in, info_middle, info_out);

    activatedCounts_ = regionFinder->activatedCounts();

//#ifdef ROBOT_DETECTOR_USES_VATNAO
//	debugger_->highlightRegions(info_middle.roi);
//    debugger_->showSaliencyImage(*newTop_);
//#endif
	return;
}


Eigen::MatrixXf RobotDetector::convertMat(RegionI& region){

    RegionI::iterator_fovea cur_point = region.begin_fovea();
    // Track the literal location of the iterators.
    int x = 0;
    int y = 0;
    // The number of rows and columns in the region.
    int rows = region.getRows();
    int cols = region.getCols();
    Eigen::MatrixXf dst(rows, cols);
    // Loop
    for(int pixel = 0; pixel < cols * rows; ++pixel)
    {
        // std::cout<<cur_point.colour() << " ";
        if (cur_point.colour() == cWHITE){
            dst(y, x) = 1;
        }
        else{
            dst(y, x) = 0;
        }
        cur_point++;
        x++;
        if (x == cols){
            x = 0;
            ++y;
        }
    }
    return dst;
}


void RobotDetector::featureExtraction(VisionInfoMiddle& info_mid, BBox bound, std::vector<float>& features){
    RegionI *newBoundRegion = new RegionI(info_mid.full_regions[0], bound, 1, DENSITY_MAINTAIN, false);
    features.push_back((float)bound.height()/bound.width());
    features.push_back(bound.height()*bound.width());

    Eigen::MatrixXf imageMat = convertMat(*newBoundRegion);
    features.push_back(imageMat.mean());

    float m00 = imageMat.sum();
    float m10 = 0;
    float m01 = 0;
    for (int i = 0; i < imageMat.rows(); i++){
        for (int j = 0; j < imageMat.cols(); j++){
            m10 += imageMat(i, j) * i;
            m01 += imageMat(i, j) * j;
        }
    }
    features.push_back((m10/m00)/bound.width());
    features.push_back((m01/m00)/bound.height());
    delete newBoundRegion;
}





// Runs the random forest classification and stores the candidates in info_out
void RobotDetector::findRobots(VisionInfoMiddle& info_mid, VisionInfoOut& info_out, std::vector<Cluster>& candidates) {

    std::vector<RobotInfo> robots;

    for(size_t i = 0; i < candidates.size(); ++i) {
        BBox bound = candidates[i].box_;
        if (bound.width() > bound.height()) continue;

        Point aa, bb;
        aa << bound.a.x()/16, bound.a.y()/16;
        bb << bound.b.x()/16, bound.b.y()/16;
        BBox _box(aa, bb);
        std::vector<float> features;
        featureExtraction(info_mid, _box, features);


        float confidence;

        int classification = classifier_.classify(features, confidence);
        candidates[i].isRobot_ = classification;
        candidates[i].confidence_ = confidence;
        if(classification == 0||confidence <= 0.7)
            continue;

        Point pointForRR = Point(bound.b.x()-bound.a.x(), bound.b.y() + (!candidates[i].isTopCamera()*TOP_IMAGE_ROWS));
        Point feet(bound.width()/2 + bound.a.x(), bound.b.y() + (!candidates[i].isTopCamera()*TOP_IMAGE_ROWS));
        RRCoord rr_feet = info_out.cameraToRR->convertToRR(feet, false);
        if (candidates[i].isTopCamera())
            robots.push_back(RobotInfo(rr_feet, RobotInfo::rUnknown, bound, RobotInfo::TOP_CAMERA));
        else
            robots.push_back(RobotInfo(rr_feet, RobotInfo::rUnknown, bound, RobotInfo::BOT_CAMERA));

    }

    info_out.robots = robots;

    return;
}

// Naively creates candidate clusters by merging any regions that overlap each other
std::vector<Cluster> RobotDetector::createCandidateRobots(std::vector<RegionI>& regions) {
	std::vector<Cluster> clusters;

    size_t processed = 0;
    std::vector<bool> isProcessed(regions.size(), false);

    //all regions will eventually be added to a cluster (even if its just the one region)
    while(processed < regions.size()) {
        Cluster newCluster = Cluster();
        int merges = -1;
        while(merges != 0) {
            merges = 0;
            for(size_t i = 0; i < regions.size(); ++i) {
                if(isProcessed[i])
                    continue;

                if(regions[i].getCols()/regions[i].getRows() > 8) {
                    processed++;
                    isProcessed[i] = true;
                    continue;
                }
                //if its the first region in the cluster, add it
                else if(newCluster.regions_.empty()) {
                    newCluster.addRegionToCluster(regions[i], activatedCounts_[i]);
                    isProcessed[i] = true;
                    processed++;
                }
                //if it overlaps the cluster add it
                else if(newCluster.overlapsRegion(regions[i])) {
                    newCluster.addRegionToCluster(regions[i], activatedCounts_[i]);
                    isProcessed[i] = true;
                    processed++;
                    merges+=1;
                }
            }
        }
        clusters.push_back(newCluster);
    }
	return clusters;
}

// Creates candidate clusters by merging using a slightly modified Density based clustering algorithm
std::vector<Cluster> RobotDetector::DBSCAN(std::vector<RegionI>& regions, int epsilon, unsigned int minPoints) {
    //-2 = unprocessed
    //-1 = noise
    std::vector<int> labels(regions.size(), -2);

    //cluster label
    int c = 0;

    for(unsigned int regionID = 0; regionID < regions.size(); regionID++) {
        BBox reg = regions[regionID].getBoundingBoxRaw();
        //region already processed so keep going
        if(regions[regionID].getCols()/regions[regionID].getRows() > 8) {
            labels[regionID] = -1;
        }


        if(labels[regionID] != -2)
            continue;


        std::vector<int> neighbours = rangeQuery(regions, regionID, epsilon);
        if(neighbours.size() < minPoints) {
            labels[regionID] = -1; //noise since less than required points
            continue;
        }



        c++;
        labels[regionID] = c;

        for(unsigned int connectedRegion = 0; connectedRegion<neighbours.size(); connectedRegion++) {
            if(labels[neighbours[connectedRegion]] == -1)
                labels[neighbours[connectedRegion]] = c;
            if(labels[neighbours[connectedRegion]] != -2)
                continue;
            labels[neighbours[connectedRegion]] = c;

            std::vector<int> connectedNeighbours = rangeQuery(regions, neighbours[connectedRegion], epsilon);

            if(connectedNeighbours.size() >= minPoints) {
                neighbours.insert(neighbours.end(), connectedNeighbours.begin(), connectedNeighbours.end());
            }
        }
    }


    std::vector<Cluster> clusters;
    //now need to put appropriate regions in their clusters
    for(int i = 0; i < c; ++i) {
        clusters.push_back(Cluster());
    }


    for(unsigned int regionID = 0; regionID != labels.size(); regionID++) {
        if(labels[regionID] >=0) {
            clusters[labels[regionID]-1].addRegionToCluster(regions[regionID], activatedCounts_[regionID]);
        }
    }
    return clusters;
}

// Finds the neighbours for a particular region (governed by currRegionID) and epsilon
std::vector<int> RobotDetector::rangeQuery(std::vector<RegionI>& regions, int currRegionID, int epsilon) {
    std::vector<int> neighbours;
    for(unsigned int regionID = 0; regionID < regions.size(); regionID++) {
        if(regions[regionID].getCols()/regions[regionID].getRows() > 8) {
            continue;
        }
        if(regions[regionID].isTopCamera() != regions[currRegionID].isTopCamera()) {
            continue;
        }
        BBox currRegBox = regions[currRegionID].getBoundingBoxRaw();

    // Classify the region as a neighbour if it either overlaps the region or is within epsilon of it
    if(distance(regions[currRegionID], regions[regionID]) <= epsilon || overlaps(regions[currRegionID], regions[regionID])) {
            neighbours.push_back(regionID);
        }
    }
    return neighbours;
}

//note could have a variable epsilon based on size of region.

//distance between the centrepoints of the square.
int RobotDetector::distance(const RegionI& regionA, const RegionI& regionB) {

    double aX = (regionA.getBoundingBoxRaw().b.x() + regionA.getBoundingBoxRaw().a.x())/2;
    double aY = (regionA.getBoundingBoxRaw().b.y() + regionA.getBoundingBoxRaw().a.y())/2;
    double bX = (regionB.getBoundingBoxRaw().b.x() + regionB.getBoundingBoxRaw().a.x())/2;
    double bY = (regionB.getBoundingBoxRaw().b.y() + regionB.getBoundingBoxRaw().a.y())/2;


    double distance = std::sqrt((aX-bX)*(aX-bX) + (aY-bY)*(aY-bY));

    return distance;
}

// Determines whether regionB is in the elipse of region A
bool RobotDetector::inEllipse(const RegionI& regionA, const RegionI& regionB) {
    double aX = (regionA.getBoundingBoxRaw().b.x() + regionA.getBoundingBoxRaw().a.x())/2;
    double aY = (regionA.getBoundingBoxRaw().b.y() + regionA.getBoundingBoxRaw().a.y())/2;
    double bX = (regionB.getBoundingBoxRaw().b.x() + regionB.getBoundingBoxRaw().a.x())/2;
    double bY = (regionB.getBoundingBoxRaw().b.y() + regionB.getBoundingBoxRaw().a.y())/2;

    // Modfy the size of the ellipse by setting these parameters
    float rx = (regionA.getBoundingBoxRaw().width())/1;
    float ry = (regionA.getBoundingBoxRaw().height())/1;

    if(((bX-aX)*(bX-aX)/(rx*rx))+((bY-aY)*(bY-aY))/(ry*ry) <= 1) {
        return true;
    }
    return false;

    //equation of ellipse

}

// Determines whether 2 regions overlap each other
bool RobotDetector::overlaps(RegionI& regionA, RegionI& regionB) {
    Cluster newCluster;
    newCluster.addRegionToCluster(regionA, 0);
    return newCluster.overlapsRegion(regionB);
}




/******************************* DEBUGGER *************************/
// #ifdef ROBOT_DETECTOR_USES_VATNAO
// RobotDetectorDebugger::RobotDetectorDebugger() {
// 	if (vdm != NULL) {
//         	vdm->addOption("Show RobotDetector Regions");
//         	vdm->addOption("Show whole saliency image");
// 		vdm->addOption("Show candidate robots");
//     }
//     dataHeaderWritten_ = false;
//     dataHeaderWrittenTimings_ = false;
// }

// void RobotDetectorDebugger::highlightRegions(std::vector<RegionI>& regions) {
//     //first check that vision debugger module exists (this is defined elsewhere so you shouldn't need to worry about it)
//     if(vdm != NULL) {

//         // create a query object to access vatnao UI information
//         VisionDebugQuery myQ = vdm->getQuery();

//         // check whether the 'tick box' is selected for showing curved regions
//         if (myQ.options["Show RobotDetector Regions"] == "true") {
//             for(size_t i = 0; i < regions.size(); ++i) {
//                 RegionI currRegion = regions[i];
//                 // get the bounding box of the region
//                 BBox boundingBox = currRegion.getBoundingBoxRaw();

//                 // Prety sure the painter function starts from the top left of the region
//                 int topLeftX = boundingBox.a.x();
//                 int topLeftY = boundingBox.a.y();

//                 // need to create a vatnao painter object which is our interface into the vatnao display frame
//                 // 5 defines the factor of downsampling for the painter. true = top camera, false = bottom
//                 // We want to downsample the image because that means we will get thicker lines drawn by vatnao
//                 VisionPainter *myPainterTop = vdm->getFrameOverlayPainter(5, true);
//                 VisionPainter *myPainterBot = vdm->getFrameOverlayPainter(5, false);

//                 // this function is defined in VisionDebuggerInterface.hpp
//                 // We again scale the coordinate points so that they match up with the coordinate system used by the painter
//                 if(currRegion.isTopCamera())
//                     myPainterTop->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::RED);
//                 else
//                     myPainterBot->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::RED);
//             }
//         }
//     }
// 	return;
// }


// void RobotDetectorDebugger::highlightCandidates(std::vector<Cluster>& candidates) {
//     //first check that vision debugger module exists (this is defined elsewhere so you shouldn't need to worry about it)
//     if(vdm != NULL) {

//         // create a query object to access vatnao UI information
//         VisionDebugQuery myQ = vdm->getQuery();

//         // check whether the 'tick box' is selected for showing curved regions
//         if (myQ.options["Show candidate robots"] == "true") {
//             for(size_t i = 0; i < candidates.size(); ++i) {
//                 Cluster currCandidate = candidates[i];
//                 // get the bounding box of the region
//                 BBox boundingBox = currCandidate.box_;

//                 // Prety sure the painter function starts from the top left of the region
//                 int topLeftX = boundingBox.a.x();
//                 int topLeftY = boundingBox.a.y();

//                 // need to create a vatnao painter object which is our interface into the vatnao display frame
//                 // 5 defines the factor of downsampling for the painter. true = top camera, false = bottom
//                 // We want to downsample the image because that means we will get thicker lines drawn by vatnao
//                 VisionPainter *myPainterTop = vdm->getFrameOverlayPainter(5, true);
//                 VisionPainter *myPainterBot = vdm->getFrameOverlayPainter(5, false);

//                 // this function is defined in VisionDebuggerInterface.hpp
//                 // We again scale the coordinate points so that they match up with the coordinate system used by the painter
// 			if(currCandidate.isTopCamera())
//                     myPainterTop->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::PINK);
//                 else
//                     myPainterBot->drawRect(topLeftX/5, topLeftY/5, boundingBox.width()/5, boundingBox.height()/5,  VisionPainter::PINK);
//             }
//         }
//     }
// 	return;
// }


// void RobotDetectorDebugger::showSaliencyImage(const RegionI& region) {
//     if (vdm != NULL) {
//         VisionDebugQuery myQ = vdm->getQuery();
//         VisionPainter *p = vdm->getGivenRegionOverlayPainter(region);
//         if (myQ.options["Show whole saliency image"] == "true") {
//             p->drawColourSaliency(region);
//         }
//     }
// }

// void RobotDetectorDebugger::outputToFile(std::vector<Cluster>& candidates) {
//     if(vdm != NULL) {
//         std::string dataFileName_ = vdm->getFilename();
//         // If we haven't written the header row then we need to do that first
//         // This will also clear the existing file
//         if(!dataHeaderWritten_) {
//             std::ofstream dataFile;
//             dataFile.open(dataFileName_.c_str());
//             dataFile << "FrameNum, x, y, isTopCamera, height, width, whites, numRegions, classification, confidence, realRobot\n";
//             dataFile.close();
//             dataHeaderWritten_ = true;
//         }

//         unsigned int currFrame = vdm->getCurrentFrame();
//         if(framesWritten_.count(currFrame) != 0)
//             return;
//         std::ofstream dataFile;
//         dataFile.open(dataFileName_.c_str(), std::ios::app);
//         for(size_t i = 0; i < candidates.size(); ++i) {
//             Point foot = candidates[i].getBaseCenter();
//             std::stringstream myLine;
//             myLine << currFrame;
//             myLine << "," << foot[0];
//             myLine << "," << foot[1];
//             myLine << "," << candidates[i].isTopCamera();
//             myLine << "," << candidates[i].box_.height();
//             myLine << "," << candidates[i].box_.width();
//             myLine << "," << candidates[i].whitePixels_;
//             myLine << "," << candidates[i].regions_.size();
//             myLine << "," << candidates[i].isRobot_;
//             myLine << "," << candidates[i].confidence_;
//             myLine << "," << 0;

//             dataFile << myLine.str() <<"\n";
//         }
//         dataFile.close();
//         framesWritten_[currFrame] = true;
//     }
//     return;
// }

// void RobotDetectorDebugger::outputTimings(std::vector<float>& timings, int numRegions, int numCandidates) {
//     if(vdm != NULL) {
//         std::string dataFileName_ = vdm->getFilename();
//         dataFileName_.resize(dataFileName_.length()-8);
//         dataFileName_ += "timings.csv";
//         // If we haven't written the header row then we need to do that first
//         // This will also clear the existing file
//         if(!dataHeaderWrittenTimings_) {
//             std::ofstream dataFile;
//             dataFile.open(dataFileName_.c_str());
//             dataFile << "FrameNum, numRegions, numCandidates, thresholding, colorRoi, clustering, classification, total\n";
//             dataFile.close();
//             dataHeaderWrittenTimings_ = true;
//             // HACK first classification value is f**cked. so hard code it
//             // so that the results aren't effed
//             timings[3] = 15;
//         }

//         unsigned int currFrame = vdm->getCurrentFrame();
//         if(framesWritten_.count(currFrame) != 0)
//             return;
//         std::ofstream dataFile;
//         dataFile.open(dataFileName_.c_str(), std::ios::app);
//         std::stringstream myLine;
//         myLine << currFrame;
//         myLine << "," << numRegions;
//         myLine << "," << numCandidates;
//         myLine << "," << timings[0];
//         myLine << "," << timings[1];
//         myLine << "," << timings[2];
//         myLine << "," << timings[3];
//         myLine << "," << (timings[0]+timings[1]+timings[2]+timings[3]);

//         dataFile << myLine.str() <<"\n";
//         dataFile.close();

//     }
//     return;

// }
// #endif

//-------------------------------- Cluster ----------------------
Cluster::Cluster() :
    box_(BBox(Point(0,0), Point(0,0))), whitePixels_(0), isRobot_(false) {};

void Cluster::addRegionToCluster(const RegionI& region, int whiteCount) {
    bool firstRegion = regions_.size() == 0 ? true : false;
    regions_.push_back(region);
    BBox bb = region.getBoundingBoxRaw();
    if (box_.a.y() > bb.a.y() || firstRegion) {
        box_.a.y() = bb.a.y();
    }
    if (box_.b.y() < bb.b.y() || firstRegion) {
        box_.b.y() = bb.b.y();
    }
    if (box_.a.x() > bb.a.x() || firstRegion){
        box_.a.x() = bb.a.x();
    }
    if (box_.b.x() < bb.b.x() || firstRegion) {
        box_.b.x() = bb.b.x();
    }
    //whitePixels_ += whiteCount;
};

bool Cluster::overlapsRegion(RegionI& region) {
    BBox rBox = region.getBoundingBoxRaw();
    if((region.isTopCamera() && isTopCamera()) ||
        (!region.isTopCamera() && !isTopCamera()))    {
        if(std::max(box_.a.x(), rBox.a.x()) <= std::min(box_.b.x(), rBox.b.x())
            && std::max(box_.a.y(), rBox.a.y()) <= std::min(box_.b.y(), rBox.b.y())) {
                return true;
        }
    }
    return false;
}

Point Cluster::getBaseCenter() {
    return Point(box_.a.x() + box_.width()/2, box_.b.y());
}
