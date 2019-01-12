#ifndef PERCEPTION_VISION_VISION_H_
#define PERCEPTION_VISION_VISION_H_

#include <list>

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/colour/ColourClassifierInterface.hpp"
#include "perception/vision/detector/DetectorInterface.hpp"
#include "perception/vision/fullfinder/FullFinderInterface.hpp"
#include "perception/vision/regionfinder/RegionFinderInterface.hpp"
#include "perception/vision/middleinfoprocessor/MiddleInfoProcessorInterface.hpp"
#include "gamecontroller/GameController.hpp"
#include "types/VisionInfoIn.hpp"
#include "types/VisionInfoMiddle.hpp"
#include "types/VisionInfoOut.hpp"
#include "types/CombinedFovea.hpp"
#include "types/CombinedFrame.hpp"
#include "utils/Timer.hpp"

class Vision {
friend class CalibrationTab;

public:

    /**
     * Default constructor for Vision module
     */
    Vision(bool run_colour_calibration,
        bool load_nnmc,
        bool save_nnmc);

    /**
     * Destructor for Vision module
     */
    ~Vision();

    /**
     * Process a frame, running appropriate finders and detectors.
     * Vision logic mainly happens here.
     */
    VisionInfoOut processFrame(const CombinedFrame& pixel_data, const VisionInfoIn& info_in);

    inline const RegionI& getFullRegionTop() { return full_region_top_; }
    inline const RegionI& getFullRegionBot() { return full_region_bot_; }

    /**
     * get FieldFeature Detector for VisionAdapter to pass in robotPos
     */

    // For offnao

    void saveNnmc(bool top, std::string filename);
    void loadNnmc(bool top, std::string filename);
    void resetNnmc(bool top);
    void runNnmc(bool top, const uint8_t *frame, int stepsize);
    void runFillPoints(bool top, bool isGreen);

private:

    void setupAlgorithms_();
    void runAlgorithms_();

    void addMiddleInfoProcessor_(uint32_t, MiddleInfoProcessor*);
    MiddleInfoProcessor* getMiddleInfoProcessor_(uint32_t);
    void runMiddleInfoProcessor_(uint32_t);

    void addDetector_(uint32_t, Detector*);
    Detector* getDetector_(uint32_t);
    void runDetector_(uint32_t);

    /**
     * Class member variables
     */
    std::list<CombinedFovea> full_foveae_;

    ColourClassifier* colour_classifier_top_;
    ColourClassifier* colour_classifier_bot_;

    Detector** detectors_;
    MiddleInfoProcessor** middle_info_processors_;

    VisionInfoIn info_in_;
    VisionInfoMiddle info_middle_;
    VisionInfoOut info_out_;


    // For colour classification
    int images_sampled_;

    // Bounding boxes for full regions
    BBox bbox_top_;
    BBox bbox_bot_;

    CombinedFovea combined_fovea_;

    // Full Regions
    RegionI full_region_top_;
    RegionI full_region_bot_;

    // Keeps track of run times for average output.
    int frameCount;
    int DCCTime;
    int foveaTime;
    int fieldFeaturesTime;
    int regionFinderTime;
    int ballDetectorTime;
    int robotDetectorTime;
};

#endif
