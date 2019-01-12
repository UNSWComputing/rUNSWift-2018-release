#ifndef PERCEPTION_VISION_VISIONADAPTER_H_
#define PERCEPTION_VISION_VISIONADAPTER_H_

#include "perception/vision/camera/CombinedCamera.hpp"
#include "perception/vision/camera/CameraToRR.hpp"
#include "perception/vision/Vision.hpp"

#include "utils/Timer.hpp"
#include "blackboard/Adapter.hpp"
#include "blackboard/Blackboard.hpp"

class VisionAdapter : Adapter {
friend class AppAdaptor;
public:

    /**
     * Constructor for VisionAdapter which bridges Vision and VisionBlackboard
     */
    VisionAdapter(Blackboard *bb);

    /**
     * Destructor for VisionAdapter
     */
    ~VisionAdapter() {}

    /**
     * tick Called to get and process each frame
     */
    void tick();
    /* Method for writing robot input onto the blackboard */
    void tickCamera();
    /* Method for processing what is on the blackboard and writing resuts */
    void tickProcess();

    // TODO: Temporary fix please resolve
	CombinedCamera *combined_camera_;
private:

    boost::shared_ptr<CombinedFrame> combined_frame_;
    CameraToRR conv_rr_;
    Vision vision_;

   /* Collection of info from frames from the last second */
    LastSecondInfo lastSecond;
};

#endif
