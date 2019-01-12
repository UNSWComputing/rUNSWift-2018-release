#include "perception/vision/VisionAdapter.hpp"

#include "perception/vision/other/VarianceCalculator.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sys/time.h>        /* For gettimeofday */
#include <pthread.h>
#include <vector>

#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "types/LastSecondInfo.hpp"
#include "utils/Timer.hpp"
#include "types/CombinedCameraSettings.hpp"
#include "types/CombinedFrame.hpp"

using namespace std;
using namespace boost::algorithm;


VisionAdapter::VisionAdapter(Blackboard *bb)
   : Adapter(bb),
     vision_((blackboard->config)["vision.run_colour_calibration"].as<bool>(),
             (blackboard->config)["vision.load_nnmc"].as<bool>(),
             (blackboard->config)["vision.save_nnmc"].as<bool>())
{
    combined_camera_ = new CombinedCamera(
        (blackboard->config)["vision.dumpframes"].as<bool>(),
        (blackboard->config)["vision.dumprate"].as<int>(),
        (blackboard->config)["vision.dumpfile"].as<string>()
    );

    combined_frame_ = boost::shared_ptr<CombinedFrame>();

    RegionI topRegion = vision_.getFullRegionTop();
    RegionI botRegion = vision_.getFullRegionBot();
    const Fovea *foveaTop = topRegion.getInternalFovea();
    const Fovea *foveaBot = botRegion.getInternalFovea();

    writeTo(vision, topSaliency, (Colour*)foveaTop->getInternalColour());
    writeTo(vision, botSaliency, (Colour*)foveaBot->getInternalColour());
    lastSecond.reset();
}

void VisionAdapter::tick() {
    Timer t;
    uint32_t time;

    /*
     * Camera Tick
     */
    llog_open(VERBOSE) << "Vision Camera Tick" << endl;
    t.restart();
    tickCamera();
    time = t.elapsed_us();
    if (time > 30000) {
        llog_close(VERBOSE) << "Vision Camera Tick: OK " << time << " us" << endl;
    } else {
        llog_close(ERROR) << "Vision Camera Tick: TOO LONG " << time << " us" << endl;
    }

    /*
     * Process Tick
     */
    llog_open(VERBOSE) << "Vision Process Tick" << endl;
    t.restart();
    tickProcess();
    time = t.elapsed_us();
    if (time > 30000) {
        llog_close(VERBOSE) << "Vision Process Tick: OK " << time << " us" << endl;
    } else {
        llog_close(ERROR) << "Vision Process Tick: TOO LONG " << time << " us" << endl;
    }

}

void VisionAdapter::tickCamera() {
    if (combined_camera_ == NULL) {
        llog_middle(WARNING) << "No Camera provided to the VisionAdapter" << endl;
    } else {
         writeTo(vision, topFrame, combined_camera_->getFrameTop());
         writeTo(vision, botFrame, combined_camera_->getFrameBottom());

         // Write the camera settings to the Blackboard
         // for syncing with OffNao's camera tab
         CombinedCameraSettings settings = combined_camera_->getCameraSettings();
         writeTo(vision, topCameraSettings, settings.top_camera_settings);
         writeTo(vision, botCameraSettings, settings.bot_camera_settings);
    }
}

void VisionAdapter::tickProcess() {
    Timer t;
    VisionInfoIn info_in;

    int behaviourReadBuf = readFrom(behaviour, readBuf);

    // Used by the Python WallTimer.py
    // Might not belong here, just quick fixing Ready skill for now.
    struct timeval tv;
    gettimeofday(&tv, 0);

    int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

    // TODO Read current Pose from blakboard
    conv_rr_.pose = readFrom(motion, pose);
    conv_rr_.updateAngles(readFrom(kinematics, sensorsLagged));

    info_in.cameraToRR = conv_rr_;
    info_in.pose = conv_rr_.pose;
    info_in.currentRole = readFrom(behaviour, request[behaviourReadBuf]).currentRole;
    info_in.behaviour_crazy_ball_override = readFrom(behaviour, request[0].wantCrazyBall);

    // Determine if the robot is in ready mode
    info_in.game_state = readFrom(gameController, gameState);

    // Set latestAngleX.
    info_in.latestAngleX =
              readFrom(motion, sensors).sensors[Sensors::InertialSensor_AngleX];

    // Store action commands
    info_in.active_action = readFrom(motion, active);
    info_in.my_player_number = readFrom(gameController, player_number);

    // TODO ADD THIS IN
    conv_rr_.findEndScanValues();

    /*
     * Acquire blackboard lock
     */
    acquireLock(serialization);
    llog_middle(VERBOSE) << "Vision tickProcess acquireLock(serialization) took " << t.elapsed_us()
      << " us" << endl;
    t.restart();

    /*
     * Reading from Blackboard into VisionBlackboardInfo
     */
    // NOTE: You can add things to info_in below by going
    // NOTE: info_in.robotDetection.sonar = readFrom(kinematics, sonarFiltered)

    info_in.top_camera_settings = readFrom(vision, topCameraSettings);
    info_in.bot_camera_settings = readFrom(vision, botCameraSettings);
    info_in.robot_pos = readFrom(localisation, robotPos);

    boost::shared_ptr<CombinedFrame> combined_frame_;
    combined_frame_ = boost::shared_ptr<CombinedFrame>(new CombinedFrame(
        readFrom(vision, topFrame),
        readFrom(vision, botFrame),
        conv_rr_,
        combined_frame_
    ));

    llog_middle(VERBOSE) << "Vision reading images from blackboard took " << t.elapsed_us()
      << " us" << endl;

    lastSecond = readFrom(vision, lastSecond);

    t.restart();

    /*
     * Running Process Frame
     */
    llog(VERBOSE) << "Vision reading remaining data from blackboard took " << t.elapsed_us() << " us" << endl;
    t.restart();

    pthread_yield();
    usleep(1); // force sleep incase yield sucks

    VisionInfoOut info_out = vision_.processFrame(*(combined_frame_.get()), info_in);

    llog(VERBOSE) << "Vision processFrame() took " << t.elapsed_us() << " us" << endl;
    t.restart();

    /*
     * Writing Results back to blackboard
     */

    // NOTE: You can add things back to the blackboard by going
    // NOTE: writeTo(vision, [blackboard var name], info_out.[info_in var name])

    // Vision->Localisation Log: collect info to lastSecond
    /*
    if (vision_timestamp - lastSecond.initial_timestamp > 1000000) {
      lastSecond.reset();
      lastSecond.initial_timestamp = vision_timestamp;
    }

    std::vector<BallInfo>::iterator bI;
    for (bI = info_out.balls.begin(); bI != info_out.balls.end(); bI++) {

          // Initialise the min if 0
      if (lastSecond.next_min_balls_distance == 0)
       lastSecond.next_min_balls_distance = bI->rr.distance();

    if (bI->rr.distance() < lastSecond.next_min_balls_distance)
       lastSecond.next_min_balls_distance = bI->rr.distance();
    lastSecond.next_avg_balls_distance += bI->rr.distance();
    if (bI->rr.distance() > lastSecond.next_max_balls_distance)
       lastSecond.next_max_balls_distance = bI->rr.distance();
    }
    lastSecond.next_num_balls += info_out.balls.size();

    std::vector<PostInfo>::iterator pI;
    for (pI = info_out.posts.begin(); pI != info_out.posts.end(); pI++) {

          // Initialise the mins if 0
      if (lastSecond.next_min_posts_kDistance == 0)
       lastSecond.next_min_posts_kDistance = pI->kDistance;

    if (lastSecond.next_min_posts_wDistance == 0)
       lastSecond.next_min_posts_wDistance = pI->wDistance;

          // Update kDistance
    if (pI->kDistance < lastSecond.next_min_posts_kDistance)
       lastSecond.next_min_posts_kDistance = pI->kDistance;
    lastSecond.next_avg_posts_kDistance += pI->kDistance;
    if (pI->kDistance > lastSecond.next_max_posts_kDistance)
       lastSecond.next_max_posts_kDistance = pI->kDistance;

          // Update wDistance
    if (pI->wDistance < lastSecond.next_min_posts_wDistance)
       lastSecond.next_min_posts_wDistance = pI->wDistance;
    lastSecond.next_avg_posts_wDistance += pI->wDistance;
    if (pI->wDistance > lastSecond.next_max_posts_wDistance)
       lastSecond.next_max_posts_wDistance = pI->wDistance;
    }
    lastSecond.next_num_posts += info_out.posts.size();
    lastSecond.next_num_robots += info_out.robots.size();
    std::vector<FieldFeatureInfo>::iterator it;
    for (it = info_out.features.begin(); it != info_out.features.end(); it++) {
        switch (it->type) {
            case FieldFeatureInfo::fLine:
                lastSecond.next_num_lines++;
                break;

            case FieldFeatureInfo::fCorner:
                if (lastSecond.next_min_corners_distance == 0)
                    lastSecond.next_min_corners_distance = it->distance();

                if (it->distance() < lastSecond.next_min_corners_distance)
                    lastSecond.next_min_corners_distance = it->distance();
                lastSecond.next_avg_corners_distance += it->distance();
                if (it->distance() > lastSecond.next_max_corners_distance)
                    lastSecond.next_max_corners_distance = it->distance();

                lastSecond.next_num_corners++;

                break;

            case FieldFeatureInfo::fTJunction:
                if (lastSecond.next_min_t_junctions_distance == 0)
                    lastSecond.next_min_t_junctions_distance = it->distance();

                if (it->distance() < lastSecond.next_min_t_junctions_distance)
                    lastSecond.next_min_t_junctions_distance = it->distance();
                lastSecond.next_avg_t_junctions_distance += it->distance();
                if (it->distance() > lastSecond.next_max_t_junctions_distance)
                    lastSecond.next_max_t_junctions_distance = it->distance();

                lastSecond.next_num_t_junctions++;
                break;
            case FieldFeatureInfo::fPenaltySpot:
                lastSecond.next_num_penalty_spots++;
                break;
            case FieldFeatureInfo::fCentreCircle:
                lastSecond.next_num_centre_circles++;
                break;
            case FieldFeatureInfo::fFieldLinePoint:
                lastSecond.next_num_field_line_points++;
                break;
            case FieldFeatureInfo::fXJunction:
                break;
            case FieldFeatureInfo::fParallelLines:
                break;
            default: // fNone
                break;
        }
    }
    lastSecond.next_num_frames++;
    */


    // Calculate covariance matrices for observations
    VarianceCalculator::setVariance(info_out.balls);
    VarianceCalculator::setVariance(info_out.posts);
    VarianceCalculator::setVariance(info_out.features);

    writeTo (vision, timestamp,       vision_timestamp        );
    // Note that these regions will not be able to access their underlying pixel
    // data.
    writeTo (vision, regions,         info_out.regions        );
    writeTo (vision, balls,           info_out.balls          );
    writeTo (vision, uncertain_balls, info_out.uncertain_balls);
    writeTo (vision, posts,           info_out.posts          );
    writeTo (vision, robots,          info_out.robots         );
    writeTo (vision, fieldFeatures,   info_out.features       );
    writeTo (vision, lastSecond,      lastSecond              );
    writeTo (vision, fieldBoundaries, info_out.boundaries     );

    releaseLock(serialization);
    llog(VERBOSE) << "Vision writing back to blackboard took " << t.elapsed_us() << " us" << endl;
}
