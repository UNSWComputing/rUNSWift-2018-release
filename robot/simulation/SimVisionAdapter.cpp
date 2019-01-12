#include "SimVisionAdapter.hpp"
#include "Observation.hpp"

#include "blackboard/Blackboard.hpp"

#include "types/Ipoint.hpp"
#include "types/FootInfo.hpp"
#include "perception/vision/camera/CombinedCamera.hpp"
#include "types/CombinedCameraSettings.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/other/VarianceCalculator.hpp"

#include <utility>

namespace Simulation {

    SimVisionAdapter::SimVisionAdapter(Blackboard* bb)
        : Adapter(bb)
    {
        // TODO @jez find a more elegant solution than this workaround
        //
        // Here we write an initial vision timestamp on construction.
        //
        // The problem is that we don't start writing to the blackboard
        // until the simulation connection is established. While we are
        // connecting we wait 2 seconds for the robot to settle (it falls
        // from the sky).
        //
        // During this 2 seconds, localisation is reading from the blackboard
        // and its reading the default vision timestamp of 0. As such, when we
        // write the first actual vision timestamp it's huge increase from 0 and
        // localisation thinks we've passed hours (?) of time and all of our
        // estimations become very uncertain due to decay (stdvar is MASSIVE).
        //
        // Workaround: write intial vision timestamp before connection takes
        // place.
        acquireLock(serialization);
        struct timeval tv;
        gettimeofday(&tv, 0);
        int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;
        writeTo(vision, timestamp, vision_timestamp);
        releaseLock(serialization);
    }

    RRCoord SimVisionAdapter::getRR(Observation& o, double head_yaw_degrees)
    {
        // o->pol[0] is distance from the torso. We want to find the distance
        // from the head. o->pol[1] is the absolute heading from the robot. We
        // want to find the heading relative to the yaw angle.
        //
        // NOTE: o->pol[2] is the vertical angle, which is not used.
        // TODO consider using vertical angle

        // Use Pythagorean Theorem to find distance from feet
        // b^2 = c^2 - a^2
        //
        // TODO tweak this
        double a = 480;                           // NAO height is about 580mm and camera is in middle of torso
                                                  // Decreasing this brings dist estimations closer, increasing
                                                  // pushes dist estimations further away.
        double c = o.pol[0] * 1000;               // Observation distance from HEAD in mm
        double b_squared = c*c - a*a;             // Derive distance from FEET squared

        double dist = sqrt(b_squared);            // Take square root

        // The value for 'a' is derived from intuition, and so inaccuracies
        // may result in the distance from feet being NaN (i.e. if b_squared
        // is negative). In such a case, we are really close to the ball,
        // so we can just change the distance to be a close value.
        if (isnan(dist))
        {
            dist = 0;
        }

        // Find heading using heading of ball from camera plus head yaw
        double heading = DEG2RAD(o.pol[1] + head_yaw_degrees);       // Looks like sim is in degrees, RR uses radians

        return RRCoord(dist, heading);
    }

    BallInfo SimVisionAdapter::handleBall(Observation& o, RRCoord rr)
    {
        // Behaviours moves the head depending on the position of the ball
        // in the image. So when we see the ball we have to simulate its
        // spot in the camera frame.
        //
        // Move to right spot in image (at least X coord) (640 is width)
        // We want this to be around 320
        // NOTE image coords depend on camera selection (top/bottom)
        Point image_coords((640/2.0) - o.pol[1]*10, 1356.0);

        BallInfo ball(rr, 5, image_coords, XYZ_Coord(195.713, -14.9154, -387.146));
        ball.topCamera = 0;
        ball.visionVar = -1.0f;
        return ball;
    }

    PostInfo SimVisionAdapter::handlePost(Observation& o, RRCoord rr)
    {
        PostInfo::Type t;
        // The simulator identifies the exact post we are observing, but rUNSWift
        // has been refactored to ingest just left/right posts since the
        // introduction of the white posts.
        switch (o.sub_type)
        {
           case Observation::G1_L:
                t = PostInfo::pRight;
                break;
            case Observation::G2_L:
                t = PostInfo::pLeft;
                break;
            case Observation::G1_R:
                t = PostInfo::pLeft;
                break;
            case Observation::G2_R:
                t = PostInfo::pRight;
                break;
             default:
                t = PostInfo::pNone;
                break;
        }

        return PostInfo(rr, t, BBox(Point(0,0),Point(0,0)), 0, 0, true, PostInfo::pUnknown);
    }

    FieldFeatureInfo SimVisionAdapter::handleFieldFeature(Observation& o, RRCoord rr, double head_yaw)
    {
        AbsCoord robot_pos = readFrom(localisation, robotPos);
        rr.orientation() = robot_pos.theta() + DEG2RAD(head_yaw);

        if (o.sub_type == Observation::CTR)    // Centre circle
        {
            return FieldFeatureInfo(rr, CentreCircleInfo());
        }
        else     // Corner flag
        {
            double target_orientation = 0;
            switch(o.sub_type)
            {
                case Observation::F1_L:       // Top left corner (bird's eye view)
                    target_orientation += -M_PI_4 - M_PI_2;
                    break;
                case Observation::F1_R:       // Top right corner
                    target_orientation += -M_PI_4;
                    break;
                case Observation::F2_L:       // Bottom left corner
                    target_orientation += M_PI_4 + M_PI_2;
                    break;
                case Observation::F2_R:       // Bottom right corner
                    target_orientation += M_PI_4;
                    break;
                default:
                    break;
            }
            rr.orientation() += target_orientation;

            return FieldFeatureInfo(rr, CornerInfo());
        }
    }

    RobotInfo handleRobot(Observation& o, RRCoord rr)
    {
        // TODO Not yet implemented
        return RobotInfo();
    }

    void SimVisionAdapter::findGoalSide(std::vector<PostInfo>& posts)
    {
        // Make sure we have 2 and only 2 posts.
        if (posts.size() != 2)
        {
            return;
        }

        // Given there are only two posts, we can use a boolean to find the index.
        // If the second post (index 1) is the left post, the boolean will evaluate
        // to true and give us the index 1. Otherwise, if the second post (index 1)
        // is not the left post, the boolean will evaluate to false and give us the
        // index 0 which we can then assume is the left post.
        int left_idx = (posts[1].type == PostInfo::pLeft);

        // If left_idx contains the index of the left post, we can use XOR to find
        // the index of the right post (either 0 or 1)
        int right_idx = left_idx ^ 1;

        // Double check the post types are correct (we might have two left or two
        // right posts [or an unknown post]).
        if (posts[left_idx].type != PostInfo::pLeft ||
            posts[right_idx].type != PostInfo::pRight)
        {
            return;
        }

        // Find the relative heading from the robots camera
        PostInfo& left = posts[left_idx];
        PostInfo& right = posts[right_idx];
        double left_dist = left.rr.distance();
        double right_dist = right.rr.distance();

        // If the left post is closer than the right, we're on the left hand side.
        // If the right post is closer than the left, we're on the right hand side.
        PostInfo::Direction d = PostInfo::pUnknown;
        if (left_dist < right_dist)
        {
            d = PostInfo::pToLeftOf;
        }
        else
        {
            d = PostInfo::pToRightOf;
        }
        left.dir = d;
        right.dir = d;
    }

    void SimVisionAdapter::tick(const PerceptorInfo& perceived)
    {
        acquireLock(serialization);
        struct timeval tv;
        gettimeofday(&tv, 0);
        int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

        double head_yaw = perceived.joints.hj1;

        // Do camera frame
        uint8_t frame[4096] = {0};
        for (int i=0; i < 4096; ++i)
        {
            frame[i] = i;
        }
        writeTo(vision, topFrame, (const uint8_t*)frame);
        writeTo(vision, botFrame, (const uint8_t*)frame);

        // Do saliency image
        // TODO @jez separate in to top and bottom
        // TODO @jez put something funny in here
        const size_t s_num = TOP_SALIENCY_ROWS*TOP_SALIENCY_COLS;
        Colour saliency[s_num * sizeof(Colour)];
        for (unsigned int i=0U; i < s_num; ++i)
        {
            saliency[i] = (Colour)(i % cNUM_COLOURS);
                         //cBLACK;
                         //(Colour)(rand() ^ cNUM_COLOURS);
        }
        writeTo(vision, topSaliency, (Colour*)&saliency);
        writeTo(vision, botSaliency, (Colour*)&saliency);
        writeTo(vision, saliency, (Colour*)&saliency);

        CombinedCameraSettings settings;
        writeTo(vision, topCameraSettings, settings.top_camera_settings);
        writeTo(vision, botCameraSettings, settings.bot_camera_settings);

        // Do vision info
        std::vector<BallInfo> sim_balls;
        std::vector<PostInfo> sim_posts;
        std::vector<FieldFeatureInfo> sim_field_features;

        // Handle observations
        for (unsigned int i=0; i < perceived.observations.size(); ++i)
        {
            Observation o = perceived.observations[i];
            RRCoord rr = getRR(o, head_yaw);

            if (o.type == Observation::BALL)
            {
                if (rr.distance() <= BALL_DETECTION_MAX_DISTANCE 
                    && o.pol[1] <= BALL_DETECTION_MAX_HEADING
                    && o.pol[1] >= -BALL_DETECTION_MAX_HEADING)
                {
                    sim_balls.push_back(handleBall(o, rr));
                }
            }
            else if (o.type == Observation::GOAL)
            {
                sim_posts.push_back(handlePost(o, rr));
            }
            else if (o.type == Observation::FLAG)
            {
                // Stop getting locked on corners
                //
                // TODO debug tweak / fix this.
                // Can fix the corner problem by changing them from one flag to
                // two flags on either side of corner. We can find orientation
                // of the corner by finding the difference in the heading of
                // each flag.
                if (o.sub_type == Observation::CTR || rr.distance() > 2000)
                {
                    sim_field_features.push_back(handleFieldFeature(o, rr, head_yaw));
                }
            }
        }

        // Find which side (left/right) of the goal we are standing on
        findGoalSide(sim_posts);

        // TODO consider SimVarianceCalculator
        VarianceCalculator::setVariance(sim_balls);
        VarianceCalculator::setVariance(sim_posts);
        VarianceCalculator::setVariance(sim_field_features);

        writeTo(vision, timestamp, vision_timestamp);

        // TODO debug
        writeTo(vision, balls,          sim_balls);
        writeTo(vision, posts,          sim_posts);
        writeTo(vision, fieldFeatures,  sim_field_features);
        //writeTo(vision, fieldBoundaries,std::vector<FieldBoundaryInfo>());
        //writeTo(vision, robots, simRobots);
        releaseLock(serialization);

    }

};
