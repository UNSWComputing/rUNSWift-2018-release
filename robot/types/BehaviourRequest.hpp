#pragma once

#include "types/ActionCommand.hpp"
#include "types/AbsCoord.hpp"
#include "perception/vision/camera/CameraDefinitions.hpp"

class BehaviourRequest {
   public:
      WhichCamera whichCamera;
      ActionCommand::All actions;
      bool goalieAttacking;

      bool goalieDiving;

      // Number of seconds since the robot last kicked the ball
      int secondsSinceLastKick;
      
      // Whether the robot is in the process of lining up with the ball to kick it.
      bool doingBallLineUp;
      
      // If the behaviour is currently executing the ready mode behaviour.
      bool isInReadyMode;
      
      // The estimated time (in seconds) that this robot needs to get to the ball. This is 
      // the primary method that we use to decide which robot should be the Striker.
      float timeToReachBall;

      // Estimated time to different positions, used for role assignment
      float timeToReachDefender;
      float timeToReachMidfielder;
      float timeToReachUpfielder;

      int kickoffSide;

      // A flag to tell vision we want crazy balls
      bool wantCrazyBall;

      // Behaviour hierarchy
      std::string behaviourHierarchy;
      
      // The encoded enum value for the robots current role.
      int currentRole;

      // The encoded enum value for the robots role
      int role;

      // Whether the robot is playing the ball or not
      bool playingBall;

      // Whether the robot needs assistance
      bool needAssistance;

      // Whether the robot is assisting
      bool isAssisting;

      bool isKickedOff;

      bool isFollowing;

      // Where we are currently aiming to kick to
      //int kickingTo;

      // TODO: this is horrible, I just cant figure out how to pass back a list/vector/array from
      // python back to C++.
      int readyPositionAllocation0;
      int readyPositionAllocation1;
      int readyPositionAllocation2;
      int readyPositionAllocation3;
      int readyPositionAllocation4;

      BehaviourRequest() {
         whichCamera = TOP_CAMERA;
         goalieAttacking = false;
         goalieDiving = false;
         secondsSinceLastKick = -1;
         doingBallLineUp = false;
         isInReadyMode = false;
         isFollowing = false;
         timeToReachBall = 10000.0f;
         timeToReachDefender = 10000.0f;
         timeToReachMidfielder = 10000.0f;
         timeToReachUpfielder= 10000.0f;
         kickoffSide = 0;
         currentRole = 0;
         role = 0;
         playingBall = false;
         needAssistance = false;
         isAssisting = false;
         isKickedOff = false;
         wantCrazyBall = false;
         behaviourHierarchy = "";
         
         readyPositionAllocation0 = -1;
         readyPositionAllocation1 = -1;
         readyPositionAllocation2 = -1;
         readyPositionAllocation3 = -1;
         readyPositionAllocation4 = -1;
      };

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & whichCamera;
         ar & actions;
         ar & goalieAttacking;
	   ar & goalieDiving;
         ar & doingBallLineUp;
         ar & isInReadyMode;
         ar & timeToReachBall;
         ar & timeToReachDefender;
         ar & timeToReachMidfielder;
         ar & timeToReachUpfielder;
         ar & kickoffSide;
         ar & currentRole;

         if (file_version >=1)
         {
            ar & role;
            ar & playingBall;
            ar & needAssistance;
            ar & isAssisting;
            ar & secondsSinceLastKick;
         }

         if (file_version >= 2)
         {
            ar & isFollowing;
         }

         if (file_version >= 3)
         {
            ar & isKickedOff;
         }

         ar & behaviourHierarchy;
         ar & readyPositionAllocation0;
         ar & readyPositionAllocation1;
         ar & readyPositionAllocation2;
         ar & readyPositionAllocation3;
         ar & readyPositionAllocation4;
      }
};

BOOST_CLASS_VERSION(BehaviourRequest, 3);