/*
* BehaviourSharedData.hpp
*
*  Created on: 15/05/2014
*      Author: osushkov
*/

#pragma once

#include "types/boostSerializationEigenTypes.hpp"
#include "perception/behaviour/ReadySkillPositionAllocation.hpp"

class BehaviourSharedData {
public:

    BehaviourSharedData() :
    goalieAttacking(false),
    goalieDiving(false),
    secondsSinceLastKick(-1),
    timeToReachBall(10000.0f),
    timeToReachUpfielder(10000.0f),
    timeToReachMidfielder(10000.0f),
    timeToReachDefender(10000.0f),
    kickoffSide(0),
    currentRole(0),
    role(0),
    playingBall(false),
    needAssistance(false),
    isAssisting(false),
    isKickedOff(false),
    isFollowing(false),
    doingBallLineUp(false),
    isInReadyMode(false) {}

    bool goalieAttacking;

    bool goalieDiving;

    // Number of seconds since the robot last kicked the ball
    int secondsSinceLastKick;

    // The estimated time (in seconds) that this robot needs to get to the ball. This is 
    // the primary method that we use to decide which robot should be the Striker.
    float timeToReachBall;

    // Times to reach particular positions - used to role assignment
    float timeToReachUpfielder;
    float timeToReachMidfielder;
    float timeToReachDefender;

    int kickoffSide;

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

    // Whether the robot is in the process of lining up with the ball to kick it.
    bool doingBallLineUp;

    bool isInReadyMode;

    // What this robot thinks the ready position allocation should be.
    ReadySkillPositionAllocation readyPositionAllocation;

    bool sanityCheck()
    {
        // Check for secondsSinceLastKick nan
        if (isnan(secondsSinceLastKick)){
            std::cout << "received nan for secondsSinceLastKick" << std::endl;
            return false;
        }

        // Check for timeToReachBall nan
        if (isnan(timeToReachBall)){
            std::cout << "received nan for timeToReachBall" << std::endl;
            return false;
        }

        // Check for timeToReachUpfielder nan
        if (isnan(timeToReachUpfielder)){
            std::cout << "received nan for timeToReachUpfielder" << std::endl;
            return false;
        }

        // Check for timeToReachMidfielder nan
        if (isnan(timeToReachMidfielder)){
            std::cout << "received nan for timeToReachMidfielder" << std::endl;
            return false;
        }

        // Check for timeToReachDefender nan
        if (isnan(timeToReachDefender)){
            std::cout << "received nan for timeToReachDefender" << std::endl;
            return false;
        }

        // Check for kickoffSide nan
        if (isnan(kickoffSide)){
            std::cout << "received nan for kickoffSide" << std::endl;
            return false;
        }

        // Check for currentRole nan
        if (isnan(currentRole)){
            std::cout << "received nan for currentRole" << std::endl;
            return false;
        }

        // Check for role nan
        if (isnan(role)){
            std::cout << "received nan for role" << std::endl;
            return false;
        }

        return true;
    }

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        ar & goalieAttacking;
        ar & goalieDiving;
        ar & timeToReachBall;
        ar & timeToReachUpfielder;
        ar & timeToReachMidfielder;
        ar & timeToReachDefender;
        ar & currentRole;

        if (file_version >= 1)
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

        ar & doingBallLineUp;
        ar & isInReadyMode;
        ar & readyPositionAllocation;
    }
};

BOOST_CLASS_VERSION(BehaviourSharedData, 3);