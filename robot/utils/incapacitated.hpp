#pragma once

#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/ActionCommand.hpp"

#define SECS_TILL_INCAPACITATED 3
#define MIN_UPTIME 20

using namespace ActionCommand;

inline bool isIncapacitated(const ActionCommand::Body::ActionType &acB) {
   return
   	  // VWong: If you have fallen over, you're not completely incapacitated
      //acB == Body::GETUP_BACK ||
      //acB == Body::GETUP_FRONT ||
      //acB == Body::TIP_OVER ||
      //acB == Body::DEAD ||
      acB == Body::REF_PICKUP;
      // acB == Body::GOALIE_DIVE_RIGHT ||
      // acB == Body::GOALIE_DIVE_LEFT
}
