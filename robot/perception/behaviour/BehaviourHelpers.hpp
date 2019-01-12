#pragma once

#include "types/ActionCommand.hpp"

class Blackboard;
class BehaviourRequest;
class PostInfo;
class RRCoord;

namespace BehaviourHelpers {
   int playerNumber(Blackboard *blackboard);
   int teamNumber(Blackboard *blackboard);

   extern const ActionCommand::rgb LED_OFF;
   extern const ActionCommand::rgb LED_RED;
   extern const ActionCommand::rgb LED_GREEN;
   extern const ActionCommand::rgb LED_BLUE;
   extern const ActionCommand::rgb LED_YELLOW;
   extern const ActionCommand::rgb LED_CYAN;
   extern const ActionCommand::rgb LED_MAGENTA;
   extern const ActionCommand::rgb LED_WHITE;
};
