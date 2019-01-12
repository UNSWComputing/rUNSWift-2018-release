#include "BehaviourHelpers.hpp"

#include "blackboard/Blackboard.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

namespace BehaviourHelpers {
   const ActionCommand::rgb LED_OFF(false, false, false);
   const ActionCommand::rgb LED_RED(true, false, false);
   const ActionCommand::rgb LED_GREEN(false, true, false);
   const ActionCommand::rgb LED_BLUE(false, false, true);
   const ActionCommand::rgb LED_YELLOW(true, true, false);
   const ActionCommand::rgb LED_CYAN(false, true, true);
   const ActionCommand::rgb LED_MAGENTA(true, false, true);
   const ActionCommand::rgb LED_WHITE(true, true, true);

   int playerNumber(Blackboard *blackboard) {
      return (readFrom(gameController, player_number));
   }

   int teamNumber(Blackboard *blackboard) {
      return (readFrom(gameController, our_team.teamNumber));
   }
};
