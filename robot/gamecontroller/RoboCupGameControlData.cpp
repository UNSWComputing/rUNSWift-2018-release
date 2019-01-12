#include "gamecontroller/RoboCupGameControlData.hpp"

const char *gameControllerGamePhaseNames[] = {
   " ",  // Don't care about saying normal
   "shootout ",
   "overtime ",
   "timeout "
   // Note extra space at end so we don't merge words
};

const char *gameControllerStateNames[] = {
   "initial",
   "ready",
   "set",
   "playing",
   "finished",
   "invalid",
   "penalised"
};

const char *gameControllerPenaltyNames[] = {
   "none",
   "illegal ball contact",
   "player pushing",
   "motion in set",
   "inactive player",
   "illegal defender",
   "leaving the field",
   "kick off goal",
   "request for pickup",
   "local game stuck",
   "invalid",
   "invalid",
   "invalid",
   "invalid",
   "substitute penalty",
   "manual penalty",
};
