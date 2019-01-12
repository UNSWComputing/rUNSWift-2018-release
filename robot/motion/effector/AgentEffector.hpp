#pragma once

#include "motion/effector/Effector.hpp"
#include "libagent/AgentData.hpp"

class AgentEffector : Effector {
   public:
      explicit AgentEffector(int team, int player_number);
      ~AgentEffector();
      void actuate(JointValues joints, ActionCommand::LED leds, float sonar, ActionCommand::Stiffen stiffen);

   private:
      int shared_fd;
      AgentData* shared_data;
};
