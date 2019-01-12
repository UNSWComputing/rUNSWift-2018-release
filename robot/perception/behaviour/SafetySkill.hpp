#pragma once

#define MIN_STANDING_WEIGHT 0.55f
#define FALLEN 9
#define FALLING 8
#define FALLEN_ANG 70
#define FALLING_ANG 50

#include "types/BehaviourRequest.hpp"
#include "types/SensorValues.hpp"

class SafetySkill {
   public:
      SafetySkill(bool use_getups);
      ~SafetySkill();
      BehaviourRequest wrapRequest(const BehaviourRequest &request, const SensorValues &s);
   private:
      float filtered_fsr_sum;
      int blink;
      float sag_angular_velocity;
      float cor_angular_velocity;
      float prev_angles[2];
      bool useGetups;
};
