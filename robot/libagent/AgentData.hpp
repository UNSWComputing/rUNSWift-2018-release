#pragma once

#include "types/ActionCommand.hpp"
#include "types/JointValues.hpp"
#include "types/SensorValues.hpp"
#include "types/ButtonPresses.hpp"

#define AGENT_MEMORY "/libagent-memory"
#define AGENT_SEMAPHORE "/libagent-semaphore"
#define AL_ON -2.0f
#define AL_command stiffnesses[Joints::LShoulderPitch]
#define AL_x angles[Joints::LShoulderPitch]
#define AL_y angles[Joints::LShoulderRoll]
#define AL_theta angles[Joints::LElbowYaw]
#define AL_frequency angles[Joints::LElbowRoll]
#define AL_stop angles[Joints::LHipYawPitch]
#define AL_reset angles[Joints::LHipRoll]
#define AL_height angles[Joints::LHipPitch]
#define AL_bend angles[Joints::LKneePitch]
#define AL_isActive joints.temperatures[Joints::LShoulderPitch]

/**
 * The longest say string for overheating is length 30.
 * The on startup SAY for "Player 6 Team 92 ... I am ... <hostname>"
 * is unbounded and with whistles not heard it becomes
 * "I can not hear whistles. Player 6 <hostname>"
 * so assume 70 is enough for now.
 */
#define MAX_SAY_LENGTH 70

struct AgentData {
   volatile uint8_t sensors_read;
   volatile uint8_t sensors_latest;
   volatile uint8_t actuators_latest;
   volatile uint8_t actuators_read;

   SensorValues sensors[3];
   ButtonPresses buttons[3];
   JointValues joints[3];
   ActionCommand::LED leds[3];
   float sonar[3];
   ActionCommand::Stiffen stiffen[3];
   char sayTexts[3][MAX_SAY_LENGTH];

   volatile bool standing;

   void init() {
      sensors_read = 0;
      sensors_latest = 0;
      actuators_read = 0;
      actuators_latest = 0;
      for (int i = 0; i < 3; i++) {
         sayTexts[i][0] = 0;
      }
   }
};
