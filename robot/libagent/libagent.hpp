#pragma once

#include <semaphore.h>
#include <bitset>
#include <string>
#include <vector>
#include <stack>
#include <boost/shared_ptr.hpp>
#include "alcommon/albroker.h"
#include "alcommon/alproxy.h"
#include "alproxies/alloggerproxy.h"
#include "alproxies/almemoryproxy.h"
#include "alproxies/alsonarproxy.h"
#include "alproxies/altexttospeechproxy.h"
#include "alcommon/almodule.h"
#include "alvalue/alvalue.h"
#include "alproxies/dcmproxy.h"
#include "libagent/AgentData.hpp"
#include "utils/Timer.hpp"

namespace AL {
    class ALBroker;
}

#define MAX_SKIPS 50
#define MAX_CLICK_INTERVAL 18

//Ints for the head buttons
#define HEAD_FRONT 1
#define HEAD_MIDDLE 2
#define HEAD_BACK 3

const float sit_angles_middle[Joints::NUMBER_OF_JOINTS] = {
    0.0,                // HeadYaw
    0.0,                // HeadPitch

    DEG2RAD(70),        // LShoulderPitch
    DEG2RAD(25),        // LShoulderRoll
    DEG2RAD(-92),       // LElbowYaw
    DEG2RAD(-61),       // LElbowRoll
    DEG2RAD(54),        // LWristYaw
    0.0,                // LHand

    DEG2RAD(0),         // LHipYawPitch
    DEG2RAD(0),         // LHipRoll
    DEG2RAD(-50),       // LHipPitch
    DEG2RAD(125),       // LKneePitch
    DEG2RAD(-70),       // LAnklePitch
    0.0,                // LAnkleRoll

    DEG2RAD(0),         // RHipRoll
    DEG2RAD(-50),       // RHipPitch
    DEG2RAD(125),       // RKneePitch
    DEG2RAD(-70),       // RAnklePitch
    0.0,                // RAnkleRoll

    DEG2RAD(70),        // RShoulderPitch
    DEG2RAD(-25),       // RShoulderRoll
    DEG2RAD(92),        // RElbowYaw
    DEG2RAD(61),        // RElbowRoll
    DEG2RAD(-54),       // RWristYaw
    0.0                 // RHand
};


const float sit_angles[Joints::NUMBER_OF_JOINTS] = {
    0.0,                // HeadYaw
    0.0,                // HeadPitch

    DEG2RAD(72),        // LShoulderPitch
    DEG2RAD(2),         // LShoulderRoll
    DEG2RAD(-33),       // LElbowYaw
    DEG2RAD(-47),       // LElbowRoll
    DEG2RAD(55),        // LWristYaw
    0.0,                // LHand

    DEG2RAD(0),         // LHipYawPitch
    DEG2RAD(0),         // LHipRoll
    DEG2RAD(-50),       // LHipPitch
    DEG2RAD(125),       // LKneePitch
    DEG2RAD(-70),       // LAnklePitch
    0.0,                // LAnkleRoll

    DEG2RAD(0),         // RHipRoll
    DEG2RAD(-50),       // RHipPitch
    DEG2RAD(125),       // RKneePitch
    DEG2RAD(-70),       // RAnklePitch
    0.0,                // RAnkleRoll

    DEG2RAD(72),        // RShoulderPitch
    DEG2RAD(-2),        // RShoulderRoll
    DEG2RAD(33),        // RElbowYaw
    DEG2RAD(47),        // RElbowRoll
    DEG2RAD(-55),       // RWristYaw
    0.0                 // RHand
};

/**
 * Acts an an agent between the Naoqi process and the rUNSWift soccer player
 * process, using shared memory and a named semaphore, to control the DCM and
 * read robot sensors. Provides safety measures if it loses contact with the
 * player. Inspired by libbhuman.
 */
#define AGENT_VERSION_MAJOR "0"
#define AGENT_VERSION_MINOR "0"
class Agent : public AL::ALModule {
    public:
        Agent(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName);
        virtual ~Agent();
        static const std::string name;
        void preCallback();
        void postCallback();
        void doLEDs(ActionCommand::LED& leds);
        void doButtons(bool chest, bool left, bool right);
        void doBattery(float charge, float current, int status);
        void doHeadTouch(
            float headTouchFront,
            float headTouchRear,
            float headTouchMiddle
        );
        void doTemps();
        void doStiffen(ActionCommand::Stiffen stf);

    private:
        AL::ALLoggerProxy* log;

        /* DCM */
        AL::DCMProxy* dcm;
        AL::ALValue angle_command;
        AL::ALValue stiffness_command;
        AL::ALValue head_angle_command;
        AL::ALValue head_stiffness_command;
        AL::ALValue led_command;
        AL::ALValue sonar_command;
        int time_offset;

        /* ALMemory */
        AL::ALMemoryProxy* memory;
        std::vector<float*> sensor_pointers;
        std::vector<float*> joint_pointers;
        std::vector<float*> temperature_pointers;
        std::vector<float*> current_pointers;
        std::vector<float*> sonar_pointers;
        int* battery_status_pointer;

        /*Sonar*/
        AL::ALSonarProxy*	 sonar;

        std::vector<std::string> leg_names;
        std::vector<float> stand_angles;

        /* ALTextToSpeech */
        AL::ALTextToSpeechProxy* speech;
        Timer sayTimer;

        /* Shared Memory */
        bool shuttingDown;
        int shared_fd;
        AgentData* shared_data;
        sem_t* semaphore;

        /* Safety */
        int skipped_frames;
        float sit_step;
        JointValues sit_joints;
        bool limp;
        bool head_limp;

        /* Button presses */
        // counters of how many 20 ms cycles button has been up or down for
        int chest_down;
        int chest_up;
        ButtonPresses buttons;
        // running counter of how many presses have been recorded so far
        unsigned int chest_presses;

        /* Head presses */
        std::stack<int> head_press_stack;
        unsigned int last_press_time;

        /* Battery */
        int old_battery;
        std::bitset<16> old_battery_status;

        /* Who am I? */
        std::string hostname;
};
