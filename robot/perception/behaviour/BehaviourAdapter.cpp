#include <Python.h>

#include <fstream>
#include <limits>
#include <utility>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include "perception/behaviour/BehaviourAdapter.hpp"
#include "perception/behaviour/BehaviourHelpers.hpp"

#include "ReadySkillPositionAllocation.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "types/BehaviourRequest.hpp"
#include "utils/body.hpp"
#include "utils/speech.hpp"
#include "utils/basic_maths.hpp"
#include "types/SensorValues.hpp"
#include "perception/behaviour/python/PythonSkill.hpp"

#include <boost/python.hpp>

using namespace std;
using namespace boost::python;

BehaviourAdapter::BehaviourAdapter(Blackboard *bb) : Adapter(bb), calibrationSkill(bb),
         safetySkill(bb->behaviour.useGetups)
{
   llog(INFO) << "Constructing BehaviourAdapter" << endl;
   std::string hostname;
   ifstream hostfile ("/etc/hostname");
   getline (hostfile, hostname);
   pythonSkill = new PythonSkill(bb);

   // Alert runswift team - whistle detection requires 4 channels, not 2
   std::string noHearWhistles = "";
   int ret = system("python $HOME/whistle/alert_pulseaudio.py");
   if (ret != 0) {
      noHearWhistles += "I can not hear whistles. ";
      std::cout << noHearWhistles << std::endl;
   }

   // Max string length is 70 characters as defined by MAX_SAY_LENGTH in robot/libagent/AgentData.hpp
   std::stringstream startupSpeech;
   startupSpeech << noHearWhistles << std::string("Player ") << BehaviourHelpers::playerNumber(blackboard);
   if (ret != 0) {
     startupSpeech << " ... " << hostname;
   } else {
      if (BehaviourHelpers::teamNumber(blackboard) == 18) {
          startupSpeech << " team rUNSWift";
      }
      else {
          startupSpeech << " team " << BehaviourHelpers::teamNumber(blackboard);
      }
      if (hostname == "mario") {
            startupSpeech << " ... It's a me ... " << hostname << " ... wah who! ...";
      }
      else {
            startupSpeech << " ... I am ... " << hostname;
      }
   }
   std::cout << startupSpeech.str() << std::endl;
   SAY(startupSpeech.str());
}

BehaviourAdapter::~BehaviourAdapter() {
}

void BehaviourAdapter::tick() {
   BehaviourRequest behaviourRequest;

   if (readFrom(kinematics, isCalibrating)) {
      // kinematics calibrator
      behaviourRequest = calibrationSkill.execute();
   } else {
      // Run the python skill
      behaviourRequest = pythonSkill->execute();
   }

   // override camera from offnao if necessary
   string whichCamera = blackboard->config["default.whichCamera"].as<string>();
   if (whichCamera != "BEHAVIOUR") {
      if (whichCamera == "TOP_CAMERA") {
         behaviourRequest.whichCamera = TOP_CAMERA;
      } else if (whichCamera == "BOTTOM_CAMERA") {
         behaviourRequest.whichCamera = BOTTOM_CAMERA;
      }
   }

   // Write ActionCommands to blackboard
   int writeBuf = (readFrom(behaviour, readBuf) + 1) % 2;
   writeTo(behaviour, request[writeBuf], safetySkill.wrapRequest(behaviourRequest, readFrom(motion, sensors)));
   writeTo(behaviour, readBuf, writeBuf);

   if (behaviourRequest.readyPositionAllocation0 > 0) {
      std::vector<int> readyAllocation;
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation0);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation1);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation2);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation3);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation4);

      ReadySkillPositionAllocation newPositionAllocation(
            readFrom(gameController, player_number), readyAllocation);

      ReadySkillPositionAllocation currentPositionAllocation =
            readFrom(behaviour, behaviourSharedData).readyPositionAllocation;
      if (newPositionAllocation.canOverride(currentPositionAllocation)) {
         writeTo(behaviour, behaviourSharedData.readyPositionAllocation, newPositionAllocation);
      }
   }

   // Write behaviourRequest attributes to behaviourSharedData on blackboard, to broadcast to the team
   writeTo(behaviour, behaviourSharedData.goalieAttacking, behaviourRequest.goalieAttacking);
   writeTo(behaviour, behaviourSharedData.goalieDiving, behaviourRequest.goalieDiving);
   writeTo(behaviour, behaviourSharedData.doingBallLineUp, behaviourRequest.doingBallLineUp);
   writeTo(behaviour, behaviourSharedData.kickoffSide, behaviourRequest.kickoffSide);
   writeTo(behaviour, behaviourSharedData.isInReadyMode, behaviourRequest.isInReadyMode);
   writeTo(behaviour, behaviourSharedData.timeToReachBall, behaviourRequest.timeToReachBall);
   writeTo(behaviour, behaviourSharedData.timeToReachDefender, behaviourRequest.timeToReachDefender);
   writeTo(behaviour, behaviourSharedData.timeToReachMidfielder, behaviourRequest.timeToReachMidfielder);
   writeTo(behaviour, behaviourSharedData.timeToReachUpfielder, behaviourRequest.timeToReachUpfielder);
   writeTo(behaviour, behaviourSharedData.currentRole, behaviourRequest.currentRole);
   writeTo(behaviour, behaviourSharedData.role, behaviourRequest.role);
   writeTo(behaviour, behaviourSharedData.playingBall, behaviourRequest.playingBall);
   writeTo(behaviour, behaviourSharedData.needAssistance, behaviourRequest.needAssistance);
   writeTo(behaviour, behaviourSharedData.isAssisting, behaviourRequest.isAssisting);
   writeTo(behaviour, behaviourSharedData.isKickedOff, behaviourRequest.isKickedOff);
   writeTo(behaviour, behaviourSharedData.isFollowing, behaviourRequest.isFollowing);
   writeTo(behaviour, behaviourSharedData.secondsSinceLastKick, behaviourRequest.secondsSinceLastKick);


}
