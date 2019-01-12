
#include <vector>
#include <algorithm>
#include <fstream>
#include "Localiser.hpp"
#include "LocalisationAdapter.hpp"
#include "LocalisationDefs.hpp"

#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/incapacitated.hpp"
#include "utils/Timer.hpp"
#include "types/BroadcastData.hpp"
#include "types/BallInfo.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "perception/localisation/robotfilter/types/RobotFilterUpdate.hpp"
#include "perception/localisation/ballfilter/types/RobotFilterUpdate.hpp"
#include "perception/localisation/BallRobot.hpp"

using namespace std;

#define PENALISED_HYSTERESIS 500

LocalisationAdapter::LocalisationAdapter(Blackboard *bb) : Adapter(bb) {
   firstCycle = true;
   playerNumber = readFrom(gameController, player_number);
   teamNumber = readFrom(gameController, our_team).teamNumber;
   prevTimestamp = 0;
   numFramesSinceSeen = 0;
   prevGameState = 0;
   isInPenaltyShootout = false;
   isPickedUpHysteresis = 0;
   isPenalisedHysteresis = 0;
   penalised_before_set = false;
   leftRight = 0;
   forwardBackward = 0;

   //If setFallen is set to true, we start the robot in getting up state
   isGettingUp = readFrom(localisation, setFallen);

   if (readFrom(localisation, setInitialPose)) {
      L = new Localiser(playerNumber, teamNumber, &readFrom(localisation, robotPos));
   }
   else {
      L = new Localiser(playerNumber, teamNumber);
   }
   robotFilter = new RobotFilter();
   ballFilter = new BallFilter();

   if (LOCALISATION_DEBUG) {
      logStream.open("localisation.log", std::ios::binary);
      logStream.rdbuf()->pubsetbuf(0,0);
   }
}

LocalisationAdapter::~LocalisationAdapter() {
   delete L;
   delete robotFilter;
   delete ballFilter;

   if (LOCALISATION_DEBUG) {
      logStream.close();
   }
}

unsigned totalFrames = 0;
unsigned framesBuckets[] = {0, 0, 0, 0, 0, 0}; // <10fps, <15fps, <20fps, <25fps, <30fps, >30fps

void logFPS(unsigned fps) {
   totalFrames++;
   if (fps <= 10) {
      framesBuckets[0]++;
   } else if (fps <= 15) {
      framesBuckets[1]++;
   } else if (fps <= 20) {
      framesBuckets[2]++;
   } else if (fps <=25) {
      framesBuckets[3]++;
   } else if (fps <= 30) {
      framesBuckets[4]++;
   } else if (fps > 30) {
      framesBuckets[5]++;
   }

   if (totalFrames % 60 == 0) {
      std::cout << std::endl;
      std::cout << "< 10: \t" << (100 * framesBuckets[0])/totalFrames << std::endl;
      std::cout << "< 15: \t" << (100 * framesBuckets[1])/totalFrames << std::endl;
      std::cout << "< 20: \t" << (100 * framesBuckets[2])/totalFrames << std::endl;
      std::cout << "< 25: \t" << (100 * framesBuckets[3])/totalFrames << std::endl;
      std::cout << "< 30: \t" << (100 * framesBuckets[4])/totalFrames << std::endl;
      std::cout << "> 30: \t" << (100 * framesBuckets[5])/totalFrames << std::endl;
   }
}

void LocalisationAdapter::tick() {
   llog(VERBOSE) << "Localisation.. ticking away" << endl;
   Timer timer;
   timer.restart();
   const ActionCommand::All active = readFrom(motion, active);

   /* calculate odometry */
   const Odometry &newOdometry = readFrom(motion, odometry);

   Odometry odometryDiff;
   if (!firstCycle) {
      odometryDiff = (newOdometry - prevOdometry);
   }
   prevOdometry = newOdometry;

   int64_t timestamp = readFrom(vision, timestamp);
   double dTimeSeconds = 0.0;
   if (!firstCycle) {
      // timestamp is in micro-seconds, hence the 1e6.
      dTimeSeconds = (timestamp - prevTimestamp) / 1000000.0;

      //unsigned fps = 1.0 / dTimeSeconds;
      //logFPS(fps);
   }
   prevTimestamp = timestamp;

   RobotFilterUpdate ball_update;

   std::vector <BallInfo> uncertain_balls = readFrom(vision, uncertain_balls);
   std::vector <RobotInfo> robot_balls;

   for (std::vector <BallInfo>::iterator it = uncertain_balls.begin(); it != uncertain_balls.end(); it++) {
      robot_balls.push_back(ballToRobot(*it));
   }

   ball_update.visualRobots = robot_balls;
   ball_update.robotPos = L->getRobotPose();
   ball_update.headYaw = readFrom(motion, sensors).joints.angles[Joints::HeadYaw];
   ball_update.odometryDiff = odometryDiff;
   ball_update.isIncapacitated = isIncapacitated(active.body.actionType);

   ballFilter->update(ball_update);

   std::vector<RobotObstacle> ball_robots = ballFilter->filteredRobots;

   std::vector <BallInfo> balls;

   CameraToRR conv_rr;
   // TODO Read current Pose from blakboard
   conv_rr.pose = readFrom(motion, pose);
   conv_rr.updateAngles(readFrom(kinematics, sensorsLagged));

   for (std::vector <RobotObstacle>::iterator it = ball_robots.begin(); it != ball_robots.end(); it++) {
      balls.push_back(robotObstacleToBall(*it, conv_rr));
   }

   if(balls.size() > 0)
   {
      writeTo(vision, balls, balls);
   }

   SensorValues values = readFrom(kinematics, sensorsLagged);
   VisionUpdateBundle visionUpdateBundle(
         readFrom(vision, fieldBoundaries),
         readFrom(vision, fieldFeatures),
         readFrom(vision, posts),
         readFrom(vision, robots),
         getTeammatePoses(),
         readFrom(gameController, team_red),
         readFrom(vision, awayGoalProb),
         values.joints.angles[Joints::HeadYaw],
         readFrom(vision, balls),
         !amTurningHead(active),
         !amWalking(active));

   if (haveTransitionedSetToPlay() && readFrom(gameController, data).gamePhase != GAME_PHASE_PENALTYSHOOT) {
      L->startOfPlayReset();
   }

   bool doingBallLineUp = readFrom(behaviour, behaviourSharedData).doingBallLineUp;
   L->setLineUpMode(doingBallLineUp);

   bool isInReadyMode = readFrom(behaviour, behaviourSharedData).isInReadyMode;
   L->setReadyMode(isInReadyMode);

   if ((readFrom(gameController, data).state == STATE_INITIAL && ((readFrom(behaviour, skill) == "GameController")||(readFrom(behaviour, skill) == "GameControllerV2")))) {
      L->setInitialGamePose(playerNumber);
   }

   //handleObservationMeasurements(visionUpdateBundle);
   if (readFrom(gameController, our_team).players[playerNumber - 1].penalty != PENALTY_NONE) {
      // Not ideal
      // TODO: Replace this with a timer
      // Use a hysteresis in case of incorrect penalising
      isPenalisedHysteresis = min(isPenalisedHysteresis + 1, PENALISED_HYSTERESIS);
   }
   else {
      isPenalisedHysteresis = 0;
   }

   bool isPenalised = isPenalisedHysteresis == PENALISED_HYSTERESIS;

   bool wasGettingUp = isGettingUp;
   ActionCommand::Body::ActionType currentAction = readFrom(motion, active).body.actionType;
   isGettingUp = (currentAction == ActionCommand::Body::GETUP_FRONT)
                  || (currentAction == ActionCommand::Body::GETUP_BACK)
                  || (currentAction == ActionCommand::Body::TIP_OVER);

   if (currentAction == ActionCommand::Body::REF_PICKUP) {
      isPickedUpHysteresis = min(isPickedUpHysteresis + 1, 10);
   }
   else {
      isPickedUpHysteresis = max(isPickedUpHysteresis - 1, 0);
   }

    // Enter manual placement if we're picked up in set or were penalised then
    // entered set.
    if ((readFrom(gameController, data).state == STATE_SET) &&
        (isPickedUpHysteresis == 10) && (readFrom(gameController, our_team
        ).players[playerNumber-1].penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET))
    {
        L->resetToManualPlacement(readFrom(gameController, data).kickingTeam);
        writeTo(vision, homeMapSize, 1);
    }

   if (readFrom(gameController, data).state == STATE_READY || readFrom(gameController, our_team).players[playerNumber - 1].penalty != PENALTY_NONE) {
      writeTo(vision, homeMapSize, 0);
   }

   handleMySharedDistribution();
   llog(VERBOSE) << "Localisation.. handle shared took " << timer.elapsed_us() << endl;
   timer.restart();

   /* update our own localisation */
   LocaliserBundle localiserBundle(
         odometryDiff,
         visionUpdateBundle,
         active,
         isPenalised,
         readFrom(gameController, data).state,
         readFrom(gameController, data).gamePhase,
         dTimeSeconds);

   if (LOCALISATION_DEBUG) {
      logStream << localiserBundle;
      logStream.flush();
   }

   int fallen_ang = 70;
   SensorValues sensors = readFrom(motion, sensors);

   float ang[2] = {RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]),
                  RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY])};

   // 1  - forward
   // -1 - backward
   // 1  - left
   // -1 - right

   if(ang[1] < -fallen_ang) {
      // Back
      forwardBackward = -1;
   } else if(ang[1] > fallen_ang) {
      // Front
      forwardBackward = 1;
   }

   if (ang[0] > 50) {
      // Right
      leftRight = -1;
   } else if (ang[0] < -50) {
      // Left
      leftRight = 1;
   }

   uint8_t myTeam = readFrom(gameController, our_team).teamNumber;
   uint8_t kickTeam = readFrom(gameController, data).kickingTeam;
   bool isKeeper = myTeam != kickTeam;

    // We need to avoid resetting hypothesis when penalised in place, so check
    // both if the hysteresis is passed and if we were penalised for motion in
    // set.
    if (isPenalised && readFrom(gameController, our_team).players[playerNumber -
                                1].penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
    {
        if (isInPenaltyShootout) {
            L->resetToPenaltyShootout(isKeeper);
        } else {
            L->setReset();
        }
    } else if (wasGettingUp != isGettingUp && wasGettingUp) {
        L->fallenOverReset(leftRight, forwardBackward);
        leftRight = 0;
        forwardBackward = 0;
        writeTo(localisation, getupLost, true);
    } else if (canLocaliseInState(readFrom(gameController, data).state, readFrom(behaviour, skill))) {
        llog(VERBOSE) << "Localisation.. localising" << endl;
        L->localise(localiserBundle, canDoObservations());
    }

   bool getupLost = readFrom(localisation, getupLost);
   if (getupLost && !L->headingUncertain()) {
      writeTo(localisation, getupLost, false);
   }
   llog(VERBOSE) << "Localisation.. localising took " << timer.elapsed_us() << endl;
   timer.restart();


   if (canLocaliseInState(readFrom(gameController, data).state, readFrom(behaviour, skill)) &&
       !isInPenaltyShootout) {
      handleIncomingSharedUpdate();
   }

   llog(VERBOSE) << "Localisation.. shared update took " << timer.elapsed_us() << endl;
   timer.restart();

   RobotFilterUpdate update;
   update.visualRobots = readFrom(vision, robots);
   update.robotPos = readFrom(localisation, robotPos);
   update.headYaw = readFrom(motion, sensors).joints.angles[Joints::HeadYaw];
   update.odometryDiff = odometryDiff;
   update.isIncapacitated = isIncapacitated(active.body.actionType);

   robotFilter->update(update);

   writeResultToBlackboard();
   llog(VERBOSE) << "Localisation.. blackboard write took " << timer.elapsed_us() << endl;
   timer.restart();

   firstCycle = false;

   if (haveTransitionedIntoSet() && readFrom(gameController, data).gamePhase == GAME_PHASE_PENALTYSHOOT) {
      if (readFrom(gameController, data).competitionType != COMPETITION_TYPE_GENERAL_PENALTY_KICK){
            isInPenaltyShootout = true;
            L->resetToPenaltyShootout(isKeeper);
      }
      else {
            L->resetToPenaltyKickChallenge(isKeeper);
      }
   }
   else if (readFrom(gameController, data).state == STATE_SET &&
      readFrom(gameController, data).gamePhase == GAME_PHASE_PENALTYSHOOT &&
      readFrom(gameController, data).competitionType != COMPETITION_TYPE_GENERAL_PENALTY_KICK){
            isInPenaltyShootout = true;
            L->resetToPenaltyShootout(isKeeper);
      }

//   L->worldDistribution->printModes();
//   if (LOCALISATION_DEBUG) {
//      logStream << localiserBundle;
//   }

   prevGameState = readFrom(gameController, data).state;
   llog(VERBOSE) << "Localisation.. total took " << timer.elapsed_us() << endl;
   llog(VERBOSE) << "Localisation: Finished" << std::endl;
}

bool LocalisationAdapter::canLocaliseInState(uint8_t state, std::string skill) {
   if (skill == "GameController"|| skill == "GameControllerV2") {
      return state == STATE_READY || state == STATE_SET || state == STATE_PLAYING;
   } else {
      return true;
   }
}

bool LocalisationAdapter::canDoObservations(void) {
   ActionCommand::Body::ActionType currentAction = readFrom(motion, active).body.actionType;

   bool isPickedUp = (currentAction == ActionCommand::Body::REF_PICKUP);
   bool isDiving = (currentAction == ActionCommand::Body::GOALIE_DIVE_LEFT || currentAction == ActionCommand::Body::GOALIE_DIVE_RIGHT);
   bool isDead = (currentAction == ActionCommand::Body::DEAD);
   bool isGettingUp =
         (currentAction == ActionCommand::Body::GETUP_FRONT)
         || (currentAction == ActionCommand::Body::GETUP_BACK)
         || (currentAction == ActionCommand::Body::TIP_OVER);

   // TODO: is there a "I am falling" state?
   return !isPickedUp && !isDiving && !isGettingUp && !isDead;
}

bool LocalisationAdapter::haveTransitionedSetToPlay(void) {
   uint8_t curGameState = readFrom(gameController, data).state;
   return prevGameState == STATE_SET && curGameState == STATE_PLAYING;
}

bool LocalisationAdapter::haveTransitionedIntoSet(void) {
   uint8_t curGameState = readFrom(gameController, data).state;
   return prevGameState != STATE_SET && curGameState == STATE_SET;
}

std::vector<AbsCoord> LocalisationAdapter::getTeammatePoses(void) {
   const BroadcastData *teamData = readFrom(receiver, data);
   const bool *incapacitated = readFrom(receiver, incapacitated);

   std::vector<AbsCoord> result;
   for (unsigned i = 0; i < ROBOTS_PER_TEAM; i++) {
      if (i != (playerNumber - 1) &&
          !incapacitated[i] &&
          teamData[i].uptime > 1.0f &&
          teamData[i].acB != Body::DEAD &&
          teamData[i].acB != Body::REF_PICKUP) {
          const AbsCoord pos(teamData[i].robotPos[0],teamData[i].robotPos[1],teamData[i].robotPos[2]);
          result.push_back(pos);
      }
   }
   return result;
}

bool LocalisationAdapter::amWalking(const ActionCommand::All &commands) {
   return abs(commands.body.forward) > 0 || abs(commands.body.left) > 0 || abs(commands.body.turn) > 0;
}

bool LocalisationAdapter::amTurningHead(const ActionCommand::All &commands) {
   return abs(commands.head.yawSpeed) > 0.1;
}

void LocalisationAdapter::writeResultToBlackboard(void) {
   AbsCoord robotPos = L->getRobotPose();
   AbsCoord ballPosition = L->getBallPosition();
   RRCoord ballPosRR = ballPosition.convertToRobotRelative(robotPos);
   AbsCoord ballPosRRC = ballPosition.convertToRobotRelativeCartesian(robotPos);
   AbsCoord ballVelocity = L->getBallVelocity();
   std::vector < AbsCoord > allrobotPos = L->getAllRobotPoses();

   double robotPosUncertainty = L->getRobotPosUncertainty();
   double robotHeadingUncertainty = L->getRobotHeadingUncertainty();
   double ballPosUncertainty = L->getBallPosUncertainty();
   double ballVelEigenvalue = L->getBallVelocityUncertainty();

   AbsCoord nextBall(ballPosition.x() + ballVelocity.x(), ballPosition.y() + ballVelocity.y(), 0.0f);
   AbsCoord nextBallRRC = nextBall.convertToRobotRelativeCartesian(robotPos);
   AbsCoord ballVelRRC(nextBallRRC.x() - ballPosRRC.x(), nextBallRRC.y() - ballPosRRC.y(), 0.0f);

   Pose pose = readFrom(motion, pose);
   XYZ_Coord ballNeckRelative = pose.robotRelativeToNeckCoord(ballPosRR, BALL_RADIUS);

   uint32_t ballLostCount = L->getBallLostCount();
   uint32_t ballSeenCount = L->getBallSeenCount();

   SharedLocalisationUpdateBundle sharedLocalisationBundle = L->getSharedUpdateData();

   acquireLock(serialization);
   writeTo(localisation, robotPos, robotPos);
   writeTo(localisation, allrobotPos, allrobotPos);
   writeTo(localisation, ballLostCount, ballLostCount);
   writeTo(localisation, ballSeenCount, ballSeenCount);
   writeTo(localisation, ballPos, ballPosition);
   writeTo(localisation, ballPosRR, ballPosRR);
   writeTo(localisation, ballPosRRC, ballPosRRC);
   writeTo(localisation, ballVelRRC, ballVelRRC);
   writeTo(localisation, ballVel, ballVelocity);
   writeTo(localisation, robotPosUncertainty, robotPosUncertainty);
   writeTo(localisation, robotHeadingUncertainty, robotHeadingUncertainty);
   writeTo(localisation, ballPosUncertainty, ballPosUncertainty);
   writeTo(localisation, ballVelEigenvalue, ballVelEigenvalue);
   writeTo(localisation, teamBall, TeamBallInfo()); // TODO: make this a different value?
   writeTo(localisation, ballNeckRelative, ballNeckRelative);
   writeTo(localisation, sharedLocalisationBundle, sharedLocalisationBundle);
   writeTo(localisation, havePendingOutgoingSharedBundle, true);
   writeTo(localisation, havePendingIncomingSharedBundle, std::vector<bool>(5, false));
   writeTo(localisation, robotObstacles, robotFilter->filteredRobots);
   releaseLock(serialization);
}

std::vector<RobotObstacle> LocalisationAdapter::getRobotsFromFilter(void) {
   return robotFilter->filteredRobots;
}

void LocalisationAdapter::handleMySharedDistribution(void) {
   if (!readFrom(localisation, havePendingOutgoingSharedBundle)) {
      L->resetSharedUpdateData();
   }
}

void LocalisationAdapter::handleIncomingSharedUpdate(void) {
   std::vector<bool> incomingSharedUpdates = readFrom(localisation, havePendingIncomingSharedBundle);
   for (unsigned i = 0; i < incomingSharedUpdates.size(); i++) {
      if (incomingSharedUpdates[i]) {
         BroadcastData incomingData = readFrom(receiver, data)[i];
         L->applyRemoteUpdate(incomingData, incomingData.playerNum);
      }
   }
}

/**
 * All of the code below is used for gathering observation variance data. We basically have the
 * robot stand and observe say a ball, average out the observations, and calculate the variance.
 * We then cover the robots vision, move the ball, and uncover the vision. This indicates a new
 * round and we do the same thing but now with a different ball distance/heading. This data allows
 * us to use an offline util to fit a function that maps observation distance to variance.
 */
// TODO(sushkov): more detailed explanation of the below code.
void LocalisationAdapter::handleObservationMeasurements(const VisionUpdateBundle &visionUpdateBundle) {
   if (visionUpdateBundle.visibleBalls.size() == 0) {
      numFramesSinceSeen++;
      if (numFramesSinceSeen > 150) {
         if (distanceObservations.size() > 100) {
            outputVariances();
         }

         numFramesSinceSeen = 0;
         distanceObservations.clear();
         headingObservations.clear();
      }
   } else {
      numFramesSinceSeen = 0;
      distanceObservations.push_back(visionUpdateBundle.visibleBalls[0].rr.distance());
      headingObservations.push_back(visionUpdateBundle.visibleBalls[0].rr.heading());
   }
}

void LocalisationAdapter::outputVariances(void) {
   std::sort(distanceObservations.begin(), distanceObservations.end());
   std::sort(headingObservations.begin(), headingObservations.end());

   double outliersRatio = 0.1;

   double distanceMean = getMean(distanceObservations, outliersRatio);
   double distanceVariances = getVariance(distanceObservations, distanceMean, outliersRatio);

   double headingMean = getMean(headingObservations, outliersRatio);
   double headingVariance = getVariance(headingObservations, headingMean, outliersRatio);

   std::cout << "variance obs: " << distanceMean << " " << distanceVariances << " "
         << headingMean << " " << headingVariance << std::endl;
}

double LocalisationAdapter::getMean(const std::vector<double> &vals, double outliersRatio) {
   int outliers = vals.size() * outliersRatio;
   double sum = 0.0;
   int count = 0;

   for (unsigned i = outliers; i < vals.size() - outliers; i++) {
      count++;
      sum += vals[i];
   }

   return sum / count;
}

double LocalisationAdapter::getVariance(const std::vector<double> &vals, double mean,
      double outliersRatio) {
   int outliers = vals.size() * outliersRatio;
   double sum = 0.0;
   int count = 0;

   for (unsigned i = outliers; i < vals.size() - outliers; i++) {
      count++;
      double diff = vals[i] - mean;
      sum += diff * diff;
   }

   return sum / count;
}
