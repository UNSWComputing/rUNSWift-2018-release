#include "MultiGaussianDistribution.hpp"
#include "LocalisationConstantsProvider.hpp"
#include "SimpleGaussian.hpp"
#include "VisionUpdateBundle.hpp"
#include "types/Odometry.hpp"
#include "LocalisationUtils.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

#include <cassert>
#include <vector>
#include <algorithm>

#include "Eigen/Geometry"
#include "Eigen/LU"

using namespace Eigen;

static const LocalisationConstantsProvider& constantsProvider(
      LocalisationConstantsProvider::instance());

/**
 * Comparison functor for sorting the list of Gaussians into descending order by weight.
 */
struct GuassianSortFunction {
   bool operator()(const SimpleGaussian *lhs, const SimpleGaussian *rhs) {
      return lhs->getWeight() > rhs->getWeight();
   }
};

MultiGaussianDistribution::MultiGaussianDistribution(unsigned maxGaussians, int playerNumber, const AbsCoord* initialPose) :
      maxGaussians(maxGaussians), playerNumber(playerNumber),
      teamBallTracker(playerNumber) {
   MY_ASSERT(maxGaussians > 0, "invalid number of maxGaussians");

   if (initialPose == NULL) {
      resetDistributionToPenalisedPose();
   }
   else {
      setInitialPose(*initialPose);
   }

   lastObservationLikelyhood = 0.00001;

   isInReadyMode = false;
   haveSeenLandmarks = false;
   numVisionUpdatesInReady = 0;
}

MultiGaussianDistribution::~MultiGaussianDistribution() {
   for (unsigned i = 0; i < modes.size(); i++) {
      delete modes[i];
   }
}

void MultiGaussianDistribution::setInitialPose(AbsCoord initialPose) {
   Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(MAIN_DIM, 1);
   Eigen::MatrixXd previousTop(MAIN_DIM, 1);

   mean(0, 0) = initialPose.x();
   mean(1, 0) = initialPose.y();
   mean(2, 0) = initialPose.theta();

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   //VCTR Hardcoded initial
   diagonalVariance << 100000, 100000, 0.5,
                       0.0, 0.0, 0.0, 0.0;
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   for (unsigned i = 0; i < modes.size(); i++) delete modes[i];
   modes.clear();
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));
}

void MultiGaussianDistribution::setInitialGamePose(int playerNumber) {
   Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(MAIN_DIM, 1);
   Eigen::MatrixXd previousTop(MAIN_DIM, 1);

   int initial_x, initial_y;
   float initial_theta;

   switch (playerNumber) {
      case 1:
         initial_x = -3900;
         initial_y = 3000;
         initial_theta = -M_PI_2;
         break;

      case 2:
         initial_x = -2000;
         initial_y = -3000;
         initial_theta = M_PI_2;
         break;

      case 3:
         initial_x = -1500;
         initial_y = 3000;
         initial_theta = -M_PI_2;
         break;

      case 4:
         initial_x = -1000;
         initial_y = -3000;
         initial_theta = M_PI_2;
         break;

      case 5:
         initial_x = -2700;
         initial_y = 3000;
         initial_theta = -M_PI_2;
         break;

      case 6:
         initial_x = -3000;
         initial_y = -3000;
         initial_theta = M_PI_2;
         break;

      default:
         initial_x = -4000;
         initial_y = -3000;
         initial_theta = M_PI_2;
         break;
   }

   mean << initial_x, initial_y, initial_theta,
              0.0, 0.0,
              0.0, 0.0;

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   //VCTR Hardcoded initial
   diagonalVariance << 100000, 100000, 0.5,
                       0.0, 0.0, 0.0, 0.0;
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   for (unsigned i = 0; i < modes.size(); i++) delete modes[i];
   modes.clear();
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));
}

void MultiGaussianDistribution::resetDistributionToPenalisedPose(void) {
   Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(MAIN_DIM, 1);
   Eigen::MatrixXd previousTop(MAIN_DIM, 1);
   bool hasPreviousLocalisation = false;
   // Standing on the left field edge in our half looking inward.
   if (modes.size() == 0) {
      mean << -2.0*FIELD_LENGTH/6.0, FIELD_WIDTH/2.0, -M_PI/2.0,
              0.0, 0.0,
              0.0, 0.0;
      // TODO: this may not be the optimal thing to do for teammate poses. Maybe copy over
      // the pose from the top Gaussian if it is available?
   } else {
      mean = modes.front()->getMean();
      mean(0, 0) = -FIELD_LENGTH/3.0;
      mean(1, 0) = FIELD_WIDTH/2.0;
      mean(2, 0) = -M_PI/2.0;

      previousTop = modes.front()->getMean();
      hasPreviousLocalisation = true;
   }

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   diagonalVariance << get95CF(FULL_FIELD_LENGTH/4.0), get95CF(FULL_FIELD_WIDTH/4.0), get95CF(M_PI / 4.0),
                       get95CF(10.0*FULL_FIELD_LENGTH), get95CF(10.0*FULL_FIELD_WIDTH),
                       get95CF(10000.0), get95CF(10000.0);
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   for (unsigned i = 0; i < modes.size(); i++) delete modes[i];
   modes.clear();
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

   mean(1, 0) *= -1.0;
   mean(2, 0) *= -1.0;
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));


   mean(0, 0) += FIELD_LENGTH/6.0;
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

   mean(1, 0) *= -1.0;
   mean(2, 0) *= -1.0;
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

   if (hasPreviousLocalisation) {
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/8.0, previousTop, diagonalVariance));
   }
}

void MultiGaussianDistribution::resetToPenaltyKickChallenge(bool isKeeper) {
   Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(MAIN_DIM, 1);
   if (isKeeper) 
      mean << -FIELD_LENGTH/2.0, 0.0, 0.0, // If youre the goalie, in goalbox facing opponent
              0.0, 0.0,
              0.0, 0.0;
   else
      mean << 2200.0, 0.0, 0.0, // centre of the field, facing opposition goal.
              3200.0, 0.0, // 2m in front of you for the ball.
              0.0, 0.0;

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   diagonalVariance << get95CF(FULL_FIELD_WIDTH/4.0), get95CF(FULL_FIELD_WIDTH/4.0), get95CF(M_PI / 8.0),
                     get95CF(10.0*FULL_FIELD_WIDTH), get95CF(10.0*FULL_FIELD_WIDTH),
                     get95CF(10000.0), get95CF(10000.0);
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   for (unsigned i = 0; i < modes.size(); i++) delete modes[i];
   modes.clear();
   if(isKeeper)
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0, mean, diagonalVariance));
   else {
      //The center position is twice as likely as any other position, so the weight is 2.0
      modes.push_back(new SimpleGaussian(MAIN_DIM, 2.0/6.0, mean, diagonalVariance));
      mean(0, 0) = 2334.0; // 30 degrees, radius 1m from penalty spot
      mean(1, 0) = 500.0;
      mean(2, 0) = -M_PI/6.0;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/6.0, mean, diagonalVariance));

      mean(0, 0) = 2700.0; // 60 degrees, radius 1m from penalty spot
      mean(1, 0) = 866.0;
      mean(2, 0) = -M_PI/3.0;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/6.0, mean, diagonalVariance));

      mean(0, 0) = 2334.0; // -30 degrees, radius 1m from penalty spot
      mean(1, 0) = -500.0;
      mean(2, 0) = M_PI/6.0;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/6.0, mean, diagonalVariance));

      mean(0, 0) = 2700.0; // -60 degrees, radius 1m from penalty spot
      mean(1, 0) = -866.0;
      mean(2, 0) = M_PI/3.0;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/6.0, mean, diagonalVariance));
   }
}

void MultiGaussianDistribution::resetToPenaltyShootout(bool isKeeper) {
   Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(MAIN_DIM, 1);
   // Standing on the left field edge in our half looking inward.
   if (isKeeper) 
      mean << -FIELD_LENGTH/2.0, 0.0, 0.0, // If youre the goalie, in goalbox facing opponent
              0.0, 0.0,
              0.0, 0.0;
   else
      mean << 2200.0, 0.0, 0.0, // centre of the field, facing opposition goal.
              3200.0, 0.0, // 2m in front of you for the ball.
              0.0, 0.0;

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   diagonalVariance << get95CF(FULL_FIELD_WIDTH/4.0), get95CF(FULL_FIELD_WIDTH/4.0), get95CF(M_PI / 8.0),
                     get95CF(10.0*FULL_FIELD_WIDTH), get95CF(10.0*FULL_FIELD_WIDTH),
                     get95CF(10000.0), get95CF(10000.0);
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   for (unsigned i = 0; i < modes.size(); i++) delete modes[i];
   modes.clear();
   modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0, mean, diagonalVariance));
}

void MultiGaussianDistribution::resetToManualPlacement(int myPlayerNumber, bool isMyKickOff) {
   //TODO: Check to see if we are kickoff team
   Eigen::MatrixXd mean(MAIN_DIM, 1);
   mean = modes.front()->getMean();

   mean(2, 0) = 0;

   Eigen::MatrixXd diagonalVariance(MAIN_DIM, 1);
   diagonalVariance << get95CF(FULL_FIELD_LENGTH/4.0), get95CF(FULL_FIELD_WIDTH/4.0), get95CF(M_PI / 4.0),
                       get95CF(10.0*FULL_FIELD_LENGTH), get95CF(10.0*FULL_FIELD_WIDTH),
                       get95CF(10000.0), get95CF(10000.0);
   // Teammates
   MatrixXd initialRobotDiagonalVariance(3, 1);
   initialRobotDiagonalVariance << get95CF(FULL_FIELD_LENGTH), get95CF(FULL_FIELD_WIDTH), get95CF(M_PI);
   for (int i = 0; i < ROBOTS_PER_TEAM - 1; i++) {
      int j = MAIN_DIM_EXCLUDING_ROBOTS_PER_TEAM + 3 * i;
      diagonalVariance.block(j, 0, 3, 1) = initialRobotDiagonalVariance;
   }
   modes.clear();

   if (myPlayerNumber == 1) {
      mean(0, 0) = -4250;
      mean(1, 0) = 0;

      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));
   }
   else {
      mean(0, 0) = -3666;
      mean(1, 0) = 464;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

      mean(1, 0) *= -1;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

      mean(1, 0) = 1998;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

      mean(1, 0) *= -1.0;
      modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));

      if (isMyKickOff) {
        mean(0, 0) = -910;
        mean(1, 0) = 0;
        modes.push_back(new SimpleGaussian(MAIN_DIM, 1.0/4.0, mean, diagonalVariance));
      }
   }
}

void MultiGaussianDistribution::setLineUpMode(bool enabled) {
   for (unsigned i = 0; i < modes.size(); i++) {
      modes[i]->setLineUpMode(enabled);
   }
}

void MultiGaussianDistribution::setReadyMode(bool enabled) {
   isInReadyMode = enabled;
   if (!isInReadyMode) {
      numVisionUpdatesInReady = 0;
      haveSeenLandmarks = false;
   }

   for (unsigned i = 0; i < modes.size(); i++) {
      modes[i]->setReadyMode(enabled);
   }
}

void MultiGaussianDistribution::startOfPlayReset(void) {
   // All other modes other than the best mode that are in the opponent half are zeroed out.
   for (unsigned i = 1; i < modes.size(); i++) {
      if (modes[i]->getRobotPose().x() > 0.0) {
         modes[i]->setWeight(0.0);
      }
   }

   AbsCoord robotPose = modes.front()->getRobotPose();

   // If we are way inside the opponent half or we are just inside the opponent half and think we
   // are facing our own goal, then we are symmetrically flipped. NOTE: we dont want to blindly
   // zero out all modes that are in the opposing half because it is possible that we are
   // actually in our own half but due to noise the state estimation places us just over the
   // halfway line. This is quite possible for robots standing near the halfway line.
   if (robotPose.x() > 1000.0 || (robotPose.x() > 0.0 && fabs(robotPose.theta()) > M_PI/2.0)) {
      SimpleGaussian *flippedMode = modes.front()->createSymmetricGaussian();
      modes.front()->setWeight(0.0);
      modes.push_back(flippedMode);
   }

   fixupDistribution();
}

void MultiGaussianDistribution::fallenOverReset(int leftRight, int forwardBackward) {
   Eigen::MatrixXd mean(MAIN_DIM, 1);

   // 1  - forward
   // -1 - backward
   // 1  - left
   // -1 - right

   // iterate through all hypothesis and update heading
   std::vector<SimpleGaussian*>::iterator it;
   for (it = modes.begin(); it != modes.end(); ++it) {
	   mean = (*it)->getMean();
	   double heading = mean(2, 0);

	   if (leftRight == 1) {
	      if (forwardBackward == 1) {
		 heading = normaliseTheta(heading + M_PI_2);
		 std::cout << "Left, forward\n";
	      }
	      else if (forwardBackward == -1) {
		 heading = normaliseTheta(heading - M_PI_2);
		 std::cout << "Left, backward\n";
	      }
	   }
	   else if (leftRight == -1) {
	      if (forwardBackward == 1) {
		 heading = normaliseTheta(heading - M_PI_2);
		 std::cout << "Right, forward\n";
	      }
	      else if (forwardBackward == -1) {
		 heading = normaliseTheta(heading + M_PI_2);
		 std::cout << "Right, backward\n";
	      }
	   }

         mean(2, 0) = normaliseTheta(heading);
         (*it)->resetMean(mean);
   }
}

void MultiGaussianDistribution::processUpdate(const Odometry &odometry, const double dTimeSeconds,
      const bool canSeeBall) {
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ processUpdate start");
   for (unsigned i = 0; i < modes.size(); i++) {
      modes[i]->processUpdate(odometry, dTimeSeconds, canSeeBall);
   }
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ processUpdate end");
}

void MultiGaussianDistribution::visionUpdate(const VisionUpdateBundle &visionBundle) {
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ visionUpdate start");

   teamBallTracker.decay();
   haveSeenLandmarks = visionBundle.posts.size() > 0;

   if (isInReadyMode) {
      numVisionUpdatesInReady++;
   }

   lastObservationLikelyhood = -1.0;

   std::vector<SimpleGaussian*> allNewModes;
   for (unsigned i = 0; i < modes.size(); i++) {
      std::vector<SimpleGaussian*> newModes = modes[i]->visionUpdate(visionBundle);
      allNewModes.insert(allNewModes.end(), newModes.begin(), newModes.end());

      if (i == 0 && (visionBundle.fieldFeatures.size() > 0 || visionBundle.posts.size() > 0 ||
            visionBundle.visibleBalls.size() > 0)) {
         for (unsigned j = 0; j < newModes.size(); j++) {
            double weightAdjustment = newModes[j]->getWeight();
            if (weightAdjustment > lastObservationLikelyhood) {
               lastObservationLikelyhood = weightAdjustment;
            }
         }
      }
   }

   modes.insert(modes.end(), allNewModes.begin(), allNewModes.end());

   // This is a bit of a dodgy hack that performs ICP if we are in "initial state". Initial state refers
   // to the first few frames after booting up. We want to do this to better disambiguate which side of the
   // field we are on so we dont flip sides during the very first ready state.

   /*
   // VictorW:
   if (isInInitialState()) {
   //if (isInInitialState() && haveSeenLandmarks) {
      for (unsigned i = 0; i < modes.size(); i++) {
         if (modes[i]->getHaveLastVisionUpdate()) {
            modes[i]->doICPUpdate(visionBundle, true);
         }
      }
   }

   // Make sure that the goalie is always in its own half.
   if (playerNumber == 1) {
      for (unsigned i = 0; i < modes.size(); i++) {
         if (modes[i]->getRobotPose().x() > 0.0) {
            modes[i]->setWeight(modes[i]->getWeight() * 0.00001);
         }
      }
   }

   fixupDistribution();

   // Perform ICP on the top mode. We perform the ICP update and see how well it matches the current
   // state by checking the weight of the update. If the weight is below some threshold, then the
   // ICP update is probably invalid and it is rejected.
   // NOTE: we do this last bit only because we cannot afford to perform ICP on all modes. If we could,
   // then we would simply perform ICP on all modes with their weights being updated.
   if (!isInInitialState()) {
      if (!modes.empty() && modes.front()->getHaveLastVisionUpdate()) {
         SimpleGaussian *noICPMode = modes.front()->createSplitGaussian();
         double invalidICPProbability =
               constantsProvider.get(LocalisationConstantsProvider::INVALID_ICP_PROBABILITY);

         double icpWeight = modes.front()->doICPUpdate(visionBundle, false);
         if (icpWeight < invalidICPProbability) {
            modes.front()->setWeight(0.0);
            modes.push_back(noICPMode);
         } else {
            delete noICPMode;
         }

         fixupDistribution();
      }
   }
   */
   fixupDistribution();

   if (modes.empty()) {
      std::cout << "Distribution is empty! This is bad, resetting to initial state to recover" << std::endl;
      resetDistributionToPenalisedPose();
   }

   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ visionUpdate end");
}

void MultiGaussianDistribution::applyRemoteUpdate(
      const BroadcastData &broadcastData, int teammateIndex, bool isFromGoalie) {
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ remoteUpdate start");

   // VWong: Don't think our system is robust enough (has enough unaliased features) to risk self flipping)
   /*
   if (broadcastData.sharedLocalisationBundle.haveBallUpdates) {
      addSymmetricMode(modes);
   }
   */

   // VWong: Apply updates in place rather than appending updates in new modes. New information about ball / team-mates should be more valuable than what we currently have
   //std::vector<SimpleGaussian*> allNewModes;
   bool amIGoalie = (playerNumber == 1);
   for(unsigned i = 0; i < modes.size(); i++) {
      modes[i]->applyRemoteUpdate(
      //std::vector<SimpleGaussian*> newModes = modes[i]->applyRemoteUpdate(
            broadcastData.sharedLocalisationBundle, teammateIndex, amIGoalie);

      //allNewModes.insert(allNewModes.end(), newModes.begin(), newModes.end());
   }
   //modes.insert(modes.end(), allNewModes.begin(), allNewModes.end());

   if (!amIGoalie) { // We dont want teammates to try flipping the goalie.
      teamBallTracker.addTeammateObservation(broadcastData);
      for (unsigned i = 0; i < modes.size(); i++) {
         if (teamBallTracker.isModeFlipped(*modes[i])) {
            if (i == 0) {
               SAY("Teammates flipping me");
            }

            double weightScale = constantsProvider.get(LocalisationConstantsProvider::TEAMMATE_FLIP_WEIGHT);
            modes[i]->setWeight(modes[i]->getWeight() * weightScale);
         }
      }
   }

   fixupDistribution();
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ remoteUpdate end");
}

const SimpleGaussian* MultiGaussianDistribution::getTopGaussian(void) const {
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ getTopGaussian");
   return modes[0];
}

std::vector < SimpleGaussian* > MultiGaussianDistribution::getAllModes(void) const {
   MY_ASSERT(checkValidDistribution(modes), "invalid distribution @ getTopGaussian");
   return modes;
}

double MultiGaussianDistribution::getLastObservationLikelyhood(void) const {
   return lastObservationLikelyhood;
}

void MultiGaussianDistribution::doTeammateRobotVisionUpdate(const VisionUpdateBundle &visionBundle) {
   std::vector<SimpleGaussian*> allNewModes;
   for (unsigned i = 0; i < modes.size(); i++) {
      std::vector<SimpleGaussian*> newModes = modes[i]->visionTeammateRobotsUpdate(visionBundle);
      allNewModes.insert(allNewModes.end(), newModes.begin(), newModes.end());
   }

   modes.insert(modes.end(), allNewModes.begin(), allNewModes.end());
   fixupDistribution();
}

bool MultiGaussianDistribution::isInInitialState(void) {
   return isInReadyMode;// && numVisionUpdatesInReady < 120;
}

void MultiGaussianDistribution::fixupDistribution(void) {
   std::sort(modes.begin(), modes.end(), GuassianSortFunction());
   normaliseDistribution(modes);
   mergeSimilarModes(modes);
   std::sort(modes.begin(), modes.end(), GuassianSortFunction());
   normaliseDistribution(modes);
   removeUnlikelyModes(modes);
   removeExcessModes(modes, maxGaussians);
   normaliseDistribution(modes);
}

void MultiGaussianDistribution::normaliseDistribution(const std::vector<SimpleGaussian*> &distribution) {
   double weightSum = 0.0;
   for (unsigned i = 0; i < distribution.size(); i++) {
      weightSum += distribution[i]->getWeight();
   }

   // Make sure we dont divide by zero.
   double scale = 1.0 / weightSum;
   for (unsigned i = 0; i < distribution.size(); i++) {
      distribution[i]->setWeight(distribution[i]->getWeight() * scale);
   }
}

void MultiGaussianDistribution::mergeSimilarModes(const std::vector<SimpleGaussian*> &distribution) {
   for (unsigned i = 0; i < distribution.size() - 1; i++) {
      if (distribution[i]->getWeight() > 0.0) {
         std::vector<SimpleGaussian*> similarGaussians;

         for (unsigned j = i + 1; j < distribution.size(); j++) {
            if (distribution[i]->isSimilarTo(*distribution[j])) {
               similarGaussians.push_back(distribution[j]);
            }
         }

         for (unsigned j = 0; j < similarGaussians.size(); j++) {
            distribution[i]->mergeWith(*similarGaussians[j]);
         }
      }
   }
}

void MultiGaussianDistribution::removeUnlikelyModes(std::vector<SimpleGaussian*> &distribution) {
   MY_ASSERT(checkValidDistribution(distribution), "invalid distribution @ removeUnlikelyModes");

   double minWeight = constantsProvider.get(LocalisationConstantsProvider::MIN_MODE_WEIGHT);
   while (distribution.back()->getWeight() < minWeight && distribution.size() > 1) {
      delete distribution.back();
      distribution.pop_back();
   }
}

void MultiGaussianDistribution::removeExcessModes(
      std::vector<SimpleGaussian*> &distribution, unsigned maxGaussians) {
   MY_ASSERT(checkValidDistribution(distribution), "invalid distribution @ removeExcessModes");

   while (distribution.size() > maxGaussians) {
      delete distribution.back();
      distribution.pop_back();
   }
}

void MultiGaussianDistribution::addBaselineModes(std::vector<SimpleGaussian*> &distribution) {
   std::vector<SimpleGaussian*> baseline = SimpleGaussian::createBaselineGaussians();
   distribution.insert(distribution.end(), baseline.begin(), baseline.end());
}

void MultiGaussianDistribution::addSymmetricMode(std::vector<SimpleGaussian*> &distribution) {
   AbsCoord ballPos = distribution.front()->getBallPosition();
   double fromCentre = sqrt(ballPos.x()*ballPos.x() + ballPos.y()*ballPos.y());

   // If the ball is close to the centre then a symmetric mode will be too close to the original mode
   // and hence provide spurious matches potentially.
   if (fromCentre >= 1300.0) { // TODO: make this a constant in the ConstantsProvider
      SimpleGaussian *symmetric = distribution.front()->createSymmetricGaussian();
      distribution.push_back(symmetric);
   }
}

bool MultiGaussianDistribution::checkValidDistribution(
      const std::vector<SimpleGaussian*> &distribution) {
   if (distribution.size() == 0) {
      std::cout << "Invalid distribution, empty" << std::endl;
      return false;
   }

   for (unsigned i = 0; i < distribution.size(); i++) {
      if (distribution[i]->getWeight() < 0.0 || distribution[i]->getWeight() > 1.0) {
         std::cout << "Invalid distribution, weight invalid: " << distribution[i]->getWeight() << std::endl;
         return false;
      }

      // Assert that the modes are ordered in descending order of weight.
      if (i < (distribution.size() - 1) &&
          distribution[i]->getWeight() < distribution[i + 1]->getWeight()) {
         std::cout << "Invalid distribution, unordered" << std::endl;
         return false;
      }
   }

   return true;
}

void MultiGaussianDistribution::printModes() const {

   std::vector<SimpleGaussian*>::const_iterator it;
   int i = 0;
   for (it = modes.begin(); it != modes.end(); ++it) {
      std::cout << i << " : " << (*it)->getRobotPose().x() << ", "
                << (*it)->getRobotPose().y() << ", "
                << (*it)->getRobotPose().theta() << " - "
                << (*it)->getWeight() << " | ";
                std::cout << std::endl;
      ++i;
   }
   std::cout << std::endl;
}
