#pragma once

#include <string>
#include <fstream>

#include "blackboard/Adapter.hpp"
#include "types/Odometry.hpp"
#include "types/ActionCommand.hpp"
#include "VisionUpdateBundle.hpp"
#include "robotfilter/RobotFilter.hpp"
#include "ballfilter/BallFilter.hpp"

/* Forward declarations */
class Localiser;
class RobotFilter;
class BallFilter;

/* Adapter that allows Localisation to communicate with the Blackboard */
class LocalisationAdapter : Adapter {
   public:
      /* Constructor */
      LocalisationAdapter(Blackboard *bb);
      /* Destructor */
      ~LocalisationAdapter();
      /* One cycle of this thread */
      void tick();

      /* Bit of a hack: this gets the filtered robots
         for vision before vision happens*/
      std::vector<RobotObstacle> getRobotsFromFilter(void);



   private:
      /* Filter module instances */
      Localiser *L;
      RobotFilter *robotFilter;
      BallFilter *ballFilter;
      bool firstCycle;
      int playerNumber;
      int teamNumber;
      Odometry prevOdometry;
      int64_t prevTimestamp;
      uint8_t prevGameState;
      bool isGettingUp;
      int leftRight;
      int forwardBackward;

      bool isInPenaltyShootout;

      bool penalised_before_set;

      std::ofstream logStream;

      bool canLocaliseInState(uint8_t state, std::string skill);
      bool canDoObservations(void);

      bool haveTransitionedSetToPlay(void);
      bool haveTransitionedIntoSet(void);
      std::vector<AbsCoord> getTeammatePoses(void);

      bool amWalking(const ActionCommand::All &commands);
      bool amTurningHead(const ActionCommand::All &commands);

      void writeResultToBlackboard(void);
      void handleMySharedDistribution(void);
      void handleIncomingSharedUpdate(void);

      // The below methods/variables are used for calibration purposes.
      std::vector<double> distanceObservations;
      std::vector<double> headingObservations;
      int numFramesSinceSeen;
      int isPickedUpHysteresis;
      int isPenalisedHysteresis;

      void handleObservationMeasurements(const VisionUpdateBundle &visionUpdateBundle);
      void outputVariances(void);

      double getMean(const std::vector<double> &vals, double outliersRatio);
      double getVariance(const std::vector<double> &vals, double mean, double outliersRatio);
};
