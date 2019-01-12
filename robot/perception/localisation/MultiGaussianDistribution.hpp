#pragma once

#include "SimpleGaussian.hpp"
#include "VisionUpdateBundle.hpp"
#include "TeamBallTracker.hpp"
#include "types/Odometry.hpp"
#include "types/BroadcastData.hpp"

#include <vector>

/**
 * Class for representing a multi-modal distribution. Each mode is a Gaussian. We restrict the
 * maximum number of modes to a maximum of a fixed value. Each mode has an associated weight
 * representing the confidence that it corresponds to the true state.
 */
class MultiGaussianDistribution {
public:
   explicit MultiGaussianDistribution(unsigned maxGaussians, int playerNumber, const AbsCoord* initialPose = NULL);
   virtual ~MultiGaussianDistribution();

   /**
    * Resets the distribution to have the robot be in its own half on the field edge looking in.
    * This is basically the pose we expect the robot to have when it resumes playing after being
    * penalised.
    */
   void resetDistributionToPenalisedPose(void);
   void setInitialPose(AbsCoord initialPose);
   void setInitialGamePose(int playerNumber);

    /**
     * Resets the distribution for the Penalty Kick Challenge possible initial positions according
     * to 2018 rulebook
     */
   void resetToPenaltyKickChallenge(bool);
   /**
    * Resets the distribution for the start of a penalty shootout. Will reset to a different pose
    * depending on whether the current robot is a goalie or not.
    */
   void resetToPenaltyShootout(bool);

   /**
    * Resets the distribution to manual placement positions during the ready skill
    */
   void resetToManualPlacement(int myPlayerNumber, bool isMyKickOff);

   /**
    * When the robot is lining up with the ball to kick it, we set this mode to true. In this mode
    * we disregard the ball's velocity for the process update, and use a lower certainty decay for
    * the ball's position. All of this is to make the ball position estimate more stable, as we care
    * more about stability than reactive tracking when lining up.
    *
    * NOTE: its questionable whether we actually need this, it was included mostly because a similar
    * mode existed in last years localisation, and we didnt have time to investigate whether there is
    * actually a point to doing this.
    */
   void setLineUpMode(bool enabled);

   /**
    * In ready mode we perform additional computation to help us localise, since it is not that
    * bad if we drop frames in ready. Call this method at the start of ready with true, and
    * we transitioning out of ready call this with false.
    */
   void setReadyMode(bool enabled);

   /**
    * This function should be called at the time when we transition from SET into PLAY. What it does
    * is to remove all modes that put the robot in the opponent's half, since we know it is impossible
    * to start in the opponents half (we would be manually placed even if we were mislocalised).
    */
   void startOfPlayReset(void);

   /**
    * This function should be called whenever we transition from a non-getting up state to a getting up state
    * implying we have fallen over. When we have fallen over, we refresh the modes based on the top estimate
    * creating hypotheses in all directions since when we fall over, we are unsure about where we are facing
    */

   void fallenOverReset(int leftRight, int forwardBackward);

   /**
    * Perform the process update on the state estimate. This shifts out state estimate robot pose by
    * the given odometry amount, shifts the ball's position according to its current velocity, and
    * increases the uncertainty of our entire state estimate.
    */
   void processUpdate(const Odometry &odometry, const double dTimeSeconds, const bool canSeeBall);

   /**
    * Perform the vision update with the given observed landmarks and vision features.
    */
   void visionUpdate(const VisionUpdateBundle &visionBundle);

   /**
    * Update out state estimate by incorporating the state estimate from our teammate.
    */
   void applyRemoteUpdate(const BroadcastData &broadcastData, int teammateIndex, bool isFromGoalie);

   /**
    * Returns the top mode of the distribution. This is the mode that we think most accurately
    * models the current state of the world.
    */
   const SimpleGaussian* getTopGaussian(void) const;
   std::vector < SimpleGaussian* > getAllModes(void) const;

   // This is for debugging and logging purposes.
   double getLastObservationLikelyhood(void) const;

   // This is for debugging and logging purposes.
   void printModes() const;

private:

   const unsigned maxGaussians;
   const int playerNumber;

   // The list of Gaussian modes that makes up the multi-modal distribution. These are ordered by
   // weight in decreasing order. The highest weighted mode is first.
   std::vector<SimpleGaussian*> modes;

   double lastObservationLikelyhood;

   bool isInReadyMode;
   bool haveSeenLandmarks;
   unsigned numVisionUpdatesInReady;
   TeamBallTracker teamBallTracker;

   void doTeammateRobotVisionUpdate(const VisionUpdateBundle &visionBundle);
   bool isInInitialState(void);
   void fixupDistribution(void);

   /**
    * Normalises the distribution so that the weights all add up to 1.0
    */
   static void normaliseDistribution(const std::vector<SimpleGaussian*> &distribution);

   /**
    * Takes Gaussians with similar means and covariances and merges them
    */
   static void mergeSimilarModes(const std::vector<SimpleGaussian*> &distribution);

   /**
    * Removes Gaussians with very low weights from the distribution. Distribution may be
    * non-normalised after this method is called.
    */
   static void removeUnlikelyModes(std::vector<SimpleGaussian*> &distribution);

   /**
    * Enforces the maximum number of Gaussians constraint over the distribution.
    */
   static void removeExcessModes(std::vector<SimpleGaussian*> &distribution, unsigned maxGaussians);

   /**
    * Adds a baseline mode to the distribution. This is a mode with large covariance and a mean
    * in the middle of the field, and a very small weight. This is used to indicate that the
    * robot is completely lost. It's useful for when an observation strongly disagrees with all
    * other modes, indicating that they are all invalid.
    */
   static void addBaselineModes(std::vector<SimpleGaussian*> &distribution);

   static void addSymmetricMode(std::vector<SimpleGaussian*> &distribution);

   static bool checkValidDistribution(const std::vector<SimpleGaussian*> &distribution);
};
