#pragma once

#include "types/Odometry.hpp"
#include "types/BallInfo.hpp"
#include "SimpleGaussian.hpp"
#include "SharedLocalisationUpdateBundle.hpp"

/**
 * The SharedDistribution is the distribution that encapsulates the updates that are sent to
 * our teammates so that they can incorporate them into their state estimation. This
 * distribution consists of a single mode, and as such only accepts UniModalVisionUpdates. The
 * way that we get a uni-modal vision update from a normal vision update is to take the
 * UniModalVisionUpdate from the top weighted mode from the main distribution.
 * Another caveat of the SharedDistribution is that we reset it every time it is broadcast
 * to our teammates. This is done to prevent 'double counting' an observation. IF we did not
 * reset, then when our teammates receive the update and apply it, it would in effect be applying
 * the same observation multiple times.
 */
class SharedDistribution {
public:
   SharedDistribution();
   virtual ~SharedDistribution();

   void processUpdate(const Odometry &odometry, const double dTimeSeconds, const bool canSeeBall);

   void visionUpdate(const StoredICPUpdate &icpUpdate,
    const UniModalVisionUpdate &vu,
    int localiserBallSeenCount);

   /**
    * Returns the data bundle to broadcast to our teammates.
    */
   SharedLocalisationUpdateBundle getBroadcastData(void);

   /**
    * Reset the distribution. This basically sets the covariance of the state estimation to be
    * very high and takes the mean of the given best distribution from the main distribution.
    * This should be called every time we broadcast to our teammates the SharedDistribution.
    */
   void reset(const SimpleGaussian *topDistribution);
   void setReadyMode(bool enabled);

private:
   unsigned ballSeenCount;
   unsigned filteredBallSeenCount;
   unsigned framesCount;

   bool haveVisionUpdates;
   SimpleGaussian *sharedGaussian;

   double sharedDx;
   double sharedDy;
   double sharedDh;

   double sharedCovarianceDx;
   double sharedCovarianceDy;
   double sharedCovarianceDh;

};
