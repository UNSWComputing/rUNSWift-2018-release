#include "LocalisationConstantsProvider.hpp"
#include "utils/basic_maths.hpp"
#include "utils/angles.hpp"
#include <cmath>
#include <iostream>

LocalisationConstantsProvider::LocalisationConstantsProvider() :
      keyValues(static_cast<int>(NUM_CONSTANTS)) {

   set(MEAN_DIST_SIMILARITY_THRESHOLD, 200.0);
   set(MEAN_DIST_SIMILARITY_THRESHOLD_NEAR_GOALS, 100.0);

   set(MEAN_HEADING_SIMILARITY_THRESHOLD, DEG2RAD(20.0));
   set(MEAN_HEADING_SIMILARITY_THRESHOLD_NEAR_GOALS, DEG2RAD(10.0));

   set(COVARIANCE_DIST_SIMILARITY_THRESHOLD, 400000.0);
   set(BALL_FRICTION, 0.5); // VWong
   //set(BALL_FRICTION, 0.98); // 0.9

   // VWong: Reduce the drift of our estimates
   set(ROBOT_POS_MOTION_UPDATE_COVARIANCE_A, 100.00);
   set(ROBOT_POS_MOTION_UPDATE_COVARIANCE_C, 1000);
   set(ROBOT_HEADING_MOTION_UPDATE_COVARIANCE_A, 3.11);
   set(ROBOT_HEADING_MOTION_UPDATE_COVARIANCE_C, 0.003); // This is made up. It's about 5 degrees? Used to be 17 - craaazy. Comp 2017 Hayden Smith

   /* Original
   set(ROBOT_POS_MOTION_UPDATE_COVARIANCE_A, 2.75);
   set(ROBOT_POS_MOTION_UPDATE_COVARIANCE_C, 85437.3);
   set(ROBOT_HEADING_MOTION_UPDATE_COVARIANCE_A, 3.11);
   set(ROBOT_HEADING_MOTION_UPDATE_COVARIANCE_C, 0.8);
   */

   // Vwong
   set(BALL_POS_MOTION_UPDATE_COVARIANCE_C, 1000.0); // 344290
   set(BALL_UNSEEN_POS_MOTION_UPDATE_COVARIANCE_C, 1000.0); // 4000000 // 2m per second
   set(BALL_POS_MOTION_UPDATE_LINE_UP_COVARIANCE_C, 150000.0); // ~40cm per second
   set(BALL_VEL_MOTION_UPDATE_COVARIANCE_C, 361396.0);

   set(REMOTE_OBSERVATION_UNCERTAINTY_FACTOR, 3.0);
   set(SHARED_GAUSSIAN_RESET_UNCERTAINTY_FACTOR, 1.0);
   set(TEAMMATE_ODOMETRY_UNCERTAINTY_SCALE, 4.0);

   // ORIGINAL
   /*
   set(BALL_POS_MOTION_UPDATE_COVARIANCE_C, 244290.0); // 344290
   set(BALL_UNSEEN_POS_MOTION_UPDATE_COVARIANCE_C, 6000000.0); // 4000000 // 2m per second
   set(BALL_POS_MOTION_UPDATE_LINE_UP_COVARIANCE_C, 150000.0); // ~40cm per second
   set(BALL_VEL_MOTION_UPDATE_COVARIANCE_C, 361396.0);

   set(REMOTE_OBSERVATION_UNCERTAINTY_FACTOR, 4.0);
   set(SHARED_GAUSSIAN_RESET_UNCERTAINTY_FACTOR, 1000.0);
   set(TEAMMATE_ODOMETRY_UNCERTAINTY_SCALE, 4.0);
   */

   set(LIKELY_MODE_WEIGHT_ADJUSTMENT, 1);
   //set(LIKELY_MODE_WEIGHT_ADJUSTMENT, 1.005);

   set(INVALID_OBSERVATION_PROBABILITY, 0.3); //0.3
   set(INVALID_ICP_PROBABILITY, 0.8);
   set(INVALID_REMOTE_OBSERVATION_PROBABILITY, 0.5);
   set(INVALID_REMOTE_OBSERVATION_PROBABILITY_GOALIE, 0.8);
   set(INVALID_TEAMMATE_ROBOT_OBSERVATION_PROBABILITY, 0.8);

   set(MIN_MODE_WEIGHT, 0.001);

   set(SYMMETRIC_MODE_WEIGHT, 0.002);
   set(TEAMMATE_FLIP_WEIGHT, 0.01);

   set(UNRELIABLE_DISTANCE_CUTOFF_FACTOR, 0.66);
   set(UNRELIABLE_DISTANCE_VARIANCE_SCALE, 4.0);
   set(UNRELIABLE_HEADING_VARIANCE_SCALE, 2.0);

   set(BALL_MAX_DISTANCE_OBSERVATION, 4000.0);
   set(GOALPOST_MAX_DISTANCE_OBSERVATION, 4000.0); // 4000.0
   set(CENTRE_CIRCLE_MAX_DISTANCE_OBSERVATION, 4000.0);
   set(TEAMMATE_ROBOT_MAX_DISTANCE_OBSERVATION, 3000.0);

   set(HEADING_VARIANCE, 0.3);
   set(DISTANCE_VARIANCE_B, 0.00144844);
   set(DISTANCE_VARIANCE_C, 9.04618);

   set(TEAMMATE_ROBOT_HEADING_VARIANCE, 0.8);
   set(TEAMMATE_ROBOT_DISTANCE_VARIANCE_SCALE, 9.0);

   // VictorW:
   set(ICP_DISTANCE_VARIANCE_SCALE, 5000.0);
   set(ICP_HEADING_VARIANCE, 0.4);
   /* OLD
   set(ICP_DISTANCE_VARIANCE_SCALE, 1.0);
   set(ICP_HEADING_VARIANCE, 0.4);
   */

   set(ACCEPTABLE_OFF_FIELD_ERROR_MARGIN, 3000.0); // Excessive number to "turn it off"
}

LocalisationConstantsProvider& LocalisationConstantsProvider::instance(void) {
   static LocalisationConstantsProvider provider;
   return provider;
}

double LocalisationConstantsProvider::get(LocalisationConstant key) const {
   return keyValues[static_cast<int>(key)];
}

void LocalisationConstantsProvider::set(LocalisationConstant key, double value) {
   keyValues[static_cast<int>(key)] = value;
}

void LocalisationConstantsProvider::setReadyMode(bool enabled) {
   // Because I cbf writing this properly anymore, 4 days to go... TODO: fix this ugly hack.
   if (enabled) {
      set(UNRELIABLE_DISTANCE_CUTOFF_FACTOR, 0.5);
      set(GOALPOST_MAX_DISTANCE_OBSERVATION, 5000.0);
   } else {
      set(UNRELIABLE_DISTANCE_CUTOFF_FACTOR, 0.66);
      set(GOALPOST_MAX_DISTANCE_OBSERVATION, 4000.0);
   }
}
