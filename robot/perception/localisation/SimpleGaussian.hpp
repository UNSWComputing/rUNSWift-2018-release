#pragma once

#include "GaussianUpdateDebugData.hpp"
#include "PostType.hpp"
#include "ObservedPostsHistory.hpp"
#include "SharedLocalisationUpdateBundle.hpp"
#include "VisionUpdateBundle.hpp"
#include "VarianceProvider.hpp"
#include "ICP.hpp"
#include "LocalisationDefs.hpp"
#include "types/AbsCoord.hpp"
#include "types/Odometry.hpp"

#include <cassert>
#include <Eigen/Eigen>


/**
 * A normal vision update is inherently multi-modal. For example, in a vision bundle if
 * we have a goal post object, it is not defined which of the 4 posts it corresponds to,
 * as they all look the same. What we do is generate multiple UniModalVisionUpdates from a
 * given VisionUpdateBundle that cover all of the possible correspondences from observation to
 * field landmark.
 */
struct UniModalVisionUpdate {
   UniModalVisionUpdate() {}

   UniModalVisionUpdate(const VisionUpdateBundle &visionBundle);

   UniModalVisionUpdate(const VisionUpdateBundle &visionBundle,
         const PostType &postType);

   UniModalVisionUpdate(const VisionUpdateBundle &visionBundle,
         const std::vector<PostType> &postTypes);

   VisionUpdateBundle visionBundle;
   std::vector<PostType> postTypes;
};


struct UniModalTeammateUpdate {
   UniModalTeammateUpdate(const VisionUpdateBundle &visionBundle,
         const std::vector<RobotInfo> &teammateRobots, const AbsCoord &teammatePosition);

   UniModalTeammateUpdate(const VisionUpdateBundle &visionBundle,
         const std::vector<RobotInfo> &teammateRobots, const std::vector<AbsCoord> &teammatePositions);

   VisionUpdateBundle visionBundle;
   std::vector<RobotInfo> teammateRobots;
   std::vector<AbsCoord> teammatePositions;
};


struct StoredICPUpdate {
   StoredICPUpdate() : updateDimension(0) {}

   StoredICPUpdate(int updateDimension, const Eigen::MatrixXd &innovation,
         const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &observationVariance) :
            updateDimension(updateDimension), innovation(innovation),
            jacobian(jacobian), observationVariance(observationVariance) {}

   int updateDimension;
   Eigen::MatrixXd innovation;
   Eigen::MatrixXd jacobian;
   Eigen::MatrixXd observationVariance;
};


struct OdometryUpdateResult {
   double dx, dy, dh;
   double covDx, covDy, covDh;
};

/**
 * A SimpleGaussian represents a single mode of a multi-model distribution. Each Gaussian mode
 * has an associated mean state, covariance matrix, and a weight. The weight represents the
 * confidence of the mode and is used as part of an overall distribution which is represented as
 * a weighted sum of Gaussians.
 */
class SimpleGaussian {
public:

   /**
    * Static constructor function to create baseline Gaussians with very high variance and low
    * weight.
    */
   static std::vector<SimpleGaussian*> createBaselineGaussians(void);
   static SimpleGaussian* createBaselineSharedGaussian(void);

   /**
    * Set the mean to the given vector.
    */
   void resetMean(const Eigen::MatrixXd &src);

   /**
    * Sets the covariance matrix to the given matrix.
    */
   void resetCovariance(const Eigen::MatrixXd &src);

   /**
    * Sets whether the robot is currently lining up to kick the ball. This is basically
    * an indication to the filter to disgregard ball velocity when calculating its position
    * to get a more stable, but potentially less accurate, result. See the corresponding method
    * in MultiGaussianDistribution for more info.
    */
   void setLineUpMode(bool enabled);

   /**
    * Enables/Disables the Ready mode. For more info, see the corresponding method in
    * MultiGaussianDistribution.
    */
   void setReadyMode(bool enabled);

   /**
    * Performs the odometry update on the Gaussian.
    */
   OdometryUpdateResult processUpdate(const Odometry &odometry, const double dTimeSeconds,
         const bool canSeeBall);

   /**
    * Performs a vision update on this Gaussian, and returns a vector of newly generated hypotheses
    * which are in addition to this Gaussian. These should be inserted back into the distribution.
    */
   std::vector<SimpleGaussian*> visionUpdate(const VisionUpdateBundle &visionBundle);
   std::vector<SimpleGaussian*> visionTeammateRobotsUpdate(const VisionUpdateBundle &visionBundle);

   double doICPUpdate(const VisionUpdateBundle &visionBundle, const bool updateWeight);

   void uniModalBallVisionUpdate(const VisionUpdateBundle &visionBundle, const RRCoord &observedBallCoords);
   int uniModalVisionUpdate(const UniModalVisionUpdate &vu);
   void uniModalTeammateRobotVisionUpdate(const UniModalTeammateUpdate &vu);

   void applyRemoteUpdate(const SharedLocalisationUpdateBundle &updateBundle,
   //std::vector<SimpleGaussian*> applyRemoteUpdate(const SharedLocalisationUpdateBundle &updateBundle,
         int teammateIndex, bool amGoalie);

   double applyObservation(int obsDimension, const Eigen::MatrixXd &innovation,
         const Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &observationVariance,
         const bool updateWeight);

   /**
    * Returns the x,y,theta pose of the robot.
    */
   AbsCoord getRobotPose(void) const;

   /**
    * Returns the x,y ball world position
    */
   AbsCoord getBallPosition(void) const;

   /**
    * Returns the x',y' ball velocity in milli-metres per second.
    */
   AbsCoord getBallVelocity(void) const;

   /**
    * Returns a measure of uncertainty of the robots position estimation. This is the area
    * bound by the 1 standard deviation of the covariance matrix.
    */
   double getRobotPosUncertainty(void) const;

   /**
    * Returns the 1 standard deviation of the heading part of the covariance matrix.
    */
   double getRobotHeadingUncertainty(void) const;

   /**
    * Returns a measure of uncertainty of the ball position estimation. This is the area
    * bound by the 1 standard deviation of the covariance matrix.
    */
   double getBallPosUncertainty(void) const;

   /**
    * Returns a measure of uncertainty of the ball velocity estimation. This is the area
    * bound by the 1 standard deviation of the covariance mZatrix.
    */
   double getBallVelocityUncertainty(void) const;

   /**
    * Returns whether this and the other Gaussian are similar enough in their mean and variance to
    * merge.
    */
   bool isSimilarTo(const SimpleGaussian &other) const;

   /**
    * Combines this Gaussian with the other. The other Gaussian has its weight set to 0.
    */
   void mergeWith(SimpleGaussian &other);

   double getWeight(void) const;
   void setWeight(double weight);

   bool getHaveLastVisionUpdate(void) const;

   StoredICPUpdate getLastAppliedICPUpdate(void) const;
   UniModalVisionUpdate getLastAppliedVisionUpdate(void) const;

   Eigen::MatrixXd getMean(void) const;
   Eigen::MatrixXd getCovariance(void) const;

   void sanityCheck(void) const;

   enum FieldFeatureType {
        CENTRE_T_JUNCTION,
        GOAL_T_JUNCTION,
        OUTER_CORNER,
        GOAL_CORNER
   };

   /**
    * Creates a Gaussian that is a symmetric reflection of the current one.
    */
   SimpleGaussian* createSymmetricGaussian(void);

   /**
    * Constructs a Gaussian with the given mean, a covariance matrix with the given diagonal
    * values, and the given weight.
    */
   SimpleGaussian(
         const unsigned dim,
         const double weight,
         const Eigen::MatrixXd &mean,
         const Eigen::MatrixXd &diagonalVariance);

   /**
    * Creates a Gaussian that is a copy of the current one.
    */
   SimpleGaussian* createSplitGaussian(void);

   /**
    * Set of field feature candidates
	 *    Point pos - x y of field feature
	 *    int facesAway - does the feature face away from us? (Currently only used for corners)
	 *       Different views of the same feature (example goal corner) are less aliased (when it faces away)
	 *       compared to others (when it faces towards us), thus we have different thresholds for each case
	 *    double targetOrientation -
	 *       This is a measurement combining the relative heading of the object, the heading of the robot
	 *       and the orientation of the feature on a soccer field
	 *
	 *       Example:
	 *       Consider the bottom left corner. Based on where we are, if we draw the feature on the field's
	 *       co-ordinate plane (exactly how Overviewtab draws it on the field), we expect the corner drawn
	 *       to be oriented with an orientation of M_PI_4 (using our usual co-ordinate system)
	 *
	 *       NB: Orientation for a corner is defined by angle subtended by the angle bisecotr
	 *           Orientation for a T-Junction, this is defined by the angle subtended by the "vertical" part
	 *           of a "T"
	 *
	 *******************************************************************************************************
	 *                                   Thresholds (optional)
	 *
	 *    With a feature position, relative distance, relative heading and targetOrientation, we can
	 *    backsolve where the robot's precise (x, y, theta) should be in order to make such an observation
	 *
	 *    From this, we have an error associated with x, y and theta
	 *
	 *    For each dimension (and field feature (based on how aliased it is in its location), we have
	 *    different tolerances for error
	 *
	 *    HardThresh represents a hard threshold where we completely ignore features associated with error
	 *    that exceed our hardthreshold
	 *
	 *    SofThresh represents a soft threshold where any excess error beyond the softhreshold will linearly
	 *    increase the variance of the observation by a factor of VarianceLinearGrowth
	 *******************************************************************************************************
	 *    int xErrorSoftThresh_ -
	 *    int xErrorHardThresh_ -
	 *    int xVarianceLinearGrowth_ -
	 *    int yErrorSoftThresh_ -
	 *    int yErrorHardThresh_ -
	 *    int yVarianceLinearGrowth_ -
	 *    double thetaErrorSoftThresh_ -
	 *    double thetaErrorHardThresh_ -
	 *    int thetaVarianceLinearGrowth_ -
	 */

   class FieldFeatures {
	   public:
			FieldFeatures (Point pos, int facesAway, double targetOrientation) :
	   			pos_(pos), facesAway_(facesAway), targetOrientation_(targetOrientation),
	   			xErrorSoftThresh_(500), xErrorHardThresh_(1000), xVarianceLinearGrowth_(0.008),
	   			yErrorSoftThresh_(500), yErrorHardThresh_(1000), yVarianceLinearGrowth_(0.008),
	   			thetaErrorSoftThresh_(0.5 * M_PI_4),	thetaErrorHardThresh_(M_PI_4),
	   			thetaVarianceLinearGrowth_(10)
	   	{}

	   	FieldFeatures (Point pos, int facesAway,
	   		double targetOrientation, const int xErrorSoftThresh,
	   		const int xErrorHardThresh, const double xVarianceLinearGrowth, const int yErrorSoftThresh,
	   		const int yErrorHardThresh, const double yVarianceLinearGrowth, const double thetaErrorSoftThresh,
	   		const double thetaErrorHardThresh, const double thetaVarianceLinearGrowth) :
	   			pos_(pos), facesAway_(facesAway), targetOrientation_(targetOrientation),
	   			xErrorSoftThresh_(xErrorSoftThresh), xErrorHardThresh_(xErrorHardThresh), xVarianceLinearGrowth_(xVarianceLinearGrowth),
	   			yErrorSoftThresh_(yErrorSoftThresh), yErrorHardThresh_(yErrorHardThresh), yVarianceLinearGrowth_(yVarianceLinearGrowth),
	   			thetaErrorSoftThresh_(thetaErrorSoftThresh),	thetaErrorHardThresh_(thetaErrorHardThresh),
	   			thetaVarianceLinearGrowth_(thetaVarianceLinearGrowth)
	   	{}

	   	int ifCanObserveThenAddMeasurement(SimpleGaussian &pSG,
	   		const VisionUpdateBundle &visionBundle,
	   		const RRCoord &observedObjectCoords,
	   		Eigen::MatrixXd &innovationOut,
	   		Eigen::MatrixXd &jacobianOut,
	   		Eigen::MatrixXd &observationVarianceOut,
	   		const int currentMeasurement) const;

	   private:
	   	Point pos_;
	   	int facesAway_;
	   	double targetOrientation_;
	   	const int xErrorSoftThresh_;
	   	const int xErrorHardThresh_;
	   	const double xVarianceLinearGrowth_;

	   	const int yErrorSoftThresh_;
	   	const int yErrorHardThresh_;
	   	const double yVarianceLinearGrowth_;

	   	//TODO: Maybe add a total distance threshold

	   	const double thetaErrorSoftThresh_;
	   	const double thetaErrorHardThresh_;
	   	const double thetaVarianceLinearGrowth_;
   };

private:

   // The dimensionality of the state space. Should be either 7 or 19
   // DIM (0,1,2) => Robot x,y,theta world position.
   // DIM (3,4) => Ball x,y world position.
   // DIM (5,6) => Ball x,y world velocity.
   const unsigned DIM;
   const Eigen::MatrixXd identity;

   double weight;
   Eigen::MatrixXd mean; // DIM x 1
   Eigen::MatrixXd covariance; // DIM x DIM

   bool doingBallLineUp;
   bool isInReadyMode;
   bool haveLastVisionUpdate;
   UniModalVisionUpdate lastVisionUpdate;
   StoredICPUpdate storedICPUpdate;
   ObservedPostsHistory observedPostsHistory;

   // Basically a copy constructor but one that is used by createSplitGaussian and
   // creates a new id.
   SimpleGaussian(
         const unsigned dim,
         const double weight,
         const Eigen::MatrixXd &mean,
         const Eigen::MatrixXd &covariance,
         bool doingBallLineUp,
         bool isInReadyMode,
         const ObservedPostsHistory &observedPostsHistory);

   std::vector<PostInfo> getICPQualityPosts(const VisionUpdateBundle &visionBundle);

   bool canUsePostType(PostType type, double distance);
   bool canUsePostType(PostType type0, double distance0, PostType type1, double distance1);
   double distanceTo(const AbsCoord &coord);

   /**
    * Returns whether the given row index corresponds to a heading in the mean vector.
    */
   bool isHeadingRow(int row);

   /**
    * Returns an angle that corresponds to the given one, but is within the range [-pi, pi].
    */

   void addNoPostsObservationMode(const VisionUpdateBundle &visionBundle,
            std::vector<SimpleGaussian*> &outModes);
   void addOnePostObservationModes(const VisionUpdateBundle &visionBundle,
         std::vector<SimpleGaussian*> &outModes);
   void addTwoPostObservationModes(const VisionUpdateBundle &visionBundle,
         std::vector<SimpleGaussian*> &outModes);

   double performKalmanUpdate(
         const int observationDim,
         const Eigen::MatrixXd &innovation,
         const Eigen::MatrixXd &jacobian,
         const Eigen::MatrixXd &observationVariance,
         const bool updateWeight);

   double performTrimmedKalmanUpdate(
         const Eigen::MatrixXd &innovation,
         const Eigen::MatrixXd &jacobian,
         const Eigen::MatrixXd &observationVariance,
         const bool isSharedUpdate,
         const bool updateWeight);

   int addGoalPostMeasurement(
         const VisionUpdateBundle &visionBundle,
         const RRCoord &observedPostCoords,
         const AbsCoord &postWorldCoords,
         const bool useDistance,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement);

   int addFieldFeatureMeasurement(
         const VisionUpdateBundle &visionBundle,
         const AbsCoord &fieldFeaturePosition,
         const RRCoord &observedCentreCircleCoords,
         const bool useDistance,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement);

   int addBallMeasurement(
         const VisionUpdateBundle &visionBundle,
         const RRCoord &observedBallCoords,
         const bool useDistance,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement);

   int addTeammateMeasurement(
         const VisionUpdateBundle &visionBundle,
         const RRCoord &observedRobotCoords,
         const AbsCoord &teammatePosition,
         const bool useDistance,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement);

   void addGenericMeasurement(
         const RRCoord &observedObjectCoords,
         const bool useDistance,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         const int currentMeasurement,
         const double dx, const double dy);

   int addTriangulatedMeasurement(
      const VisionUpdateBundle &visionBundle,
      const RRCoord &observedObjectCoords,
      const AbsCoord &targetPos,
      const double targetOrientation,
      Eigen::MatrixXd &innovationOut,
      Eigen::MatrixXd &jacobianOut,
      Eigen::MatrixXd &observationVarianceOut,
      const int currentMeasurement);

   int addTriangulatedMeasurement_NEW(
      const VisionUpdateBundle &visionBundle,
      double dx, double dy, double dh, double obj_dist,
      const int xErrorSoftThresh_,
      const double xVarianceLinearGrowth_,
      const int yErrorSoftThresh_,
      const double yVarianceLinearGrowth_,
      const double thetaErrorSoftThresh_,
      const double thetaVarianceLinearGrowth_,
      Eigen::MatrixXd &innovationOut,
      Eigen::MatrixXd &jacobianOut,
      Eigen::MatrixXd &observationVarianceOut,
      const int currentMeasurement);

   void addObservationVariance(
         const VisionUpdateBundle &visionBundle,
         VarianceProvider::ObservationType type,
         const RRCoord &observedObject,
         bool useDistance,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement);

   int performICPUpdate(
         const VisionUpdateBundle &visionBundle,
         Eigen::MatrixXd &innovationOut,
         Eigen::MatrixXd &jacobianOut,
         Eigen::MatrixXd &observationVarianceOut,
         const int currentMeasurement,
         const bool haveKeyICPFeatures);

   void updateMeanVectorWithOdometry(const Odometry &odometry, const double dTimeSeconds,
         OdometryUpdateResult &outOdometryUpdateResult);
   void updateCovarianceWithOdometry(const Odometry &odometry, const double dTimeSeconds,
         const bool canSeeBall, OdometryUpdateResult &outOdometryUpdateResult);

   void updateMeanVectorWithRemoteOdometry(const SharedLocalisationUpdateBundle &updateBundle, int teammateIndex);
   void updateCovarianceWithRemoteOdometry(const SharedLocalisationUpdateBundle &updateBundle, int teammateIndex);

   void processUpdateCovarianceMatrix(const Odometry &odometry, const double dTimeSeconds);
   void additiveProcessNoiseUpdateCovarianceMatrix(const Odometry &odometry, const double dTimeSeconds,
         const bool canSeeBall, OdometryUpdateResult &outOdometryUpdateResult);

   void clipToField(Eigen::MatrixXd &pose);
   void clipBallOutOfRobot(Eigen::MatrixXd &pose);

   bool isStateValid(void) const;

   bool goalieUpdateDisagrees(const SharedLocalisationUpdateBundle &updateBundle) const;
   bool isBallTooCloseForRemoteUpdate(void) const;

   AbsCoord getPenaltySpotPosition(void);
   AbsCoord getOuterCornerPosition(void);
   AbsCoord getCentreTJunctionPosition(void);
   AbsCoord getGoalTJunctionPosition(void);
   AbsCoord getGoalCornerPosition(const RRCoord& object_rr);
   double getOuterCornerTargetOrientation(void);
   double getCentreTJunctionTargetOrientation(void);
   double getGoalTJunctionTargetOrientation(void);
   double getCentreCircleTargetOrientation(const RRCoord& object_rr);
   double getGoalCornerTargetOrientation(const RRCoord& object_rr);
   bool canObservePenaltySpot(const RRCoord& object_rr);
   bool canObserveGenericFieldFeature(const RRCoord& object_rr, SimpleGaussian::FieldFeatureType t);

   /**
    * Returns a value that is on the same side of 0 on the number line as the input value, and is
    * as close as possible to the value but is not within EPSILON of 0.
    */
   static double getNonZero(double val);

   static AbsCoord getGoalpostPosition(PostType type);
   bool isUnambiguousCentreCircleOrientation(const RRCoord& object_rr);

   bool isFacingAway(double orientation);

   // Returns the balls that we deem are on the field
   std::vector <RRCoord> getOnFieldBallRR(const std::vector<BallInfo> &visibleBalls);

   // Based on our current pose estimate, checks whether ball is on field
   bool isOnField(const RRCoord &rr);
};
