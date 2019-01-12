#ifndef ROBOT_OBSERVATION_HPP
#define ROBOT_OBSERVATION_HPP

#include "types/RobotInfo.hpp"
#include "types/Odometry.hpp"
#include "types/RRCoord.hpp"

class RobotObservation {
public:
    /**
     * Creates a robot observation from a visual robot. If it is static, it
     * will never go stale and therefore be removed.
     */
    RobotObservation(const RobotInfo &visualRobot);

    /**
     * Apply odometry changes to the robot and update the staleness of the
     * observation.
     */
    void tick(Odometry odometryDiff, float headYaw, double distanceObservationToGroupPercentageOfMax);

    /**
     * Determines whether this observation should now be considered stale.
     */
    bool isStale() const;

    /**
     * Get the relative cartesian coordinates of the robot.
     */
    Point getCartesianCoordinates() const;

    /**
     * Get the relative vector to the robot.
     */
    RRCoord getRRCoordinates() const;

    /**
     * Get the weight of the robot, e.g. how important we should consider this
     * observation.
     */
    int getWeight() const;

    /**
     * Given a head yaw it determines whether this observation would be in the
     * field of view.
     */
    bool inView(float headYaw) const;

    /**
     * Get the minimum weight a robot can have.
     */
    static int getMinWeight();

    /**
     * Get the maximum weight a robot can have.
     */
    static int getMaxWeight();

    /**
     * Returns the number of frames that an observation would be alive if
     * it was onscreen the whole time.
     */
    static unsigned int numberOnScreenFramesToBeStale();

    /**
     * Returns the number of frames that an observation would be alive if
     * it was offscreen the whole time.
     */
    static unsigned int numberOffScreenFramesToBeStale();

    static const int MIN_WEIGHT;
    static const int WEIGHT_RANGE;

    static const int ESTIMATED_FRAMES_PER_SECOND;
    static const int MAX_LIFE_SCORE;

    static const double SECONDS_ALIVE_ON_FOV;
    static const double SECONDS_ALIVE_OFF_FOV;

    static const int OFF_SCREEN_SCORE_REDUCER;
    static const int ON_SCREEN_SCORE_REDUCER;
    static const int DISTANCE_SCORE_REDUCER;

    // Used to be private, used by GroupedRobots.cpp
    RRCoord rr;
    RobotInfo::Type type;

private:

    int lifeScore;
};

#endif // ROBOT_OBSERVATION_HPP
