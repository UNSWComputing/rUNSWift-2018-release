#ifndef ROBOT_FILTER_UPDATE_HPP
#define ROBOT_FILTER_UPDATE_HPP

struct RobotFilterUpdate {
    std::vector<RobotInfo> visualRobots;
    AbsCoord robotPos;
    float headYaw;
    Odometry odometryDiff;
    bool isIncapacitated;
};

#endif // ROBOT_FILTER_UPDATE_HPP
