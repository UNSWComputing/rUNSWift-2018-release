#ifndef BALL_ROBOT_HPP
#define BALL_ROBOT_HPP

#include "types/BallInfo.hpp"
#include "types/RobotInfo.hpp"
#include "types/BBox.hpp"
#include "types/Point.hpp"

#define ARBITRARY_BALL_SIZE 60

RobotInfo ballToRobot(BallInfo& ball)
{
    RobotInfo newRobot(ball.rr, RobotInfo::rUnknown,
        BBox(Point(ball.imageCoords.x()-ball.radius,
        ball.imageCoords.y()-ball.radius),
        Point(ball.imageCoords.x()+ball.radius,
                                ball.imageCoords.y()+ball.radius)), RobotInfo::BOT_CAMERA);
    return(newRobot);
}

BallInfo robotObstacleToBall(RobotObstacle& robot, CameraToRR& camConverter)
{
    BallInfo newBall(robot.rr, ARBITRARY_BALL_SIZE,
        camConverter.convertToImageXY(robot.rr.toCartesian()),
        camConverter.pose.robotRelativeToNeckCoord(robot.rr,
                                                          ARBITRARY_BALL_SIZE));
    newBall.topCamera = false;
    return(newBall);
}

#endif // BALL_ROBOT_HPP
