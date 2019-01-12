import robot

from util.MathUtil import normalisedTheta
from util import Vector2D
from util import TeamStatus
import math
import Global
from Constants import (
    FIELD_LENGTH,
    FIELD_WIDTH,
    GOAL_POST_ABS_Y,
    GOAL_POST_DIAMETER,
    MARKER_CENTER_X,
)

blackboard = None


# Enemy goal vectors.
OFFSET_REDUCE_SHARPNESS = 150  # so angles aren't too sharp near goals
ENEMY_GOAL_CENTER = Vector2D.Vector2D(FIELD_LENGTH / 2.0, 0)
ENEMY_GOAL_BEHIND_CENTER = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0 + OFFSET_REDUCE_SHARPNESS, 0)
ENEMY_GOAL_INNER_LEFT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_INNER_RIGHT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, -GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_OUTER_LEFT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_OUTER_RIGHT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, -GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_CORNER_LEFT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, FIELD_WIDTH / 2)
ENEMY_GOAL_CORNER_RIGHT = Vector2D.Vector2D(
    FIELD_LENGTH / 2.0, -FIELD_WIDTH / 2)
ENEMY_PENALTY_CENTER = Vector2D.Vector2D(MARKER_CENTER_X, 0)

# Own goal vetors
OWN_GOAL_CENTER = Vector2D.Vector2D(-FIELD_LENGTH / 2.0, 0)


def update(newBlackboard):
    """
    Updates the FieldGeometry.py global variables, i.e. the blackboard.

    Callable via `FieldGeometry.update(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard


TURN_RATE = math.radians(60.0)  # radians/second
WALK_RATE = 300.0  # mm/second
CIRCLE_STRAFE_RATE = math.radians(40.0)  # radians/second


def calculateTimeToReachPose(myPos, myHeading, targetPos, targetHeading=None):
    toTarget = targetPos.minus(myPos)
    toTargetHeading = math.atan2(toTarget.y, toTarget.x)

    # How far we need to turn to point at the targetPos
    toTargetTurn = abs(normalisedTheta(toTargetHeading - myHeading))

    # The straightline distance to walk to the targetPos
    toTargetDistance = toTarget.length()

    # How far we need to turn once we get to the targetPos so
    # that we are facing the targetHeading
    if targetHeading is None:
        toTargetHeadingTurn = 0.0
    else:
        toTargetHeadingTurn = abs(
            normalisedTheta(toTargetHeading - targetHeading))

    # approximate time it takes to avoid robots on the way
    avoidTime = 0
    if toTargetDistance > 400:
        robots = Global.robotObstaclesList()
        for _robot in robots:
            robotPos = Vector2D.makeVector2DCopy(_robot.pos)
            toRobot = robotPos.minus(myPos)
            dist_squared = toRobot.length2()
            heading = abs(normalisedTheta(toRobot.heading() - toTargetHeading))
            # further away robots are less relevant
            distValue = min(1, dist_squared / toTargetDistance ** 2)
            # robots aren't in the way are less relevant
            headingValue = min(1, heading / (math.pi / 2))
            # heading is more relevant than distance,
            # has enough weighting to revert striker bonus time
            combinedValue = (1 - distValue ** 2) * (1 - headingValue ** 2) * 3
            if combinedValue > avoidTime:
                avoidTime = combinedValue

    return (toTargetTurn / TURN_RATE +
            toTargetDistance / WALK_RATE +
            toTargetHeadingTurn / CIRCLE_STRAFE_RATE +
            avoidTime)


def calculateTimeToReachBall():
    interceptPoint = getBallIntersectionWithRobot(maintainCanSeeBall=False)
    interceptToGoal = ENEMY_GOAL_CENTER.minus(interceptPoint)
    interceptToGoalHeading = normalisedTheta(math.atan2(interceptToGoal.y,
                                                        interceptToGoal.x))
    return calculateTimeToReachPose(Global.myPos(), Global.myHeading(),
                                    interceptPoint, interceptToGoalHeading)


def myTimeToReach(positioning):
    myPos = positioning.getInstantPosition(
        Global.ballWorldPos(),  # target position
    )
    heading = Global.myHeading()
    return calculateTimeToReachPose(Global.myPos(), heading, myPos)


def calculateTimeToReachPositioning(playerNumber, positioning):
    currentPos, currentHeading = TeamStatus.getTeammatePose(playerNumber)
    targetPos = positioning.getPosition()
    return calculateTimeToReachPose(currentPos, currentHeading, targetPos)


def angleToPoint(point, absCoord):
    phi = Vector2D.angleBetween(point, Vector2D.makeVector2DCopy(absCoord))
    return normalisedTheta(phi)


def angleToGoal(absCoord):
    phi = Vector2D.angleBetween(ENEMY_GOAL_BEHIND_CENTER,
                                Vector2D.makeVector2DCopy(absCoord))
    return normalisedTheta(phi)


def angleToBallToGoal(absCoord):
    ball = blackboard.localisation.ballPos
    ballRR = blackboard.localisation.ballPosRR
    goalDir = angleToGoal(ball)
    return normalisedTheta(goalDir - (absCoord.theta + ballRR.heading))


# sharing this for goalie and striker to know where threshold is
def isInGoalBox(absPos, buffx=0, buffy=0, isStriker=False):
    if isStriker:
        (buffx, buffy) = (300, 200)
    return (
        absPos.x < -(robot.FIELD_LENGTH / 2 - robot.GOAL_BOX_LENGTH) + buffx
        and abs(absPos.y) < (robot.GOAL_BOX_WIDTH / 2) + buffy)


def addRrToRobot(robotPos, rx, ry):
    theta = robotPos.theta
    x = robotPos.x + math.cos(theta) * rx - math.sin(theta) * ry
    y = robotPos.y + math.sin(theta) * rx + math.cos(theta) * ry
    return x, y


def getBallIntersectionWithRobot(maintainCanSeeBall=True):
    intervalInSeconds = 1
    numSecondsForward = 1.0  # Estimate the ball position up to 1 second away
    numIterations = int(round(numSecondsForward / intervalInSeconds))
    FRICTION = 0.9  # friction per second
    FRICTION_PER_ITERATION = FRICTION ** intervalInSeconds

    ballVel = Global.ballWorldVelHighConfidence()
    ballPos = Global.ballWorldPos()
    myHeading = Global.myHeading()

    # If he ball is moving slowly, just chase the ball directly
    if ballVel.isShorterThan(10.0):
        return ballPos

    # Dont bother chasing a moving ball if its quite close.
    if Global.ballDistance() < 600.0:
        return ballPos

    ballVel.scale(intervalInSeconds)

    robotPos = Global.myPos()

    interceptPoint = ballPos
    bestChasePoint = ballPos.clone()

    seconds = 0.0
    for i in xrange(0, numIterations): # noqa
        seconds += intervalInSeconds

        interceptPoint.add(ballVel)
        ballVel.scale(FRICTION_PER_ITERATION)

        toIntercept = interceptPoint.minus(robotPos)
        toInterceptHeading = math.atan2(toIntercept.y, toIntercept.x)

        # How far we need to turn to point at the interceptPoint
        toInterceptTurn = abs(normalisedTheta(toInterceptHeading - myHeading))

        timeToTurn = toInterceptTurn / TURN_RATE
        timeToWalk = toIntercept.length() / WALK_RATE

        canReach = (timeToTurn + timeToWalk) <= seconds

        # Calculate difference in heading to the current ball position and
        # the intersect position, to make sure we don't turn too far and
        # lose sight of the ball
        v1 = interceptPoint.minus(robotPos).normalised()
        v2 = ballPos.minus(robotPos).normalised()
        heading = v1.absThetaTo(v2)

        if maintainCanSeeBall and heading > math.radians(75):
            return bestChasePoint

        if canReach:
            return bestChasePoint
        else:
            bestChasePoint = Vector2D.makeVector2DCopy(interceptPoint)

    return bestChasePoint


def globalPoseToRobotRelativePose(globalVector, globalHeading):
    robotPos = Global.myPos()
    robotHeading = Global.myHeading()

    rrVector = globalVector.minus(robotPos).rotate(-robotHeading)
    rrHeading = globalHeading - robotHeading

    return rrVector, rrHeading


def globalPointToRobotRelativePoint(globalVector):
    robotPos = Global.myPos()
    robotHeading = Global.myHeading()

    return globalVector.minus(robotPos).rotate(-robotHeading)


def nearFieldBorder():
    pos = Global.myPos()
    if abs(pos.x) > FIELD_LENGTH / 2 - 300:
        return True
    if abs(pos.y) > FIELD_WIDTH / 2 - 300:
        return True
    return False


def isInsideOwnGoalBox(x, y):
    buff = 100
    if x < -3900 + buff:
        if abs(y) < 1100 + buff:
            return True
    return False


def boundOutsideGoalBox(x, y):
    buff = 100
    x = max(x, -3900 + buff)
    if y > 0:
        y = max(1100, y)
    else:
        y = min(-1100, y)
    return x, y
