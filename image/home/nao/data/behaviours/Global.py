import robot

import math

from Constants import GameState, BALL_KICKOFF_STARTED_DISTANCE
from util.Vector2D import Vector2D
from util import TeamStatus

# Object caches.
_robotObstacles = None
_ballWorldPos = None
_myPose = None
_ballLostCount = None
_ballSeenCount = None

closest = False
blackboard = None


def update(newBlackboard):
    """
    Updates the Global.py global variables, such as the `_robotObstacles`.

    Callable via `Global.update(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard

    global _robotObstacles
    _robotObstacles = None

    global _ballWorldPos
    _ballWorldPos = blackboard.localisation.ballPos

    global _myPose
    _myPose = blackboard.localisation.robotPos

    global _ballLostCount
    _ballLostCount = blackboard.localisation.ballLostCount

    global _ballSeenCount
    _ballSeenCount = blackboard.localisation.ballSeenCount

    # global _feet
    # _feet = blackboard.vision.feet;


def crazyBallRunning():
    return len(blackboard.vision.uncertain_balls) > 0


def searchingAfterGetup():
    return blackboard.localisation.getupLost is True


# Vector2D world coordinates of the ball
# TODO: consider using a "teamball" type thing here if our confidence
# is very low.
def ballWorldPos():
    return Vector2D(_ballWorldPos.x, _ballWorldPos.y)


# Vector2D robot relative coordinates of the ball.
def ballRelPos():
    ballPosRRC = blackboard.localisation.ballPosRRC
    return Vector2D(ballPosRRC.x, ballPosRRC.y)


# Vector2D world velocity of the ball, in mm/s
def ballWorldVel():
    ballVel = blackboard.localisation.ballVel
    return Vector2D(ballVel.x, ballVel.y)


def ballWorldVelHighConfidence(minConfidence=0.15):
    """
    Lower values for the minConfidence means closer to the mean estimated
    velocity, but also results in higher uncertainty that the velocity is
    truly that fast.
    """
    velocity = ballWorldVel()
    speed = velocity.length()

    uncertainty = blackboard.localisation.ballVelEigenvalue * minConfidence
    if uncertainty > speed:
        return Vector2D(0.0, 0.0)

    velocity.normalise()
    speed = max(speed - uncertainty, 0.0)
    velocity.scale(speed)

    return velocity


# Vector2D robot relative velocity of the ball, in mm/s
def ballRelVel():
    ballVelRRC = blackboard.localisation.ballVelRRC
    return Vector2D(ballVelRRC.x, ballVelRRC.y)


# Float. Returns the Euclidian distance to the ball.
def ballDistance():
    return blackboard.localisation.ballPosRR.distance


def ballHeading():
    return blackboard.localisation.ballPosRR.heading


def myPose():
    return _myPose


# Vector2D robot world coordinates
def myPos():
    return Vector2D(_myPose.x, _myPose.y)


# Has the ball been kicked off from the centre circle
def ballKickedOff():
    return _ballWorldPos.x ** 2 + _ballWorldPos.y ** 2 > BALL_KICKOFF_STARTED_DISTANCE ** 2  # noqa


# Float of the robot world relative heading, in radians.
def myHeading():
    return _myPose.theta


# Boolean of whether the robot can currently see the ball.
def canSeeBall(frames=1):
    return _ballSeenCount >= frames


def amILost():
    robotPosUncertainty = blackboard.localisation.robotPosUncertainty
    robotHeadingUncertainty = blackboard.localisation.robotHeadingUncertainty

    if robotPosUncertainty > 1.2 * 10 ** 6:
        return True
    if robotHeadingUncertainty > 1.3:
        return True
    return False


def amILocalised():
    return (not amILost() and len(blackboard.localisation.allrobotPos) == 1) # noqa


# Whether robot is starting to get uncertain of its position
def robotPosGettingUncertain():
    # NOTE: this value was obtained from testing, and is used to determine
    # if we want to perform a quicklocalise headskill
    return blackboard.localisation.robotPosUncertainty > 0.5 * 10 ** 6


# Whether robot is starting to get uncertain of its heading
def robotHeadingGettingUncertain():
    # NOTE: this value was obtained from testing, and is used to determine
    # if we want to perform a quicklocalise headskill
    return blackboard.localisation.robotHeadingUncertainty > 0.7


def ballPosUncertainty():
    return math.sqrt(blackboard.localisation.ballPosUncertainty)


def isBallUncertain():
    return blackboard.localisation.ballPosUncertainty > (1500.0 * 1500.0)


def isBallLost():
    # TODO: these numbers are not necessarily appropriate
    # This means we don't just need to lose certainty with
    #  localisation on the ball,
    #  but also let some ticks pass before we consider it lost
    #  this number is hand tuned and has no intrinsic value
    return isBallUncertain() and _ballLostCount > 300


def amILiningUpKick():
    return blackboard.behaviour.behaviourSharedData.doingBallLineUp


def offNetwork():
    return False


# Robot Obstacles.
def robotObstaclesList():
    # Convert blackboard array to an easier to use list
    global _robotObstacles
    if _robotObstacles is not None:
        return _robotObstacles
    _robotObstacles = []
    for i in range(len(blackboard.localisation.robotObstacles)):
        _robotObstacles.append(blackboard.localisation.robotObstacles[i])
    return _robotObstacles


def ballLostFrames():
    return _ballLostCount


def EmptyBehaviourRequest():
    b = robot.BehaviourRequest()
    b.actions = robot.All()
    return b


def currentVisionTime():
    return blackboard.vision.timestamp


def iAmPenalised():
    return blackboard.gameController.our_team.players[
             TeamStatus.myPlayerNumber() - 1].penalty != GameState.PENALTY_NONE


def iAmPenalisedOrInitial():
    return blackboard.gameController.data.state == GameState.INITIAL or \
                                                                 iAmPenalised()


def usingGameControllerSkill():
    return blackboard.behaviour.skill == "GameController"


def getClosest():
    global closest
    return closest


def setClosest(close):
    global closest
    closest = close
