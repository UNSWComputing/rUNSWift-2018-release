import robot

import Constants
from util.Vector2D import Vector2D

blackboard = None
fieldPlayerIndices = None


def update(newBlackboard):
    """
    Updates the TeamStatus.py global variables, such as the fieldPlayerIndices.

    Callable via `TeamStatus.update(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    global fieldPlayerIndices
    i = myPlayerIndex()
    if fieldPlayerIndices is None:
        fieldPlayerIndices = range(1, numTeammates())
        if i != 0:  # Skip Goalie
            fieldPlayerIndices.remove(i)


NUM_LOST_FRAMES_CAN_SEE_BALL = 80


def getBehaviourSharedData(index):
    return blackboard.receiver.data[index].behaviourSharedData


def getTimeToReachBall(index):
    return getBehaviourSharedData(index).timeToReachBall


def getTimeToReachUpfielder(index):
    return getBehaviourSharedData(index).timeToReachUpfielder


def getTimeToReachMidfielder(index):
    return getBehaviourSharedData(index).timeToReachMidfielder


def getTimeToReachDefender(index):
    return getBehaviourSharedData(index).timeToReachDefender


def myPlayerIndex():
    return blackboard.gameController.player_number - 1


def myPlayerNumber():
    return blackboard.gameController.player_number


def getTeammatePlayerNum(index):
    return index + 1


def getTeammatePose(index):
    pose = blackboard.receiver.data[index].robotPos
    return (Vector2D(pose.x, pose.y), pose.theta)


def getTeammatePos(index):
    pose = blackboard.receiver.data[index].robotPos
    return Vector2D(pose.x, pose.y)


def getTeammateRole(index):
    return getBehaviourSharedData(index).currentRole


def getTeammateKickoffSide(index):
    return getBehaviourSharedData(index).kickoffSide


def countNumTeammatesHaveRole(targetRole):
    count = 0
    for i in getFieldPlayerIndices():
        if isTeammateActive(i) and getTeammateRole(i) == targetRole:
            count += 1

    return count


def iAmStriker():
    i = myPlayerIndex()
    return (getTeammateRole(i) == Constants.ROLE_STRIKER and
            teammateLostBallCount(i) < NUM_LOST_FRAMES_CAN_SEE_BALL)


def seesBallStrikerCount():
    """
    :return: The number of teammates reporting themselves as a striker,
        who also reported they can see the ball.
    """
    count = 0
    for i in getFieldPlayerIndices():
        if (isTeammateActive(i) and
                getTeammateRole(i) == Constants.ROLE_STRIKER and
                teammateLostBallCount(i) < NUM_LOST_FRAMES_CAN_SEE_BALL):
            count += 1

    return count


def teammateLostBallCount(index):
    return blackboard.receiver.data[index].lostCount


def countActiveTeammates():
    """
    :return: The number of teammates, including the goalie, that are
        currently on the field, communicating on wireless, and not penalised.
    """
    count = 0
    for i in xrange(0, numTeammates()): # noqa
        if not blackboard.receiver.incapacitated[i]:
            count += 1

    return count


def isTeammateActive(index):
    if getTeammatePlayerNum(index) == myPlayerNumber():
        return True
    else:
        return not blackboard.receiver.incapacitated[index]


def numTeammates():
    return len(blackboard.receiver.data)


def getFieldPlayerIndices():
    # array with player numbers of all OTHER robots on team, excluding goalie
    return fieldPlayerIndices


def isGoalieAttacking():
    for i in xrange(0, numTeammates()): # noqa
        if isTeammateActive(i) and getBehaviourSharedData(i).goalieAttacking:
            return True
    else:
        return False


def isTeamMate(unknownRobot):
    return unknownRobot.type == robot.RobotInfoType.rRed


def ourKickOff():
    return teamNumber() == blackboard.gameController.data.kickOffTeam


def teamNumber():
    return blackboard.gameController.our_team.teamNumber


def lonelyPlayer():
    """Lonely Player.

    Return whether the current player is lonely,
    i.e, hasn't heard from his comrades in a while.
    """

    incapacitated = blackboard.receiver.incapacitated
    for i in xrange(robot.ROBOTS_PER_TEAM): # noqa
        if i == blackboard.gameController.player_number - 1:
            # Skip myself.
            continue
        if not incapacitated[i]:
            return False
    return True
