import Constants

from util.Vector2D import Vector2D

ATT = 800
REP = 500
REP_DECAY = 1
EPSILON = 1


def getAttractiveField(targetVector):
    return Vector2D(ATT, 0).rotate(targetVector.heading())


# distThreshold - maximum distance to feel the field
def getRepulsiveField(obsVector, distThresh=800):
    Urep = Vector2D(0, 0)
    dist = obsVector.length()

    # the buffer from your self to the obstacle
    dist = max(EPSILON, dist - Constants.ROBOT_DIAM / 2)

    repulsion = REP - REP_DECAY * dist

    if repulsion < 0:
        return Urep

    # find new obsPos with modified dist
    Urep = obsVector.normalise().multiply(-repulsion)
    return Urep
