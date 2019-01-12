import robot

import math


blankRGB = robot.rgb()


def head(yaw=0, pitch=0, isYawRelative=False, yawSpeed=1.0, pitchSpeed=1.0):
    return robot.HeadCommand(yaw, pitch, isYawRelative, yawSpeed, pitchSpeed)


def stand(power=0.1):
    return walk(0, 0, 0, power, bend=0)


def crouch(power=0.4):
    return walk(0, 0, 0, power, bend=1)


def jabKick(forward=1, left=0, turn=0, power=0.5, foot=robot.Foot.LEFT):
    # speed=0 and isFast=True signals jabKick to Walk2014
    return walk(forward, left, turn, power, 0, 0, foot, True)


# deprecated not supported
def walkKick(forward=1, left=0, turn=0, power=0.5, foot=robot.Foot.LEFT):
    # speed=1 and isFast=True signals walkKick to Walk2014
    return walk(forward, left, turn, power, 0, 1, foot, True)


def walk(forward=0, left=0, turn=0, power=0.0, bend=1, speed=1.0,
         foot=robot.Foot.LEFT, isFast=False, useShuffle=False, leftArmLimp=False, rightArmLimp=False): # noqa
    # Positional arguments only http://stackoverflow.com/a/35962682
    return robot.BodyCommand(
        robot.ActionType.WALK,  # actionType
        int(forward),           # forward
        int(left),              # left
        float(turn),            # turn
        float(power),           # power
        float(bend),            # bend
        # TODO: Should kick parameters be zero-ed in the walk?
        float(speed),           # speed
        float(speed),           # kickDirection :nao-wat:?
        foot,                   # foot
        bool(isFast),           # isFast
        False,                  # misalignedKick
        bool(useShuffle),       # shuffle
        bool(leftArmLimp),      # leftArmLimp
        bool(rightArmLimp)      # rightArmLimp
    )


def kick(power=1.0, kickDirection=0.0, foot=robot.Foot.LEFT, turn=0,
         misalign=False, turnThreshold=math.radians(20)):
    return robot.BodyCommand(
        robot.ActionType.KICK,  # actionType
        0,                      # forward
        0,                      # left
        float(turn),            # turn
        power,                  # power
        0.0,                    # bend
        turnThreshold,          # speed
        kickDirection,          # kickDirection
        foot,                   # foot
        False,                  # isFast
        misalign,               # misalignedKick
        False,                  # shuffle
        False,                  # leftArmLimp
        False                   # rightArmLimp
    )


def dribble(foot=robot.Foot.LEFT, turn=0, forward=150):
    return robot.BodyCommand(
        robot.ActionType.DRIBBLE,  # actionType
        forward,                   # forward
        0,                         # left
        float(turn),               # turn
        1,                         # power
        0.0,                       # bend
        math.radians(20.0),        # speed
        0.0,                       # kickDirection
        foot,                      # foot
        False,                     # isFast
        False,                     # misalignedKick
        False,                     # shuffle
        False,                     # leftArmLimp
        False                      # rightArmLimp
    )


def turnDribble(foot=robot.Foot.LEFT, turn=0):
    return robot.BodyCommand(
        robot.ActionType.TURN_DRIBBLE,  # actionType
        0,                         # forward
        0,                         # left
        float(turn),               # turn
        1,                         # power
        0.0,                       # bend
        math.radians(20.0),        # speed
        0.0,                       # kickDirection
        foot,                      # foot
        False,                     # isFast
        False,                     # misalignedKick
        False,                     # shuffle
        False,                     # leftArmLimp
        False                      # rightArmLimp
    )


def motionCalibrate():
    return _type_only_body_command(robot.ActionType.MOTION_CALIBRATE)


def standStraight():
    return _type_only_body_command(robot.ActionType.STAND_STRAIGHT)


def goalieDiveRight():
    return _type_only_body_command(robot.ActionType.GOALIE_DIVE_RIGHT)


def goalieCentre():
    return _type_only_body_command(robot.ActionType.GOALIE_CENTRE)


def goalieUncentre():
    return _type_only_body_command(robot.ActionType.GOALIE_UNCENTRE)


def goalieDiveLeft():
    return _type_only_body_command(robot.ActionType.GOALIE_DIVE_LEFT)


def defenderCentre():
    return _type_only_body_command(robot.ActionType.DEFENDER_CENTRE)


def _type_only_body_command(action_type):
    return robot.BodyCommand(action_type, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             robot.Foot.LEFT, False, False, False, False, False) # noqa


def leds(leye=blankRGB, reye=blankRGB, chest=blankRGB,
         lfoot=blankRGB, rfoot=blankRGB):
    return robot.LEDCommand(leye, reye, chest, lfoot, rfoot)


def compose(head=head(), body=walk(), leds=leds()):
    return robot.All(head, body, leds, 0.0, robot.StiffenCommand.NONE)
