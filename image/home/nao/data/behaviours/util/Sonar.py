from util.Hysteresis import Hysteresis

WINDOW_LENGTH = 30
NEARBY_HIT_DECAY_FRAMES = 10
MIN_FRAMES_TO_SEE_SONAR_OBS = 3
blackboard = None
nearbySonarHysteresis = [
    Hysteresis(-MIN_FRAMES_TO_SEE_SONAR_OBS, NEARBY_HIT_DECAY_FRAMES),
    Hysteresis(-MIN_FRAMES_TO_SEE_SONAR_OBS, NEARBY_HIT_DECAY_FRAMES)]


def update(newBlackboard):
    """
    Updates the Sonar.py global variables, such as the blackboard.

    Callable via `Sonar.update(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    updateNearbySonarValues()


def hasNearbySonarObject(i):
    return nearbySonarHysteresis[i].value > 0


def updateNearbySonarValues():
    sonarValues = blackboard.motion.sensors.sonar
    for i, sonarVal in enumerate([sonarValues[0], sonarValues[10]]):
        # print("SonarVal "+str(i)+": " + str(sonarVal))
        if sonarVal < 0.4:
            if nearbySonarHysteresis[i].value == 0:
                nearbySonarHysteresis[i].resetMax()
            else:
                nearbySonarHysteresis[i].up()
        else:
            nearbySonarHysteresis[i].down()


def getRawSonarArray():
    sonar_list = []
    for i in range(len(blackboard.motion.sensors.sonar)):
        sonar_list.append(blackboard.motion.sensors.sonar[i])
    return sonar_list
    # return robot.floatArray_frompointer(blackboard.motion.sensors.sonar)
