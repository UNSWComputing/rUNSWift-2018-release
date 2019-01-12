import robot
import Sonar


class SonarChecker():
    def __init__(self):
        self.sonarTracker = []
        self.alertCounter = 0
        self.running = True

    def _tick(self):
        # If we've already alerted 4 times, stop running
        if self.running is False:
            return

        sonarValues = Sonar.blackboard.motion.sensors.sonar
        numVals = len(sonarValues)

        # Compare current sonar values with what we had last tick
        for i, sonarVal in enumerate([sonarValues[0],
                                      sonarValues[len(sonarValues) - 1]]):
            if len(self.sonarTracker) < numVals:
                temp = [sonarVal, 0]
                self.sonarTracker.append(temp)
                continue

            # If one of the current sonar values hasn't changed, increment its
            # corresponding counter
            if self.sonarTracker[i][0] == sonarVal:
                self.sonarTracker[i][1] += 1

            # If the current sonar has changed from last tick,
            # reset its counter
            else:
                self.sonarTracker[i][0] = sonarVal
                self.sonarTracker[i][1] = 0
        if self._check_freeze() is True:
            robot.say("sonar frozen")
            self.alertCounter += 1
            self._reset_tracker()
            if self.alertCounter >= 4:
                self.running = False

    def _check_freeze(self):
        frozen = True
        all10 = True
        # If all non-10 values are not moving, it is frozen
        for sonarVal in self.sonarTracker:
            if sonarVal[0] != 10 and sonarVal[1] < 50:
                frozen = False
            if sonarVal[0] != 10:
                all10 = False

        # if they're all 10, they probably arent frozen
        if all10 is True:
            frozen = False

        return frozen

    def _reset_tracker(self):
        for sonarVal in self.sonarTracker:
            # TODO: This either threw a Python error or never did anything...
            self.sonarTracker[1] = 0
