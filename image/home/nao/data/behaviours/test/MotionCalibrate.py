import actioncommand

from skills import HeadSkill
from Task import BehaviourTask


class MotionCalibrate(BehaviourTask):
    def init(self):
        pass

    def tick(self):
        self.world.b_request.actions.body = actioncommand.motionCalibrate()
        HeadSkill.singleton.requestState(HeadSkill.HEAD_SKILL_LOOK_CENTER)
