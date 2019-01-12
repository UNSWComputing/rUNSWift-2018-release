from Task import BehaviourTask
import actioncommand


class StandStraight(BehaviourTask):
    def init(self):
        pass

    def transition(self):
        pass

    def _tick(self):
        self.world.b_request.actions.body = actioncommand.standStraight()
