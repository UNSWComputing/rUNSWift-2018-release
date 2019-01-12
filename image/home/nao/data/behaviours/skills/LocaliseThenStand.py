# flake8: noqa
import robot
import actioncommand
import Global

from LostScanSkill import LostScanSkill

class LocaliseThenStand(object):
   def __init__(self, blackboard, parent):
      self.parent = parent
      self.lostScanSkill = LostScanSkill(blackboard)

   def tick(self, blackboard):
      if Global.amILost():
         return self.lostScanSkill.tick(blackboard)

      req = robot.BehaviourRequest()
      req.actions = robot.All()
      req.actions.leds = actioncommand.leds()
      req.actions.body = actioncommand.stand()
      return req

