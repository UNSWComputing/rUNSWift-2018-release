"""
This is the main entry point to our Python application from PythonSkill.cpp,
where 'robot' is defined so 'import robot' does not fail with ImportError.

The `import robot` bridge from C++ to Python is defined by various Python
wrappers, run once at import time. Please see the `tick` function below for a
bridge from C++ to Python and back to C++ which is run many times at run time.

For example, for the implementation of `robot.TOP_IMAGE_ROWS`,
see the file `VisionDefinitions_wrap.cpp` in the following directory:
@see
https://github.com/UNSWComputing/rUNSWift/tree/master/robot/perception/behaviour/python/wrappers

For info, the following contains the C++ definition of `TOP_IMAGE_ROWS`:
@see
https://github.com/UNSWComputing/rUNSWift/blob/master/robot/perception/vision/VisionDefinitions.hpp
"""
import Simulation  # noqa
import robot

import os
import pkgutil
import traceback

import Global
import Log
import world

from Constants import LEDColour
from Task import BehaviourTask
from skills import HeadSkill
from util import (
    FieldGeometry,
    LedOverride,
    Sonar,
    TeamStatus,
    Timer,
)

skill_instance = None


class DummySkill(object):
    def __init__(self, blackboard):
        self.dummy = None
        # TODO(Ritwik): Move back to top when it's converted.


def catch_all(tick):
    def catcher(blackboard):
        try:
            return tick(blackboard)
        except:
            Log.error("Behaviour exception:", exc_info=True)
            raise

    return catcher


def skill_instance_factory(blackboard):
    skill = blackboard.behaviour.skill
    # Load the module and the class we're going to use.
    found_skill = False
    SkillClass = None
    behaviour_root = '/home/nao/data/behaviours'  # Nao + Windows = wat?
    behaviour_packages = ['roles', 'skills', 'test']
    for package in behaviour_packages:
        look_in = os.path.join(behaviour_root, package)
        seen_modules = [name for _, name, _ in pkgutil.iter_modules([look_in])]
        if skill not in seen_modules:
            Log.info('%s was not in %s, skipping import attempt.',
                     skill, package)
            continue
        skill_path = '%s.%s' % (package, skill)
        try:
            skill_module = __import__(skill_path, fromlist=[skill])
            # Access the class so we can do some reflection.
            SkillClass = getattr(skill_module, skill)
            found_skill = True
            Log.info("Successfully imported %s from %s", skill, skill_path)
            break
        except ImportError as e:
            Log.error("%s %s", package, e)
            Log.error(traceback.format_exc())

    if not found_skill:
        raise ImportError('Skill: %s not found in behaviour packages %s' %
                          skill, behaviour_packages)

    if issubclass(SkillClass, BehaviourTask):
        new_world = world.World(blackboard)  # It's a whole new world.
        _skill_instance = SkillClass(new_world)
    else:
        parentSkill = DummySkill(blackboard)
        _skill_instance = SkillClass(blackboard, parentSkill)
    return _skill_instance


@catch_all
def tick(blackboard):
    """
    This is the main entry point from C++ into our Python behaviours and back.

    More specifically it is the bridge from which C++ calls Python inside
    the runswift executable, and receives a BehaviourRequest back.

    Currently called in `robot/perception/behaviour/python/PythonSkill.cpp`,
    the `PythonSkill::execute()` C++ function, and explicitly the line
    `behaviour_tick(bb)`.

    :param blackboard: The runswift Blackboard, a bunch of things
        stored in global memory.
    :return: A `robot.BehaviourRequest()` instance, defined in C++ inside
        `robot/types/BehaviourRequest.hpp`.
    """
    # Update all blackboard dependent helper modules.
    Global.update(blackboard)
    TeamStatus.update(blackboard)
    FieldGeometry.update(blackboard)
    Timer.update(blackboard)
    LedOverride.reset()
    Sonar.update(blackboard)

    # Set the HeadSkill
    HeadSkill.singleton.resetRequestState()

    global skill_instance
    if not skill_instance:
        skill_instance = skill_instance_factory(blackboard)

    if isinstance(skill_instance, BehaviourTask):
        # On every tick of the perception thread, we update the blackboard,
        # tick the skill, and then return the resulting behaviour request.
        skill_instance.world.update(blackboard)
        skill_instance.world.b_request = robot.BehaviourRequest()
        skill_instance.world.b_request.actions = robot.All()
        skill_instance.tick()
        request = skill_instance.world.b_request
    else:
        # Backwards compat for old style skills if called directly via -s.
        request = skill_instance.tick(blackboard)

    headRequest = HeadSkill.singleton.tick(blackboard)
    request.actions.head = headRequest.actions.head

    # LED colouring.
    if len(blackboard.vision.uncertain_balls) > 0:
        request.actions.leds.rightEye = LEDColour.blue
    elif len(blackboard.vision.balls) > 0:
        request.actions.leds.rightEye = LEDColour.red
    else:
        request.actions.leds.rightEye = LEDColour.off

    if Global.amILost():
        request.actions.leds.leftEye = LEDColour.off
    else:
        request.actions.leds.leftEye = LEDColour.cyan

    return request
