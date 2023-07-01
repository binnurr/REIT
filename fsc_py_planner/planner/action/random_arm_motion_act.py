
from planner.action.action import Action
import time


class RandomArmMotionAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        # self.robot.goto_posture('Crouch', 1.0)
        # time.sleep(1.0)
        pass

    def step(self):
        self.robot.random_arm_motion()

    def finalize(self):
        pass
