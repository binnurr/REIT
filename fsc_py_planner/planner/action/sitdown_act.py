

from planner.action.action import Action
import time


class SitDownAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        sit_angles = [0.0, 0.0,  # head
                      1.545, 0.33, -1.57, -0.486,  # left arm
                      -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,  # left leg
                      -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,  # right leg
                      1.545, -0.33, 1.57, 0.486]  # right arm
        nao_names = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                     "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch",
                     "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch",
                     "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
        # check if AL motion manager or custom motion manager
        self.robot.motion_manager_proxy.angleInterpolation(nao_names, sit_angles, 3.5, True)
        time.sleep(3.5)
        self.robot.release_stiffness()

    def step(self):
        pass

    def finalize(self):
        pass
