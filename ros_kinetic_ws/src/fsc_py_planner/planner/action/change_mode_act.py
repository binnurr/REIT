from planner.action.action import Action


class ChangeModeAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        if self.robot.mode == 0:
            self.robot.mode = 1
            self.robot.learned_object_count[0] = 0
        elif self.robot.mode == 1:
            self.robot.mode = 0
            self.robot.tested_object_count[0] = 0

    def step(self):
        pass

    def finalize(self):
        pass