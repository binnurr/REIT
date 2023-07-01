from planner.action.action import Action


class DecideEmotionAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        print("Nao will decide emotion")
        self.robot.decide_emotion()
        pass

    def step(self):
        pass

    def finalize(self):
        pass