from planner.action.action import Action


class ExpressEmotionAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.pre_emotion = False

    def init(self):
        if self.pre_emotion:
            print("Nao will show pre-emotion with value of : " + str(self.robot.hope_level))
        else:
            print("Nao will show post-emotion with value of : " + str(self.robot.joy_level))
        self.robot.express_emotion(self.pre_emotion)
        pass

    def step(self):
        pass

    def finalize(self):
        pass

    def set_pre_emotion(self, pre):
        if (pre):
            self.pre_emotion = True
