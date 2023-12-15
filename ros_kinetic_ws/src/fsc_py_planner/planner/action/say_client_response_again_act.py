from planner.action.action import Action
import time


class SayClientResponseAgainAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
        self.turn_count = 0

    def init(self):
        self.robot.say_text("sure. " + self.robot.robot_text)

    def step(self):
        pass

    def finalize(self):
        pass

    def set_text(self, text):
        self.text = text

    def pass_initial_say(self):
        self.pass_initial_time = True
