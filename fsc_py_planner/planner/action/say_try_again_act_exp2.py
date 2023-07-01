from planner.action.action import Action
import time
import random


class SayTryAgainAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        possible_texts = ['Would you like to try again? Please select one and say loudly!',
                          'Would you like to try again?',
                          'Please try again',
                          'Please select one and say loudly',
                          'Can you try again?',
                          "Let's try again"]
        self.robot.say_text(random.choice(possible_texts))

    def step(self):
        pass

    def finalize(self):
        pass

    def set_text(self, text):
        self.text = text

    def pass_initial_say(self):
        self.pass_initial_time = True
