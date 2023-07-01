from planner.action.action import Action
import time

from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support.ui import WebDriverWait


class SayClientResponseAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
        self.turn_count = 0

    def init(self):
        self.robot.selected_ques_dict[self.turn_count] = self.robot.selected_ques_ind
        self.robot.robot_text = self.robot.scenario.traverse_once(int(self.robot.selected_ques_ind)-1)
        self.robot.robot_text = self.robot.robot_text.encode("utf-8")
        print(self.robot.robot_text)
        self.robot.say_text(self.robot.robot_text)
        self.turn_count += 1

    def step(self):
        pass

    def finalize(self):
        pass

    def set_text(self, text):
        self.text = text

    def pass_initial_say(self):
        self.pass_initial_time = True
