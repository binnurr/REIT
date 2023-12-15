

from planner.action.action import Action
import time


class RestartDriverAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
    
    def init(self):
        self.robot.init_web_driver()
        
    def step(self):
        pass

    def finalize(self):
        pass
    
