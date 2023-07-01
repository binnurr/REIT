

from planner.action.action import Action
import time

class WaitAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
    
    def init(self):
        time.sleep(1)
        
    def step(self):
        pass
    
    def finalize(self):
        pass
