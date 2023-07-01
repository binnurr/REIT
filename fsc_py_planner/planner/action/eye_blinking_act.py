

from planner.action.action import Action
import time

class EyeBlinkingAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.count = 0
    
    def init(self):
        self.count = 0
        
    def step(self):
        if(self.count == 0):
            self.robot.leds_on()
        elif(self.count == self.robot.blinking_count):
            self.robot.leds_off()
        elif(self.count == self.robot.blinking_count+2):
            self.count = -1
        self.count = self.count +1
    def finalize(self):
        pass
    
