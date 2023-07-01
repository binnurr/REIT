

from planner.action.action import Action

class SayTextAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.text = ''
        self.pass_initial_time = False
    
    def init(self):
        if (self.pass_initial_time):
            self.pass_initial_time = False
        else:
            self.robot.say_text(self.text)
        
    def step(self):
        pass
    
    def finalize(self):
        pass
        
    def set_text(self, text):
        self.text = text

    def pass_initial_say(self):
        self.pass_initial_time = True
