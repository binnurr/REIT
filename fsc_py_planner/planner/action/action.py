

class Action():
    
    #constructor
    def __init__(self, robot):
        self.robot = robot
    
    def init(self):
        return True
        
    def step(self):
        return True
    
    def finalize(self):
        return True
