

from planner.action.action import Action

class StaticMotionAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        
        self.name = ""
    
    def init(self):
        self.robot.run_static_motion(self.name)
        
    def step(self):
        pass
    
    def finalize(self):
        pass
    
    def set_motion_name(self, name):
        self.name = name
