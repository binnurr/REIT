

from planner.transition.transition import Transition 

class AlwaysTrueT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
    
    # override only init method and condition method
    def init(self):
        pass
        
    def condition(self):
        return True
        
