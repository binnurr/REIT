

from planner.transition.transition import Transition
from std_msgs.msg import String

class BinaryT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.transition1 = None
        self.transition2 = None

    # override only init method and condition method
    def init(self):
        self.transition1.init()
        self.transition2.init()
        
    def condition(self):
        if self.transition1.condition() and self.transition2.condition():
            return True
        else:
            return False


    def set_transitions(self, transition1, transition2):
        self.transition1 = transition1
        self.transition2 = transition2

        
