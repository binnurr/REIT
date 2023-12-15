

from planner.transition.transition import Transition 

class TimeElapsedT(Transition):
    
    # override only init method and condition method
    def init(self):
        return True
        
    def condition(self):
        if (self.fsc.get_elapsed_time() > self.elapsed_time):
            return True
        else:
            return False
        
    # declare its own parameters here
    # set elapsed time in seconds
    def set_elapsed_time(self, elapsed_time):
        self.elapsed_time = elapsed_time
        
