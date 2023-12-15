

class Transition():
    
    #constructor
    def __init__(self, fsc, robot):
        self.fsc = fsc
        self.robot = robot
        self.decided = False
        self.name = "transition"
        self.target_state = None
    
    def check(self):
        if (self.condition()):
            self.decided = True
            return True
        
        return False
    
    def run(self):
        if (self.decided):
            self.decided = False
            self.do_transition()
            return
        
        if (self.condition()):
            self.do_transition()
            
    def condition(self):
        return False
    
    def init(self):
        return True
        
        
    def do_transition(self):
        self.fsc.set_next_state(self.target_state)
        
    def set_target_state(self, target_state):
        self.target_state = target_state
            
    
