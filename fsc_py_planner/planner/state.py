
import time

class State():
    
    #constructor
    def __init__(self, state_name):
        self.name = state_name
        self.start_time = 0.0
        
        # create action and transition containers
        self.actions = []
        self.transitions = []
        
        self.is_transited = False
        
        
    def run(self):
        self.run_actions()
        self.is_transited = self.run_transitions()
        
    def on_enter(self):
        for action in self.actions:
            action.init()
        
        for transition in self.transitions:
            transition.init()
        
        self.start_time = time.time()
        
    
    def on_exit(self):
        for action in self.actions:
            action.finalize()
        
    def run_actions(self):
        for action in self.actions:
            action.step()
        
    def run_transitions(self):
        self.is_transited = False
        for transition in self.transitions:
            if (transition.check()):
                transition.run()
                self.is_transited = True
                break
        
        return self.is_transited  
        
    def add_action(self, action):
        self.actions.append(action)
        
    def add_transition(self, transition):
        self.transitions.append(transition)
    
    # returns elapsed time since on_enter in seconds
    def get_elapsed_time(self):
        return (time.time()-self.start_time)
        
        
