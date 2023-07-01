

from state import State

class FSC(State):
    
    # constructor
    def __init__(self, robot, name):
        
        State.__init__(self, name)
        
        self.robot = robot
        
        self.first_run = True
        
        self.initial_state = None
        self.active_state = None
        self.next_state = None
        self.prev_state = None
        
        self.transition_count = 0
        self.max_transition_count = 100
        
        self.states = []
        
        
    def run(self):
        
        # run the base classes run
        State.run_actions(self)
        
        if (self.first_run):
            self.active_state.on_enter()
            self.first_run = False
        
        if (self.next_state):
            self.active_state.on_exit()
            self.prev_state = self.active_state
            self.active_state = self.next_state
            
            #print("transition to " + self.active_state.name)
            self.robot.rospy.loginfo("transition to %s", self.active_state.name)
            
            self.active_state.on_enter()
            self.next_state = None
        
        self.active_state.run()
        
        if (self.active_state.is_transited):
            self.transition_count += 1
            if (self.transition_count < self.max_transition_count):
                self.run()
        else:
            self.transition_count = 0
            
        self.is_transited = State.run_transitions(self)
    
    
    def set_initial_state(self, initial_state):
        self.initial_state = initial_state
        self.active_state = self.initial_state
        
    def set_next_state(self, next_state):
        self.next_state = next_state
        
    def on_exit(self):
        State.on_exit(self)
        self.active_state.on_exit()
        
    def set_transition(self, transition, from_state, to_state):
        from_state.add_transition(transition)
        transition.set_target_state(to_state)
        
    def get_elapsed_time(self):
        if (self.active_state):
            return self.active_state.get_elapsed_time()
        return None
            
            
            
        
            
        
    
