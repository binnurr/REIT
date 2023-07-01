

from planner.transition.transition import Transition

class SpeechRecognizedNaoqiT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
    
    def init(self):
        pass
        
    def condition(self):
        return self.robot.speech_recognition()
        
