

from planner.transition.transition import Transition
import time
from std_msgs.msg import String

class SpeechDoneT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.robot.rospy.Subscriber("remote_speech_finished", String, self.remote_speech_finished_cb)
    
    # override only init method and condition method
    def init(self):
        self.speech_finished = False
        
    def condition(self):
        if self.speech_finished:
            return True
        else:
            return False
        
    def remote_speech_finished_cb(self, msg):
        if msg.data == 'finish':
            self.speech_finished = True
