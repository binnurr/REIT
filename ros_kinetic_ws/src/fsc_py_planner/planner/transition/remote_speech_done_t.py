

from planner.transition.transition import Transition
import time
from std_msgs.msg import String

class SpeechDoneT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.robot.rospy.Subscriber("remote_speech_finished", String, self.remote_speech_finished_cb)
        self.speech_done_pub = self.robot.rospy.Publisher('speech_done', String, queue_size=10)
    
    # override only init method and condition method
    def init(self):
        self.speech_finished = False
        
    def condition(self):
        if self.speech_finished:
            speech_done_msg = String()
            speech_done_msg.data = "speech_done"
            self.speech_done_pub.publish(speech_done_msg)
            return True
        else:
            return False
        
    def remote_speech_finished_cb(self, msg):
        if msg.data == 'finish':
            self.speech_finished = True
