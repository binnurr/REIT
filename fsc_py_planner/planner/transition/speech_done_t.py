

from planner.transition.transition import Transition
import time
from std_msgs.msg import String

class SpeechDoneT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.wait_sec = 0
        self.speech_done_pub = self.robot.rospy.Publisher('speech_done', String)
    
    # override only init method and condition method
    def init(self):
        pass
        
    def condition(self):
        if self.robot.is_say_finished() and self.fsc.get_elapsed_time() > self.wait_sec:
            speech_done_msg = String()
            speech_done_msg.data = "speech_done"
            self.speech_done_pub.publish(speech_done_msg)
            return True
        else:
            return False

    # after robot speech is done, if you want to allow some seconds for the user, use this function
    def wait_for_sec(self, wait_sec):
        self.wait_sec = wait_sec
