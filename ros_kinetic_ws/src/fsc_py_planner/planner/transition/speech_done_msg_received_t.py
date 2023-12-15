

from planner.transition.transition import Transition
from std_msgs.msg import String

class SpeechDoneMsgReceivedT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.topic = ''
        self.message_arrived = False
        self.is_waiting_saying = True
    # override only init method and condition method
    def init(self):
        self.is_waiting_saying = True
        
    def condition(self):
        if (self.robot.is_say_finished() and self.is_waiting_saying):
            return False
        else:
            self.is_waiting_saying = False
            
        return (self.robot.is_say_finished() & self.message_arrived and not self.is_waiting_saying)
    
    def set_topic(self,topic):
        self.topic_name = topic
        self.robot.rospy.Subscriber(self.topic_name, String, self.message_callback)
        
    def message_callback(self, msg):
        self.message_arrived = True

        
        
    
        
