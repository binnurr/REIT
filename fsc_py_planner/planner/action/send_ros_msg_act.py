

from planner.action.action import Action
from std_msgs.msg import String
import time

class SendRosMsgAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        
        self.topic_name = ''
        self.msg = ''
        
        self.publisher = None
        self.wait_time = 0
        self.published = False
    
    def init(self):
        # to send once in a state publish message in init method
        self.published = False
        
    def step(self):
        if(not self.published):
            self.publisher.publish(String(self.msg))
            self.published = True
        else:
            pass
        
            
    def finalize(self):
        pass
        
    def set_topic_and_msg(self, topic, msg):
        self.topic_name = topic
        self.msg = msg
        
        # connect to the ros
        self.publisher = self.robot.rospy.Publisher(self.topic_name, String)

        
