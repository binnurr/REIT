

from planner.action.action import Action
from std_msgs.msg import String
from random import uniform

class SayRosMsgAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        
        self.topic = ''
        
        self.message_arrived = False
        
    
    def init(self):
        self.message_arrived = False
        
    def step(self):
        if (self.message_arrived):
            if(self.robot.userName and uniform(0,1) < 0.5):
                if (not self.msg in self.robot.sound_file_ids.keys()):
                    speech = self.msg+' '+self.robot.userName
                else:
                    speech = str(self.robot.sound_file_texts[self.msg])+' '+self.robot.userName
                self.robot.say_text(speech)
                self.message_arrived = False
            else:
                self.robot.say_text(self.msg)
                self.message_arrived = False
    
    def finalize(self):
        self.message_arrived = False
    
    def set_topic(self, topic):
        self.topic_name = topic
        
        # connect to ROS
        self.robot.rospy.Subscriber(self.topic_name, String, self.message_callback)
        
    def message_callback(self, msg):
        print ('planner: message: ' + str(msg.data))
        self.msg = str(msg.data)
        self.message_arrived = True
        
        
