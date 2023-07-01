# -*- coding: utf-8 -*-


from planner.action.action import Action
from std_msgs.msg import String
import time

class SayRosMsgWithTextAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        
        self.topic = ''
        
        self.message_arrived = False
        self.text = ''
        self.stepExecuted = False
    
    def init(self):
        self.message_arrived = False
        #time.sleep(3)  #TODO: why there is a sleep in here?
        
    #def step(self):
        #if (self.message_arrived):
            #speech = self.text+' '+self.msg
            #self.robot.say_text(speech)
            #self.message_arrived = False
            
    def step(self):
        if (not self.stepExecuted):
            #speech = self.text+' '+self.msg
            speech = self.text % (self.msg)
            self.robot.say_text(speech)
            self.message_arrived = False
            self.stepExecuted = True
    
    def finalize(self):
        self.message_arrived = False
        self.stepExecuted = False
        
    def set_topic(self, topic):
        self.topic_name = topic
        
        # connect to ROS
        self.robot.rospy.Subscriber(self.topic_name, String, self.message_callback)
        
    def set_text(self, text):
        self.text = text

        
    def message_callback(self, msg):
        print ('planner: message: ' + str(msg.data))
        self.msg = str(msg.data)
        if self.topic_name == 'userName':
            self.robot.userName = str(msg.data)
        self.message_arrived = True
        
        
