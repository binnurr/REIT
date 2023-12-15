

from planner.transition.transition import Transition
from std_msgs.msg import String


class RosMsgT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.any_value = False
        self.topic_name = None
        self.expected_value_list = None
        self.message_ok = False
        self.is_inc_attribute = False
        self.is_set_attribute = False
        self.obj = None
        self.field = None
    
    # override only init method and condition method
    def init(self):
        self.message_ok = False
        
    def condition(self):
        if self.message_ok:
            return True
        else:
            return False
        
    def set_topic_and_value(self, topic, expected_value_list):
        self.topic_name = topic
        self.expected_value_list = expected_value_list
        self.robot.rospy.Subscriber(self.topic_name, String, self.message_callback, queue_size=1)
        print("CALLED")
        
    def set_any_value(self, enabled):
        self.any_value = enabled
        
    def message_callback(self, msg):
        print ('ros_msg_t::message_callback msg: ' + str(msg.data))
        if self.any_value:
            self.message_ok = True
        else:
            if msg.data.lower() in self.expected_value_list:
                self.message_ok = True
            else:
                self.message_ok = False

        if self.message_ok:
            if self.is_set_attribute:
                setattr(self.obj, self.field, msg.data)
            if self.is_inc_attribute:
                crrnt_val = getattr(self.obj, self.field)
                setattr(self.obj, self.field, crrnt_val+1)

    def inc_attribute(self, obj, field):
        self.is_inc_attribute = True
        self.obj = obj
        self.field = field

    def set_attribute(self, obj, field):
        self.is_set_attribute = True
        self.obj = obj
        self.field = field


        
