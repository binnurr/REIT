
from planner.transition.transition import Transition
from std_msgs.msg import String

class ObjectExpDoneT(Transition):

    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.robot.rospy.Subscriber('is_on_table', String, self.pos_message_callback, queue_size=1)
        self.robot.rospy.Subscriber('user_speaking_stop', String, self.speech_message_callback, queue_size=1)
        self.is_on_table = False
        self.user_not_speaking = False

    # override only init method and condition method
    def init(self):
        self.is_on_table = False
        self.user_not_speaking = False

    def condition(self):
        if (self.is_on_table): ## TODO and self.user_not_speaking):
            return True
        else:
            return False

    def pos_message_callback(self, msg):
        if (msg.data == 'true'):
            self.is_on_table = True

    def speech_message_callback(self, msg):
        if (msg.data == 'true'):
            self.user_not_speaking = True







