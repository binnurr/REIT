

from planner.transition.transition import Transition
from std_msgs.msg import String
import json


class FeedbackEvaluatedT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.robot.rospy.Subscriber('correct_choices', String, self.message_callback, queue_size=10)

    def init(self):
        pass
        
    def condition(self):
        if len(self.robot.correct_questions) == self.robot.ques_count:
            return True
        else:
            return False
        
    def message_callback(self, msg):
        self.robot.correct_questions = json.loads(str(msg.data))
