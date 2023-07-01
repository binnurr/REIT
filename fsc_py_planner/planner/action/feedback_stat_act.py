from planner.action.action import Action
import time
from fsc_py_planner.msg import GUItext
from std_msgs.msg import String


class FeedbackStatAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
        self.publisher = self.robot.rospy.Publisher('feedback_stat', String, queue_size=1)

    def init(self):
        len_ques = len(self.robot.feedback_dict)
        incorrect_ques_count = 0
        for i in range(len_ques):
            if self.robot.feedback_dict[i] != "":
                incorrect_ques_count += 1
        text = "Total number of responses: " + str(len_ques) + " \n"
        text += "Incorrectly chosen responses: " + str(incorrect_ques_count) + " \n"
        text += "After receiving feedback, you correctly chose " + str(incorrect_ques_count-self.robot.second_failure_count) + " responses \n"
        text += "Your erroneous choices per mistake category are like below: \n"
        for mistake, count in self.robot.given_feedback_cache_dict.iteritems():
            text += mistake + " : " + str(count) + " \n"
        self.publisher.publish(String(text))

    def step(self):
        pass

    def finalize(self):
        pass
