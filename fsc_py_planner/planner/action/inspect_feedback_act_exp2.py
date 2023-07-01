from planner.action.action import Action
import time
from fsc_py_planner.msg import GUItext
from std_msgs.msg import String

from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support.ui import WebDriverWait


class InspectFeedbackAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
        self.publisher = self.robot.rospy.Publisher('promptQues_Feedback', GUItext, queue_size=1)
        self.publisher2 = self.robot.rospy.Publisher('selected_incorr_ques', String, queue_size=1)

    def init(self):
        selected_ques = None

        for i in range(self.robot.current_ques+1, self.robot.ques_count):
            selected_ques = int(self.robot.selected_ques_dict[i]) - 1
            if self.robot.feedback_dict[i] != ["No mistake"]:
                self.robot.current_ques = i
                break
        print("selected_ques", selected_ques)
        print("current turn", self.robot.current_ques)
        prev_answer = self.robot.scenario.\
            traverse(self.robot.scenario.traversed_edges[:self.robot.current_ques], True)
        options = self.robot.scenario.get_options()
        GUItext_msg = GUItext()
        GUItext_msg.prev_answer = prev_answer.encode("utf-8")
        GUItext_msg.ques1 = str(1) + ". " + options[0]
        GUItext_msg.ques2 = str(2) + ". " + options[1]
        GUItext_msg.ques3 = str(3) + ". " + options[2]
        self.publisher.publish(GUItext_msg)
        time.sleep(0.3)
        self.publisher2.publish(str(selected_ques+1))


    def step(self):
        pass

    def finalize(self):
        pass
