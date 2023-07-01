from planner.action.action import Action
from std_msgs.msg import String, Bool
from fsc_py_planner.msg import GUItext
import time
import json

from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support.ui import WebDriverWait


class PromptUserAct(Action):
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.publisher = self.robot.rospy.Publisher('promptQues', GUItext, queue_size=1)
        self.is_finish_publisher = self.robot.rospy.Publisher('is_interview_finished', Bool, queue_size=1)
        self.encoded_robot_feedback_pub = self.robot.rospy.Publisher('robot_feedback_dict', String, queue_size=1)

    def init(self):
        elements_link = self.robot.driver.find_elements_by_class_name("enchantment-link")
        if len(elements_link) != 3:
            self.robot.driver.find_elements_by_tag_name("tw-link")[0].click()
            #time.sleep(1)
            timeout = 1.0
            try:
                WebDriverWait(self.robot.driver, timeout).until(lambda x: x.find_elements_by_tag_name("tw-expression"))
            except TimeoutException:
                print("Timed out waiting for page to load")
            exp_list = self.robot.driver.find_elements_by_tag_name("tw-expression")
            self.robot.ques_count = (len(exp_list)/(2*3))-1
            num_valid_feedback = 0
            for i in range(self.robot.ques_count):
                # self.robot.selected_ques_dict[i] = exp_list[3+i*3].text
                self.robot.feedback_dict[i] = exp_list[3*(self.robot.ques_count+1)+3+i*3].text
                if self.robot.feedback_dict[i] != "":
                    num_valid_feedback += 1
            encoded_robot_feedback = json.dumps(self.robot.feedback_dict)
            self.encoded_robot_feedback_pub.publish(String(encoded_robot_feedback))
            print("Number of feedback is ", num_valid_feedback)
            self.robot.is_interview_finished = True
            self.is_finish_publisher.publish(Bool(True))
            return
        GUItext_msg = GUItext()
        GUItext_msg.prev_answer = self.robot.robot_text
        GUItext_msg.ques1 = str(1) + ". " + elements_link[0].text
        GUItext_msg.ques2 = str(2) + ". " + elements_link[1].text
        GUItext_msg.ques3 = str(3) + ". " + elements_link[2].text
        self.publisher.publish(GUItext_msg)

        """
        msg = ""
        count = 1
        for elem in elements_link:
            msg = msg + str(count) + ". " + elem.text + '\n\n'
            count += 1
        self.publisher.publish(String(msg))
        """

    def step(self):
        pass

    def finalize(self):
        pass
