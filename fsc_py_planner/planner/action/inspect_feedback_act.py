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
            if self.robot.feedback_dict[i] != "":
                self.robot.current_ques = i
                break
            elements_link = self.robot.driver.find_elements_by_class_name("enchantment-link")
            elements_link[selected_ques].click()
            timeout = 0.5
            try:
                WebDriverWait(self.robot.driver, timeout).until(lambda x: x.find_elements_by_class_name("enchantment-link"))
            except TimeoutException:
                print("Timed out waiting for page to load")

        elements_link = self.robot.driver.find_elements_by_class_name("enchantment-link")
        GUItext_msg = GUItext()
        GUItext_msg.prev_answer = self.robot.driver.find_elements_by_tag_name("tw-hook")[1].text.encode("utf-8")
        GUItext_msg.ques1 = str(1) + ". " + elements_link[0].text
        GUItext_msg.ques2 = str(2) + ". " + elements_link[1].text
        GUItext_msg.ques3 = str(3) + ". " + elements_link[2].text
        self.publisher.publish(GUItext_msg)
        elements_link = self.robot.driver.find_elements_by_class_name("enchantment-link")
        elements_link[selected_ques].click()
        self.publisher2.publish(str(selected_ques+1))


    def step(self):
        pass

    def finalize(self):
        pass
