from planner.action.action import Action
from std_msgs.msg import String, Bool
from fsc_py_planner.msg import GUItext
import time
import json


class PromptUserAct(Action):
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.publisher = self.robot.rospy.Publisher('promptQues', GUItext, queue_size=1)
        self.is_finish_publisher = self.robot.rospy.Publisher('is_interview_finished', Bool, queue_size=1)
        self.encoded_robot_feedback_pub = self.robot.rospy.Publisher('robot_feedback_dict', String, queue_size=1)
        self.encoded_robot_selected_ques_pub = self.robot.rospy.Publisher('robot_selected_ques_dict', String, queue_size=1)

    def init(self):
        options = self.robot.scenario.get_options()
        if len(options) == 0:
            self.robot.ques_count = len(self.robot.scenario.traversed_edges)
            num_valid_feedback = 0
            for i in range(self.robot.ques_count):
                self.robot.feedback_dict[i] = self.robot.scenario.traversed_mistakes[i]
                if self.robot.feedback_dict[i] != ["No mistake"]:
                    num_valid_feedback += 1
            encoded_robot_feedback = json.dumps(self.robot.feedback_dict)
            self.encoded_robot_feedback_pub.publish(String(encoded_robot_feedback))
            encoded_robot_selected_ques_dict = json.dumps(self.robot.selected_ques_dict)
            self.encoded_robot_selected_ques_pub.publish(String(encoded_robot_selected_ques_dict))
            print("Number of feedback is ", num_valid_feedback)
            self.robot.is_interview_finished = True
            self.is_finish_publisher.publish(Bool(True))
            return
        GUItext_msg = GUItext()
        GUItext_msg.prev_answer = self.robot.robot_text
        GUItext_msg.ques1 = str(1) + ". " + options[0]
        GUItext_msg.ques2 = str(2) + ". " + options[1]
        GUItext_msg.ques3 = str(3) + ". " + options[2]
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
