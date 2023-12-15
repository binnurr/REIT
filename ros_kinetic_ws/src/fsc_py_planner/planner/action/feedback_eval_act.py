from planner.action.action import Action
from std_msgs.msg import Int16
import time
import random


class FeedbackEvalAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)
        self.publisher = self.robot.rospy.Publisher('feedback_eval_state', Int16, queue_size=1)

    def init(self):
        time.sleep(2)
        if self.robot.correct_questions[str(self.robot.current_ques)] == -1:
            say_text = "no! this is inappropriate choice again. actually there is no correct choice in here. this is a bit tricky!"
            feedback_eval_state = -1
        elif int(self.robot.selected_ques_ind) == self.robot.correct_questions[str(self.robot.current_ques)]:
            available_pos_set = ["correct! congrats!", "super!", "well done!", "great job!", "bravo!"]
            say_text = available_pos_set[random.choice(range(len(available_pos_set)))]
            feedback_eval_state = 1
        else:
            available_neg_set = ["no! this is inappropriate choice again.", "I am afraid, this is not correct.",
                                 "unfortunately not correct again", "sorry but this is also incorrect"]
            say_text = available_neg_set[random.choice(range(len(available_neg_set)))]
            feedback_eval_state = 0
        print(say_text)
        self.robot.say_text(say_text)
        if feedback_eval_state == 1:
            pos_emo_files = ['pride0', 'pride1', 'pride2', 'pride3', 'pride4']
            names, times, keys = self.robot.read_emotion_file(random.choice(pos_emo_files), 0.0)
            self.robot.motion_manager_proxy.angleInterpolation(names, keys, times, True)
        elif feedback_eval_state == 0:
            self.robot.second_failure_count += 1
            neg_emo_files = ['shame0', 'shame1', 'shame2', 'shame3', 'shame4', 'shame5']
            names, times, keys = self.robot.read_emotion_file(random.choice(neg_emo_files), 0.0)
            self.robot.motion_manager_proxy.angleInterpolation(names, keys, times, True)
        self.publisher.publish(Int16(feedback_eval_state))

        feedback_available = False
        for i in range(self.robot.current_ques + 1, self.robot.ques_count):
            if self.robot.feedback_dict[i] != "":
                feedback_available = True
                break
        if not feedback_available:
            self.robot.is_feedback_finished = True
        time.sleep(2)

    def step(self):
        pass

    def finalize(self):
        pass

