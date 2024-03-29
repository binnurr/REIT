from planner.action.action import Action
import time
import random


class SayFeedbackAct(Action):

    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        feedback_list = self.robot.feedback_dict[self.robot.current_ques]
        filtered_fb_list = []
        for feed in feedback_list:
            feed = feed.replace("Asking long/complex question", "Asking long question")
            feed = feed.replace("Showing lack of preparation", "Lack of preparation")
            feed = feed.replace("Lack of planning for interview", "Lack of preparation")
            if feed not in self.robot.feedback_exp_dict:
                print("Not found in dictionary: ", feed)
            else:
                filtered_fb_list.append(feed)

        if len(filtered_fb_list) == 0:
            robot_say = ""
        elif len(filtered_fb_list) == 1:
            if filtered_fb_list[0] in self.robot.given_feedback_cache_dict:
                feed0 = random.choice(self.robot.feedback_alt_exp_dict[filtered_fb_list[0]])
                self.robot.given_feedback_cache_dict[filtered_fb_list[0]] += 1
            else:
                feed0 = self.robot.feedback_exp_dict[filtered_fb_list[0]]
                self.robot.given_feedback_cache_dict[filtered_fb_list[0]] = 1
            feed0 = feed0.encode("utf-8")
            robot_say = feed0
        else:
            if filtered_fb_list[0] in self.robot.given_feedback_cache_dict:
                feed0 = random.choice(self.robot.feedback_alt_exp_dict[filtered_fb_list[0]])
                self.robot.given_feedback_cache_dict[filtered_fb_list[0]] += 1
            else:
                feed0 = self.robot.feedback_exp_dict[filtered_fb_list[0]]
                self.robot.given_feedback_cache_dict[filtered_fb_list[0]] = 1
            feed0 = feed0.encode("utf-8")

            if filtered_fb_list[1] in self.robot.given_feedback_cache_dict:
                feed1 = random.choice(self.robot.feedback_alt_exp_dict[filtered_fb_list[1]])
                self.robot.given_feedback_cache_dict[filtered_fb_list[1]] += 1
            else:
                feed1 = self.robot.feedback_exp_dict[filtered_fb_list[1]]
                self.robot.given_feedback_cache_dict[filtered_fb_list[1]] = 1
            feed1 = feed1.encode("utf-8")

            robot_say = "There are two problems in here. Firstly, " + feed0 + "  " + "Secondly, " + feed1
        # todo: why sleep in here
        time.sleep(2)
        print(robot_say)
        self.robot.say_text(robot_say)

    def step(self):
        pass

    def finalize(self):
        pass

    def set_text(self, text):
        self.text = text

    def pass_initial_say(self):
        self.pass_initial_time = True
