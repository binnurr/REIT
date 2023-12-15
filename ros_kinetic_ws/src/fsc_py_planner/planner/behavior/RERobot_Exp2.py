# -*- coding: utf-8 -*-

from planner.fsc import FSC
from planner.state import State
from planner.transition.time_elapsed_t import TimeElapsedT
from planner.transition.static_motion_done_t import StaticMotionDoneT
from planner.transition.ros_msg_t import RosMsgT
#from planner.transition.speech_done_t import SpeechDoneT
from planner.transition.remote_speech_done_t import SpeechDoneT
from planner.transition.always_true_t import AlwaysTrueT
# from planner.transition.speech_recognized_t import SpeechRecognizedT
# from planner.transition.speech_recognized_naoqi_t import SpeechRecognizedNaoqiT
from planner.transition.user_speech_finish_t import UserSpeechFinishT
from planner.transition.object_exp_done_t import ObjectExpDoneT
from planner.transition.value_check_t import ValueCheckT
from planner.transition.binary_t import BinaryT
from planner.transition.interview_finished_t import InterviewFinishedT
from planner.transition.feedback_finished_t import FeedbackFinishedT
from planner.transition.feedback_evaluated_t import FeedbackEvaluatedT
from planner.transition.feedback_exp_done_t import FeedbackExpDoneT

from planner.action.static_motion_act import StaticMotionAct
from planner.action.say_text_act import SayTextAct
from planner.action.send_ros_msg_act import SendRosMsgAct
from planner.action.say_ros_msg_act import SayRosMsgAct
from planner.action.say_ros_msg_with_text_act import SayRosMsgWithTextAct
from planner.action.sitdown_act import SitDownAct
from planner.action.track_head_motion_act import TrackHeadMotionAct
from planner.action.random_arm_motion_act import RandomArmMotionAct
#from planner.action.speech_recognition_act import SpeechRecognitionAct
#from planner.action.track_object_face_act import TrackObjectFaceAct
from planner.action.empty_act import EmptyAct
from planner.action.eye_blinking_act import EyeBlinkingAct
from planner.action.change_mode_act import ChangeModeAct
from planner.action.express_emotion import ExpressEmotionAct
from planner.action.decide_emotion import DecideEmotionAct
from planner.action.prompt_user_act_exp2 import PromptUserAct
from planner.action.say_client_response_act_exp2 import SayClientResponseAct
from planner.action.inspect_feedback_act_exp2 import InspectFeedbackAct
from planner.action.say_feedback_act_exp2 import SayFeedbackAct
from planner.action.say_client_response_again_act import SayClientResponseAgainAct
from planner.action.wait_act import WaitAct
from planner.action.feedback_eval_act_exp2 import FeedbackEvalAct
from planner.action.feedback_stat_act_exp2 import FeedbackStatAct
from planner.action.say_try_again_act_exp2 import SayTryAgainAct

import random


class RERobot(FSC):

    @staticmethod
    def get_name():
        return 'RERobot'

    # constructor
    def __init__(self, robot, name):
        # call base classes constructor
        FSC.__init__(self, robot, name)

        s_idle = State("s_idle")
        s_greeting = State("s_greeting")
        s_prompt_question_from_user = State("s_prompt_question_from_user")
        s_repeat_response = State("s_repeat_response")
        s_answer_user = State("s_answer_user")
        s_finish = State("s_finish")
        s_feedback_exp = State("s_feedback_exp")
        s_ask_feedback_corr = State("s_ask_feedback_corr")
        s_eval_feedback_corr = State("s_eval_feedback_corr")
        s_ending = State("s_ending")

        ########## DEFINE ACTIONS ##########
        track_head_motion_act = TrackHeadMotionAct(robot)
        eye_blinking_act = EyeBlinkingAct(robot)
        act_random_arm_motion = RandomArmMotionAct(robot)

        s_idle.add_action(track_head_motion_act)
        s_idle.add_action(eye_blinking_act)
        #s_idle.add_action(act_random_arm_motion)

        act_say_greeting = SayTextAct(robot)
        act_say_greeting.set_text('intro')

        s_greeting.add_action(act_say_greeting)
        s_greeting.add_action(act_random_arm_motion)
        s_greeting.add_action(track_head_motion_act)
        s_greeting.add_action(eye_blinking_act)

        act_prompt_user = PromptUserAct(robot)

        s_prompt_question_from_user.add_action(act_prompt_user)
        #s_prompt_question_from_user.add_action(act_random_arm_motion)
        s_prompt_question_from_user.add_action(track_head_motion_act)
        s_prompt_question_from_user.add_action(eye_blinking_act)

        act_say_client_response = SayClientResponseAct(robot)

        s_answer_user.add_action(act_say_client_response)
        s_answer_user.add_action(act_random_arm_motion)
        s_answer_user.add_action(track_head_motion_act)
        s_answer_user.add_action(eye_blinking_act)

        act_say_client_response_again = SayClientResponseAgainAct(robot)
        s_repeat_response.add_action(act_say_client_response_again)
        s_repeat_response.add_action(act_random_arm_motion)
        s_repeat_response.add_action(track_head_motion_act)
        s_repeat_response.add_action(eye_blinking_act)

        act_say_ending = SayTextAct(robot)
        act_say_ending.set_text('ending')

        s_finish.add_action(act_say_ending)
        s_finish.add_action(track_head_motion_act)
        s_finish.add_action(eye_blinking_act)

        inspect_feedback_act = InspectFeedbackAct(robot)
        say_feedback_act = SayFeedbackAct(robot)
        s_feedback_exp.add_action(inspect_feedback_act)
        s_feedback_exp.add_action(act_random_arm_motion)
        s_feedback_exp.add_action(track_head_motion_act)
        s_feedback_exp.add_action(eye_blinking_act)
        s_feedback_exp.add_action(say_feedback_act)

        act_say_try_again = SayTryAgainAct(robot)

        s_ask_feedback_corr.add_action(act_say_try_again)
        #s_ask_feedback_corr.add_action(act_random_arm_motion)
        s_ask_feedback_corr.add_action(track_head_motion_act)
        s_ask_feedback_corr.add_action(eye_blinking_act)

        #act_say_eval_again = SayTextAct(robot)
        #act_say_eval_again.set_text('That is correct. Congrats!')

        feedback_eval_act = FeedbackEvalAct(robot)

        wait_act = WaitAct(robot)

        s_eval_feedback_corr.add_action(feedback_eval_act)
        # todo: remove
        #s_eval_feedback_corr.add_action(wait_act)
        s_eval_feedback_corr.add_action(act_random_arm_motion)
        s_eval_feedback_corr.add_action(track_head_motion_act)
        s_eval_feedback_corr.add_action(eye_blinking_act)

        act_send_bye_msg = SendRosMsgAct(robot)
        act_send_bye_msg.set_topic_and_msg('exp_finished', 'true')
        act_feedback_stat = FeedbackStatAct(robot)
        act_say_conclude = SayTextAct(robot)
        act_say_conclude.set_text('conclude')
        s_ending.add_action(act_feedback_stat)
        s_ending.add_action(act_send_bye_msg)
        s_ending.add_action(act_say_conclude)
        #s_ending.add_action(act_random_arm_motion)
        s_ending.add_action(track_head_motion_act)
        s_ending.add_action(eye_blinking_act)

        ########## DEFINE TRANSITIONS ##########
        # t_time_elapsed1 = TimeElapsedT(self, robot)
        # t_time_elapsed1.set_elapsed_time(3)

        t_ros_msg_is_start = RosMsgT(self, robot)
        t_ros_msg_is_start.set_any_value(True)
        t_ros_msg_is_start.set_topic_and_value('is_start', ['True'])
        t_ros_msg_is_start.set_target_state(s_greeting)

        s_idle.add_transition(t_ros_msg_is_start)

        t_say_greeting = SpeechDoneT(self, robot)
        t_say_greeting.set_target_state(s_prompt_question_from_user)
        s_greeting.add_transition(t_say_greeting)

        t_interview_finished = InterviewFinishedT(self, robot)
        t_interview_finished.set_target_state(s_finish)

        t_ros_msg_ques_selected = RosMsgT(self, robot)
        t_ros_msg_ques_selected.set_any_value(True)
        t_ros_msg_ques_selected.set_attribute(robot, "selected_ques_ind")
        t_ros_msg_ques_selected.set_topic_and_value('selected_ques', ['1', '2', '3'])
        t_ros_msg_ques_selected.set_target_state(s_answer_user)

        t_ros_msg_repeat_requested = RosMsgT(self, robot)
        t_ros_msg_repeat_requested.set_any_value(True)
        t_ros_msg_repeat_requested.set_topic_and_value('repeat_request', ['True'])
        t_ros_msg_repeat_requested.set_target_state(s_repeat_response)

        s_prompt_question_from_user.add_transition(t_interview_finished)
        s_prompt_question_from_user.add_transition(t_ros_msg_repeat_requested)
        s_prompt_question_from_user.add_transition(t_ros_msg_ques_selected)

        t_say_response_again = SpeechDoneT(self, robot)
        t_say_response_again.set_target_state(s_prompt_question_from_user)
        s_repeat_response.add_transition(t_say_response_again)

        t_say_answer = SpeechDoneT(self, robot)
        t_say_answer.set_target_state(s_prompt_question_from_user)
        s_answer_user.add_transition(t_say_answer)

        t_say_feedback = SpeechDoneT(self, robot)
        t_say_feedback.set_target_state(s_feedback_exp)

        s_finish.add_transition(t_say_feedback)

        t_say_explain_feedback = SpeechDoneT(self, robot)
        t_say_explain_feedback.set_target_state(s_ask_feedback_corr)
        s_feedback_exp.add_transition(t_say_explain_feedback)

        t_ros_msg_feedback_corr_selected = RosMsgT(self, robot)
        t_ros_msg_feedback_corr_selected.set_any_value(True)
        t_ros_msg_feedback_corr_selected.set_attribute(robot, "selected_ques_ind")
        t_ros_msg_feedback_corr_selected.set_topic_and_value('selected_feedback_ques', ['1', '2', '3', '0'])

        t_ros_msg_feedback_corr_selected.set_target_state(s_eval_feedback_corr)
        s_ask_feedback_corr.add_transition(t_ros_msg_feedback_corr_selected)

        t_feedback_finished = FeedbackFinishedT(self, robot)
        t_feedback_finished.set_target_state(s_ending)
        s_eval_feedback_corr.add_transition(t_feedback_finished)

        t_say_explain_feedback_corr = FeedbackExpDoneT(self, robot)
        t_say_explain_feedback_corr.set_target_state(s_feedback_exp)
        s_eval_feedback_corr.add_transition(t_say_explain_feedback_corr)

        # add states to behavior
        self.states.append(s_greeting)
        self.states.append(s_prompt_question_from_user)
        self.states.append(s_repeat_response)
        self.states.append(s_answer_user)
        self.states.append(s_finish)
        self.states.append(s_feedback_exp)
        self.states.append(s_ask_feedback_corr)
        self.states.append(s_eval_feedback_corr)
        self.states.append(s_ending)

        # set initial state of FSC
        self.set_initial_state(s_idle)
