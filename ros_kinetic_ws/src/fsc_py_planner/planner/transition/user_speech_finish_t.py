

from planner.transition.transition import Transition
from std_msgs.msg import String
import time


class UserSpeechFinishT(Transition):

    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)

        # we access rospy from robot
        self.robot.rospy.Subscriber("user_speech_state", String, self.user_speech_finished)

        self.speech_finished = False

    # override only init method and condition method
    def init(self):
        self.speech_finished = False

    def condition(self):
        if self.speech_finished:
            return True
        else:
            return False

    # declare its own parameters here
    def user_speech_finished(self, msg):
        if msg.data == 'finish':
            self.speech_finished = True
            print('User speech finished. Now robot turn!')

