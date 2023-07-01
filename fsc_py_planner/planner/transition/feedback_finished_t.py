
from planner.transition.transition import Transition
import time


class FeedbackFinishedT(Transition):

    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)

    # override only init method and condition method
    def init(self):
        pass

    def condition(self):
        if self.robot.is_feedback_finished:
            time.sleep(3)
            return True
        else:
            return False







