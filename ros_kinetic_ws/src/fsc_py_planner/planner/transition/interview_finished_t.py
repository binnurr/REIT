
from planner.transition.transition import Transition


class InterviewFinishedT(Transition):

    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)

    # override only init method and condition method
    def init(self):
        pass

    def condition(self):
        if self.robot.is_interview_finished:
            return True
        else:
            return False







