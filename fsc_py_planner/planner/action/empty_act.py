from planner.action.action import Action


class EmptyAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)

    def init(self):
        pass

    def step(self):
        pass

    def finalize(self):
        pass