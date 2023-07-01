from planner.transition.transition import Transition

class ValueCheckT(Transition):

    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.value1 = None
        self.value2 = None
        self.op = None

    # override only init method and condition method
    def init(self):
        return True

    def condition(self):
        if isinstance(self.value1,(list,)):
            checkval1 = self.value1[0]
        else:
            checkval1 = self.value1
        if isinstance(self.value2,(list,)):
            checkval2 = self.value2[0]
        else:
            checkval2 = self.value2
        #print('I am hre' + str(self.value1) + str(self.value2) + str(self.op))
        if self.op == '<':
            if checkval1 < checkval2:
                print("true")
                return True
            else:
                return False
        elif self.op == '<=':
            if checkval1 <= checkval2:
                return True
            else:
                return False
        elif self.op == '>':
            if checkval1 > checkval2:
                return True
            else:
                return False
        elif self.op == '>=':
            if checkval1 >= checkval2:
                return True
            else:
                return False
        elif self.op == '==':
            if checkval1 == checkval2:
                return True
            else:
                return False
        else:
            return False

    # allowed op: <, <=, >, >=, ==
    def set_values_operator(self, value1, value2, op):
        self.value1 = value1
        self.value2 = value2
        self.op = op