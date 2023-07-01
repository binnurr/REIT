

from behavior.RERobot_Exp2 import RERobot
import os

class FSCPlanner():
    
    # constructor
    def __init__(self):
        self.robot = None
        self.current_fsc = None
        
    # load the given fsc
    def load_fsc(self, fsc_name):
        print("load fsc " + fsc_name)

        if (fsc_name == RERobot.get_name()):
            self.current_fsc = RERobot(self.robot, "RERobot")
        
    # run the current fsc
    def run(self):
        #os.spawnl(os.P,'rosrun nao_trackuser trackUser -fileprefix aysel -readfromfile')
        if (self.current_fsc):
            self.current_fsc.run()
            
    def set_robot(self, robot):
        self.robot = robot
        
        
