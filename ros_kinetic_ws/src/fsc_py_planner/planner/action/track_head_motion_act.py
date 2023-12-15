
from planner.action.action import Action

class TrackHeadMotionAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
    
    def init(self):
        self.robot.disable_static_motion_joint_names(["HeadYaw","HeadPitch"])
        self.robot.set_head_stiffness()
       
    def step(self):
        self.robot.track_face()
    
    def finalize(self):
        pass
    