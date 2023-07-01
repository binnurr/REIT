

from planner.action.action import Action

class HeadTestAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        
        self.joint_names = ["HeadPitch","HeadPitch"]
        
    
    def init(self):
        self.robot.disable_static_motion_joint_names(self.joint_names)
        
    def step(self):
        self.robot.disable_static_motion_joint_names(self.joint_names)
        sensor_values = self.robot.get_sensor_values(self.joint_names)
        print (sensor_values)
    
    def finalize(self):
        self.robot.enable_static_motion_joint_names(self.joint_names)
