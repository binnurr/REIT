
from planner.transition.transition import Transition 
from std_msgs.msg import Int32MultiArray
import time

class FaceDetectionT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)

        msg = Int32MultiArray()
        # we access rospy from robot
        self.robot.rospy.Subscriber("nao_face_detection", msg, self.face_detected)
        
        self.last_msg_time = None
        
        self.forget_time = 2.0
    
    # override only init method and condition method
    def init(self):
        return True
        
    def condition(self):
        if (self.last_msg_time):
            if ((time.time() - self.last_msg_time) > self.forget_time):
                return False
            else:
                return True
        return False
        
    # declare its own parameters here
    def face_detected(self, msg):
        face_x = msg.data[0]
        face_y= msg.data[1]
        face_width = msg.data[2]
        face_height = msg.data[3]
        
        self.last_msg_time = time.time()
        
        print("x:" + str(face_x) + " y:" + str(face_y) + " w:" + str(face_width) + " h:" + str(face_height))
        
        
