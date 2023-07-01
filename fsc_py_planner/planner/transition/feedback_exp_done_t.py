
from planner.transition.transition import Transition
from std_msgs.msg import Int16
import time


class FeedbackExpDoneT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.robot.rospy.Subscriber("feedback_exp_finished", Int16, self.message_callback, queue_size=1)
        self.wait_sec = 0
        self.msg_rec_time = None
        self.message_received = False

    def init(self):
        pass
        
    def condition(self):
        #print(self.message_received)
        #print(self.msg_rec_time)
        #print(self.robot.is_say_finished())
        if self.message_received:
            if time.time()-self.msg_rec_time > self.wait_sec:
                if self.robot.is_say_finished():
                    self.wait_sec = 0
                    self.msg_rec_time = None
                    self.message_received = False
                    return True
        else:
            return False

    # after robot speech is done, if you want to allow some seconds for the user, use this function
    def wait_for_sec(self, wait_sec):
        self.wait_sec = wait_sec

    def message_callback(self, msg):
        print("MESSAGE RECEIVED")
        self.wait_sec = msg.data
        self.msg_rec_time = time.time()
        self.message_received = True
        
    
        