
from planner.action.action import Action
from convert_positions.msg import HeadAngles
from std_msgs.msg import String

class TrackObjectFaceAct(Action):

    # constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.will_lookat_user = False
        self.robot.rospy.Subscriber('nao_head_angles', HeadAngles, self.head_angles_callback)
        self.robot.rospy.Subscriber('user_speaking_start', String, self.user_speaking_start_callback)
        self.robot.rospy.Subscriber('user_speaking_stop', String, self.user_stop_speaking_callback)

    def init(self):
        self.robot.set_head_stiffness()
        self.will_lookat_user = False

    def step(self):
        now = self.robot.rospy.Time.now()
        if (now - self.robot.to_object_headanglelasttime > self.robot.rospy.Duration(5)):
            self.robot.track_face()
            print("looking at user")
        else:
            if (not self.will_lookat_user):
                self.robot.track_object()
                print("looking at object")
            else:
                if(now-self.lasttime_will_lookat_user > self.robot.rospy.Duration(5)):
                    self.robot.track_object()
                    print("looking at object")
                else:
                    self.robot.track_face()
                    print("looking at user")

    def finalize(self):
        pass

    def head_angles_callback(self, msg):
        self.robot.to_object_headpitch = msg.headpitch
        self.robot.to_object_headyaw = msg.headyaw
        self.robot.to_object_headanglelasttime = self.robot.rospy.Time.now()

    def user_speaking_start_callback(self, msg):
        if (msg.data == 'true'):
            self.will_lookat_user = True
            self.lasttime_will_lookat_user = self.robot.rospy.Time.now()


    def user_stop_speaking_callback(self, msg):
        if (msg.data == 'true'):
            self.will_lookat_user = False