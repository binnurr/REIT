

from planner.action.action import Action
import time
from std_msgs.msg import String
from fsc_py_planner.srv import GoogleSTT

class SpeechRecognitionAct(Action):
    
    #constructor
    def __init__(self, robot):
        Action.__init__(self, robot)
        self.user_speech_finished = False
        self.label_pub = None
        self.ground_t_pub = None
        self.google_stt_server = None
        self.service_started = True
        self.publisher = None
        self.robot.rospy.Subscriber('user_speaking_stop', String, self.speech_message_callback, queue_size=1)
        self.label_pub = self.robot.rospy.Publisher('user_text_input', String, queue_size=1)
        self.ground_t_pub = self.robot.rospy.Publisher('user_ground_t', String, queue_size=1)
        self.pref_pub = self.robot.rospy.Publisher('user_pref', String, queue_size=1)

        try:
            self.robot.rospy.wait_for_service('google_stt', 2.0)
            self.google_stt_server = self.robot.rospy.ServiceProxy('google_stt', GoogleSTT)
        except (self.robot.rospy.ServiceException, self.robot.rospy.ROSException), e:
            self.robot.rospy.logerr("Google Speech recognition service can not be started!")
            self.service_started = False

    def set_publisher(self, pub_name):
        if pub_name == 'user_label':
            self.publisher = self.label_pub
        elif pub_name == 'user_ground_t':
            self.publisher = self.ground_t_pub
        elif pub_name == 'user_pref':
            self.publisher = self.pref_pub

    def init(self):
        self.user_speech_finished = False

    def step(self):
        if self.service_started:
            if self.user_speech_finished:
                self.user_speech_finished = False
                label = None
                try:
                    resp = self.google_stt_server("foo")
                    label = resp.label
                except self.robot.rospy.ServiceException as e:
                    print("Service call failed:" + str(e))

                if label:
                    self.publisher.publish(String(label))
                else:
                    print("can not recognize user's speech for object label")
    
    def finalize(self):
        #self.robot.speech_recognition_stop()
        pass

    def speech_message_callback(self, msg):
        if msg.data == 'true':
            self.user_speech_finished = True
    
