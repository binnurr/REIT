

from planner.transition.transition import Transition
from nao_speechtotext.msg import SpeechRecognition
from std_msgs.msg import String

class SpeechRecognizedT(Transition):
    
    def __init__(self, fsc, robot):
        Transition.__init__(self, fsc, robot)
        self.words = []
        
        # speech recognition module specific information
        self.listen_publisher = None
        self.listen_topic_name = "listen"
        self.speech_recognition_topic_name = "speech_recognition"
        
        self.recognized_words = ''
        self.recognition_confidence = 0.0
        self.is_recognized = False
        
        # wire ros publisher for listen
        self.listen_publisher = self.robot.rospy.Publisher(self.listen_topic_name, String, latch=True)
        # wire ros subscriber for getting recognized text
        self.robot.rospy.Subscriber(self.speech_recognition_topic_name, SpeechRecognition, self.speech_recognized_callback)
        self.is_recognized = False
    
    # override only init method and condition method
    def init(self):
        
        # publish message to start listening
        listenMsg = String()
        listenMsg.data = "listen"
        self.listen_publisher.publish(listenMsg)
        
        
    def condition(self):
        if (self.is_recognized):
            for word in self.words:
                if (word in self.recognized_words):
                    return True
        
        return False
        
        
    # set words to be recognized
    def set_words(self, words):
        self.words = words
        
    def speech_recognized_callback(self, recognition_data):
        self.is_recognized = recognition_data.isRecognized
        self.recognized_words = recognition_data.text.split()
        self.recognition_confidence = recognition_data.confidence
        
