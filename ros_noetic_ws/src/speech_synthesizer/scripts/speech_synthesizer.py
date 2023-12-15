import rospy
from std_msgs.msg import String
from pydub import AudioSegment
from pydub.playback import play
import os
from gtts import gTTS 


class SpeechSynthesizer:
    def __init__(self):
        rospy.init_node('speech_synthesizer', anonymous=True)
        rospy.Subscriber('text_to_say', String, self.say_text, queue_size=1)
        self.pub = rospy.Publisher('remote_speech_finished', String, queue_size=10)
        rospy.spin()

    def say_text(self, msg):
        tts = gTTS(text=msg.data, lang='en')
        filename = 'temp.mp3'
        tts.save(filename)
        audio_f = AudioSegment.from_file(filename)
        play(audio_f)
        os.remove(filename)
        self.pub.publish(String("finish"))

if __name__ == '__main__':
    SpeechSynthesizer()
