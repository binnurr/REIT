import os
import rospy
from std_msgs.msg import String
import time
import sys
import csv


class VideoClipTimeListener:
    def __init__(self, subject_name, session_type):
        rospy.init_node('video_clip_time_listener', anonymous=True)
        self.subject_name = subject_name
        self.session_type = session_type
        file_p_prefix = "/Users/binnurgorer/Documents/binnur_projects/FaceChannel-master/resources/experiment/requests"
        self.request_fp = os.path.join(file_p_prefix, subject_name, session_type, "request.csv")
        try:
            os.remove(self.request_fp)
        except OSError:
            pass
        os.makedirs(os.path.dirname(self.request_fp), exist_ok=True)
        self.speech_done_time = None
        self.selected_ques_time = None
        self.turn_counter = 0
        rospy.Subscriber('selected_ques', String, self.selected_ques_cb, queue_size=1)
        rospy.Subscriber('speech_done', String, self.speech_done_cb, queue_size=1)
        rospy.spin()

    def speech_done_cb(self, msg):
        self.speech_done_time = time.time()

    def selected_ques_cb(self, msg):
        self.selected_ques_time = time.time()
        with open(self.request_fp, mode="a") as employee_file:
            csv_writer = csv.writer(employee_file, delimiter=',')
            turn = 'turn_' + str(self.turn_counter)
            csv_writer.writerow([turn, self.speech_done_time, self.selected_ques_time])
        self.turn_counter = self.turn_counter + 1


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    subject_name = args[1]
    session_type = args[2]
    VideoClipTimeListener(subject_name, session_type)