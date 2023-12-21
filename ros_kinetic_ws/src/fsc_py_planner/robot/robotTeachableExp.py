# -*- coding: utf-8 -*-


from naoqi import ALProxy
from naoqi import ALBroker
from threading import Thread
from operator import itemgetter
import time
import random
import math
import subprocess
import glob
import os
import sys
import importlib
from std_msgs.msg import String

from selenium import webdriver

from .emotion_fuzzy import EmotionFuzzy
from scenario.scenario_parser import ScenarioParser


# a high level class to provide communication between all robot modules and the planner
class Robot():
    # constructor
    def __init__(self, rospy, session_mode, scenario_id, behavioral_feedback):

        self.rospy = rospy
        self.session = session_mode
        self.scenario_id = scenario_id
        self.is_behavioral_feedback = behavioral_feedback
        # set parameters
        self.naoqi_ip = self.rospy.get_param("~naoqi_ip", "10.0.1.3")
        #self.naoqi_ip = self.rospy.get_param("~naoqi_ip", "127.0.0.1")
        self.simulator = False  # this was True for simulation
        self.threshold = 0.1
        self.noface = 0
        self.userName = None
        self.currentMotion = None
        self.randomYaw = random.uniform(-1.1, 1.1)
        self.randomPitch = random.uniform(-0.43, 0.1)
        self.memValue = "FaceDetected"
        # write joint names as the same order of motion file
        self.joint_names = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "LHipYawPitch", "LHipRoll",
                            "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll",
                            "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]

        self.arm_joints_names = ["LShoulderPitch", "LElbowYaw", "LHand", "RShoulderPitch", "RElbowYaw", "RHand"]
        self.arm_joint_directions = [1, 1, 1, 1]

        self.min_arm_joint_angles = [1.4, -0.78, 0.0, 1.4, 0.78, 0.0]
        self.max_arm_joint_angles = [0.9, -1.13, 0.5, 0.9, 1.13, 0.5]

        self.to_object_headpitch = 0.0
        self.to_object_headyaw = 0.0
        self.to_object_headanglelasttime = self.rospy.Time.now()

        self.mode = 0  # 0 is training, 1 is testing

        self.gaze_direction = 0  # 0->random search , 1-> to_object,  2-> to user
        self.last_face_angle_yaw = None
        self.last_face_angle_pitch = None

        self.objectcount_to_learn = 2  # this was 3
        self.learned_object_count = [0]
        self.predicted_obj = [None]
        self.ground_t_obj = [None]
        self.allowed_test_round = 2
        self.tested_object_count = [0]
        self.last_acc = ['0.0']

        self.sensor_suffix = "/Position/Sensor/Value"
        self.sensor_prefix = "Device/SubDeviceList/"

        self.mem_proxy = None
        self.motion_manager_proxy = None
        self.posture_proxy = None
        self.speech_proxy = None
        self.face_proxy = None
        self.sr_proxy = None
        self.led_proxy = None

        self.say_text_thread = None
        self.say_finished = True
        self.ledsOn = 0
        self.ledsOff = 0

        self.sound_file_ids = {}
        self.sound_file_texts = {}

        if self.session == "robot":
            self.init_naoqi_proxies()

        self.hope_level = 0.5
        self.joy_level = 0.5

        # emotion specific parameters
        self.motion_speed = 0.5  # [0, 1]
        self.motion_amplitude = 0.5  # [0, 1]
        self.speech_volume = 60  # 55 [30, 80]
        self.speech_pitch = 90  # 98 [90, 105]
        self.speech_speed = 65  # 75 [55, 95]
        self.blinking_count = 10

        self.emotion_fuzzy_system = EmotionFuzzy()
        self.notification_speech_list = [["bilemedim", "olmadı"], ["bildim", "doğru"]]
        self.emotion_exclamations = [["amaann", "aaiyy", "uuuğğff", "ööfff", "tüüühh", "yaaa"],
                                     ["aaiyy", "süper", "ohh"]]

        # todo: added for RE interview robot
        self.driver = None
        self.is_interview_finished = False
        self.is_feedback_finished = False
        self.selected_ques_ind = None
        # self.init_web_driver()
        scenario_parser = ScenarioParser(self.scenario_id)
        self.scenario = scenario_parser.init_scenario_graph()
        self.robot_text = ""
        self.feedback_dict = {}
        self.selected_ques_dict = {}
        self.ques_count = 0
        self.current_ques = -1
        self.feedback_exp_dict = {}
        self.feedback_alt_exp_dict = {}
        self.correct_questions = {}
        self.given_feedback_cache_dict = {}
        self.second_failure_count = 0
        self.feedback_eval_states = {}

        self.load_feedback_file()
        self.goto_posture('Crouch', 1.0)
        time.sleep(1.0)
        """
        self.motion_manager_proxy.setStiffnesses(["HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll",
                                                  "RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll", "LHand", "RHand"], [0.9]*12)
        """
        #time.sleep(0.5)


    def init_naoqi_proxies(self):
        if self.session != "robot":
            return
        self.port = 9559

        # init memory proxy to read sensor values
        self.mem_proxy = ALProxy("ALMemory", self.naoqi_ip, self.port)

        try:
            self.motion_manager_proxy = ALProxy("ALMotion", self.naoqi_ip, self.port)
        except Exception as e:
            print ("Error when creating Motion Manager proxy:")
            print (str(e))

        try:
            self.posture_proxy = ALProxy("ALRobotPosture", self.naoqi_ip, self.port)
        except Exception as e:
            print ("Error when creating Posture proxy:")
            print (str(e))

        try:
            self.sr_proxy = ALProxy("ALSpeechRecognition", self.naoqi_ip, self.port)
            self.sr_proxy.setLanguage("Turkish")
            self.vocabulary = ["tamam", "evet", "hayır"]
            self.sr_proxy.setVocabulary(self.vocabulary, False)
            # pass
        except Exception as e:
            print ("Error when creating speech recognition proxy")
            print (str(e))

        try:
            self.face_proxy = ALProxy("ALFaceDetection", self.naoqi_ip, self.port)
            period = 100
            self.face_proxy.subscribe("Test_Face", period, 0.0)
        except Exception as e:
            print ("Error when creating face detection proxy:")
            print (str(e))

        try:
            self.led_proxy = ALProxy("ALLeds", self.naoqi_ip, self.port)
        except Exception as e:
            print ("Could not create proxy to ALLeds")
            print (str(e))

        try:
            self.speech_proxy = ALProxy("ALTextToSpeech", self.naoqi_ip, self.port)
            self.audio_proxy = ALProxy("ALAudioPlayer", self.naoqi_ip, self.port)
            self.load_sound_files()  # load sound files
        except Exception as e:
            print(e)

    def init_web_driver(self):
        if self.driver:
            self.driver.close()
        options = webdriver.ChromeOptions()
        options.add_argument("headless")
        self.driver = webdriver.Chrome(r"/home/binnur/Documents/robotics/ros_ws_learner_robot/src/chromedriver",
                                       chrome_options=options)
        self.driver.get("http://www.interviewsim.com.s3-website.us-east-2.amazonaws.com/")
        # self.driver.minimize_window()
        # time.sleep(3)

    def load_feedback_file(self):
        feedback_file_name = '/home/binnur/Documents/robotics/ros_ws_learner_robot/sounds/re_feedback_en.txt'
        feedback_file = open(feedback_file_name)
        for line in feedback_file:
            splitlines = line.split('-')
            self.feedback_exp_dict[splitlines[0]] = splitlines[1].rstrip('\n')
        feedback_file.close()

        self.feedback_exp_dict = {
            'No rapport with customer': 'You did not show rapport with customer. '
                                        'You need to show that you care about the customer. '
                                        'Be appreciative for the customer\'s participation in the interview.',
            'Asking long question': 'You asked complex or long question. '
                                            'Ask precise questions. '
                                            'Complex or long question can overwhelm the customer.',
            'Unnatural dialog style': 'You performed unnatural dialogue style in here. '
                                        'Give the customer freedom in what they want to say. '
                                        'Let the customers create scenarios.',
            'Lack of preparation': 'It seems that this choice is lack of preparation.'
                                   'You may not know everything in advance but do research about the domain'
                                   ' to get an idea of the vocabulary that the customer might use.',
            'Asking customer for solution': 'You asked customer for solution. '
                                            'Do not explicitly ask your stakeholders what they want. '
                                            'You need to collect your stakeholder\'s needs and learn their perspective '
                                            'and domain.',
            'Influencing customer': 'This choice may influence the customer. '
                                    'Please try to understand the customer\'s problems first and do not look for a solution '
                                    'right away.',
            'Asking vague question': 'You should not ask unclear questions. '
                                     'Instead, ask open ended questions and let the customer speak freely. ',
            'Asking technical question': 'Do not ask technical questions. Many stakeholders are not '
                                         'familiar with computer science terminology and concepts.',
            'Not asking about existing system': 'You should ask about the existing system to understand it as is. '
                                                'You need to identify what are the problems with the existing system '
                                                'and what you can modify, replace or integrate.',
            'Not identifying stakeholders': 'Your interviewee only knows part of the information. '
                                            'You need to understand which information the interviewee '
                                            'does not know and ask who could answer your additional questions.',
            'Asking unnecessary question': 'This choice is an unnecessary question. '
                                           'All questions need to be relevant to the development of the system.',
            'Incorrect ending of interview': 'You ended the interview incorrectly. '
                                             'Do not forget to thank the interviewee for being in the interview. '
                                             'You can explain what you learnt in the meeting and what the next steps are.'
        }

        self.feedback_alt_exp_dict = {
            'No rapport with customer': ['You did not show rapport with customer. There are some ways of doing it. For example do not forget to thank customer for their time. ',
                                        'You lacked customer rapport. It can be done in a variety of ways. For example try to empathize with the customers\' problems. '],
            'Asking long question': ['You asked long or complex question. Do not ask multiple questions at the same time.',
                                            'You asked complex question. Do not ask difficult or very long questions.'],
            'Unnatural dialog style': ['You performed unnatural dialogue style. Do not ask interrogative questions.'],
            'Lack of preparation': ['You seem to be lack of preparation. Do not ask questions available in the project description.',
                                   'You showed lack of preparation. Try to show enough knowledge and interest about the domain.'],
            'Asking customer for solution': ['You asked customer for solution. Do not ask questions like How do you want the system to manage reservations?',
                                            'You asked customer for solution. Do not ask questions like How should I fix the problem of customers reserving a spot at the same time?'],
            'Influencing customer': ['Your choice in here may influence the customer. Do not influence the customer towards a particular solution.'],
            'Asking vague question': ['You should not ask unclear questions. '
                                     'Do not ask questions that might confuse the customers.',
                                      'You should not ask unclear questions like the customer might not know the answer of.',
                                      'You should not ask unclear questions that might not have a definitive answer.'],
            'Asking technical question': ['Do not ask technical questions. Do not ask questions with technical jargon. ',
                                         'Do not ask technical questions like what is the current business process? ',
                                          'Do not ask technical questions like what machine learning solution do you want '
                                          'to use ?'],
            'Not asking about existing system': ['You should ask about existing system to understand it as is. '],
            'Not identifying stakeholders': ['You did not identify stakeholders. You should ask like Besides you is there'
                                             'anyone else who should be able to view your inventory report?',
                                            'You did not identify other stakeholders. You can ask like Is there '
                                            'anyone I can contact to get a more detailed understanding of your booking system? '],
            'Asking unnecessary question': ['Do not ask questions that does not pertain to the development '
                                            'of the requested system.'],
            'Incorrect ending of interview': ['Carefully end the interview. For example provide a brief summary of the interview.']
        }

    ############## SPEECH RECOGNITION MODULE FUNCTIONS ##############
    def speech_recognition_start(self):
        if self.session != "robot":
            return
        self.sr_proxy.subscribe("Test_ASR")
        print ('Speech recognition engine started')

    def speech_recognition_stop(self):
        if self.session != "robot":
            return
        self.sr_proxy.unsubscribe("Test_ASR")
        print ('Speech recognition engine stopped')

    # check if an interested word is recognized by speech recognition engine (return True or False)
    def speech_recognition(self):
        if self.session != "robot":
            return
        recognized = self.mem_proxy.getData("SpeechDetected")
        check_recognition = False
        if (recognized):
            recognized_words = self.mem_proxy.getData("WordRecognized")
            try:
                i = self.vocabulary.index(recognized_words[0])
                confidence = recognized_words[1]
            except ValueError:
                i = -1  # no match
                confidence = 0
            if confidence > 0.6:
                print ("I heard " + str(recognized_words[0]) + " with confidence of " + str(confidence))
                check_recognition = True
        return check_recognition

    ## outdated ##
    def google_speech_recognition(self):
        command = "/home/binnur/Documents/robotics/python-docs-samples/venv/bin/python /home/binnur/Documents/robotics/python-docs-samples/speech/cloud-client/quickstart.py"
        list_of_files = glob.glob('/home/binnur/Documents/robotics/ros_ws_learner_robot/*.wav')
        wav_file_name = max(list_of_files, key=os.path.getctime)
        command = command + " " + wav_file_name
        returned_result = subprocess.check_output(command, shell=True, stderr=subprocess.STDOUT)
        label = None
        if returned_result:
            print('Google STT returned result: ' + returned_result)
            label_array = returned_result.rstrip().split()
            if len(label_array) > 1:
                print("heard more than one word for the name of the object")
            label = label_array[0]
        else:
            print('Google STT returned result is none')
        return label

    ############## TEXT TO SPEECH MODULE FUNCTIONS ##############

    def load_sound_files(self):
        if self.session != "robot":
            return
        if self.is_behavioral_feedback:
            sound_file_name = '/home/binnur/Documents/robotics/ros_ws_learner_robot/sounds/re_sounds_en_bf_on.txt'
        else:
            sound_file_name = '/home/binnur/Documents/robotics/ros_ws_learner_robot/sounds/re_sounds_en_bf_off.txt'
        
        filePrefix = '/home/nao/binnur/sound/'
        fileExt = '.wav'
        sounds_file = open(sound_file_name)
        # self.speech_proxy.setLanguage('Turkish')  # turn to Turkish
        self.speech_proxy.setLanguage('English')  # turn to English
        self.speech_proxy.setVoice('naoenu')
        self.speech_proxy.setParameter('pitchShift', 1.0)
        for line in sounds_file:
            splitlines = line.split('-')
            audioFileName = filePrefix + splitlines[0] + fileExt
            text = splitlines[1].rstrip('\n')

            new_text = "\RSPD=" + str(71) + "\ "
            new_text += "\VCT=" + str(106) + "\ "
            new_text += str(text)
            new_text += "\RST\ "
        #            self.speech_proxy.sayToFile(new_text, audioFileName)
        sounds_file.close()
        sounds_file = open(sound_file_name)
        for line in sounds_file:
            splitlines = line.split('-')
            audioFileName = filePrefix + splitlines[0] + fileExt
            text = splitlines[1].rstrip('\n')
            # fileId = self.audio_proxy.loadFile(audioFileName)
            # self.sound_file_ids[splitlines[0]] = fileId
            self.sound_file_texts[splitlines[0]] = text
        sounds_file.close()

    def say_text(self, text):
        if self.simulator:
            self.say_finished = True
            return
        if self.session != "robot":
            return
        try:
            if (self.say_text_thread != None):
                self.say_text_thread.join(0.1)
            self.say_finished = False
            self.speech_proxy.stopAll()
            # self.say_text_thread = Thread(target=self.say_text_thread_func, args=[text])
            self.say_text_thread = Thread(target=self.say_text_thread_func_WOAudioFile, args=[text])
            self.say_text_thread.start()
        except:
            pass

    def say_text_thread_func(self, text):
        if self.simulator:
            self.say_finished = True
            return
        if self.session != "robot":
            return
        # we cannot find audio file, run text to speech
        if (not text in self.sound_file_ids.keys()):
            new_text = "\\vol=" + str(self.speech_volume) + "\ "
            new_text += "\\rspd=" + str(self.speech_speed) + "\ "  # 71
            new_text += "\\vct=" + str(self.speech_pitch) + "\ "  # 106
            new_text += str(text)
            new_text += "\\rst\ "
            self.speech_proxy.say(new_text)
        else:
            self.audio_proxy.play(self.sound_file_ids[text])
        self.say_finished = True

    def say_text_thread_func_WOAudioFile(self, text):
        if self.simulator:
            self.say_finished = True
            return
        if self.session != "robot":
            return
        text_to_say = text
        # we cannot find audio file, run text to speech
        if (text in self.sound_file_texts.keys()):
            text_to_say = self.sound_file_texts[text]

        new_text = "\\vol=" + str(self.speech_volume) + "\ "
        new_text += "\\rspd=" + str(self.speech_speed) + "\ "  # 71
        new_text += "\\vct=" + str(self.speech_pitch) + "\ "  # 106
        new_text += str(text_to_say)
        new_text += "\\rst\ "
        self.speech_proxy.say(new_text)
        self.say_finished = True

    def is_say_finished(self):
        if self.session != "robot":
            return True
        return self.say_finished

    ############## LED FUNCTIONS ##############

    def leds_on(self):
        if self.session != "robot":
            return
        self.led_proxy.setIntensity("LeftFaceLedsBlue", 1.0)
        self.led_proxy.setIntensity("RightFaceLedsBlue", 1.0)
        self.led_proxy.setIntensity("LeftFaceLedsGreen", 0.0)
        self.led_proxy.setIntensity("RightFaceLedsGreen", 0.0)
        self.led_proxy.setIntensity("LeftFaceLedsRed", 0.0)
        self.led_proxy.setIntensity("RightFaceLedsRed", 0.0)

    def leds_off(self):
        if self.session != "robot":
            return
        self.led_proxy.setIntensity("LeftFaceLedsBlue", 0.0)
        self.led_proxy.setIntensity("RightFaceLedsBlue", 0.0)
        self.led_proxy.setIntensity("LeftFaceLedsGreen", 0.0)
        self.led_proxy.setIntensity("RightFaceLedsGreen", 0.0)
        self.led_proxy.setIntensity("LeftFaceLedsRed", 0.0)
        self.led_proxy.setIntensity("RightFaceLedsRed", 0.0)

    ############## TOUCH SENSOR ########################
    def is_touched(self):
        if self.session != "robot":
            return
        touch_back = self.get_memory_data("Device/SubDeviceList/LHand/Touch/Back/Sensor/Value")
        touch_left = self.get_memory_data("Device/SubDeviceList/LHand/Touch/Left/Sensor/Value")
        touch_right = self.get_memory_data("Device/SubDeviceList/LHand/Touch/Right/Sensor/Value")

        if touch_back == 1 or touch_left == 1 or touch_right == 1:
            return True
        else:
            return False

    ############## FACE DETECT & TRACK & SEARCH FUNCTIONS ##############
    def track_object(self):
        if self.session != "robot":
            return
        if (
                self.to_object_headyaw > 1.5 or self.to_object_headyaw < -1.5 or self.to_object_headpitch > 0.5 or self.to_object_headpitch < -0.6):
            print("Head angles to look at the object is out of limits!!")
            return

        if (self.gaze_direction != 1):
            self.gaze_direction = 1

        self.currentYaw = self.get_memory_data("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
        self.currentPitch = self.get_memory_data("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
        setting_time_yaw = (math.fabs(self.to_object_headyaw - self.currentYaw) / 0.48)
        setting_time_pitch = (math.fabs(self.to_object_headpitch - self.currentPitch) / 0.48)
        if (setting_time_yaw >= setting_time_pitch):
            setting_time = setting_time_yaw
        else:
            setting_time = setting_time_pitch
        self.set_angles(["HeadYaw", "HeadPitch"], [self.to_object_headyaw, self.to_object_headpitch], setting_time)

    def track_face(self):
        if self.session != "robot":
            return

        if (self.gaze_direction != 2):
            self.gaze_direction = 2
            if (self.last_face_angle_yaw and self.last_face_angle_pitch):
                self.currentYaw = self.get_memory_data("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
                self.currentPitch = self.get_memory_data("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
                setting_time_yaw = (math.fabs(self.last_face_angle_yaw - self.currentYaw) / 0.48)
                setting_time_pitch = (math.fabs(self.last_face_angle_pitch - self.currentPitch) / 0.48)
                if (setting_time_yaw >= setting_time_pitch):
                    setting_time = setting_time_yaw
                else:
                    setting_time = setting_time_pitch
                self.set_angles(["HeadYaw", "HeadPitch"], [self.last_face_angle_yaw, self.last_face_angle_pitch],
                                setting_time)

        time.sleep(0.1)
        if (not self.simulator):
            val = self.mem_proxy.getData(self.memValue)
        else:
            val = None
            val = [None, [[None, None]]]
        # Check whether we got a valid output.
        if (val and isinstance(val, list) and len(val) >= 2):

            # We detected faces !
            # For each face, we can read its shape info and ID.

            # First Field = TimeStamp.
            timeStamp = val[0]

            # Second Field = array of face_Info's.
            faceInfoArray = val[1]

            try:
                # Browse the faceInfoArray to get info on each detected face.
                self.noface = 0
                for j in range(len(faceInfoArray) - 1):
                    faceInfo = faceInfoArray[j]

                    # First Field = Shape info.
                    faceShapeInfo = faceInfo[0]

                    # Second Field = Extra info (empty for now).
                    faceExtraInfo = faceInfo[1]

                    if (self.simulator):
                        faceShapeInfo = [0.0, 0.0, 0.0, 100, 100]

                    self.currentYaw = self.get_memory_data("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
                    self.currentPitch = self.get_memory_data("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
                    self.targetYaw = float(self.currentYaw + faceShapeInfo[1])
                    self.targetPitch = float(self.currentPitch + faceShapeInfo[2])
                    if (
                            self.targetYaw < 1.5 and self.targetYaw > -1.5 and self.targetPitch < 0.5 and self.targetPitch > -0.6):
                        self.set_angles(["HeadYaw", "HeadPitch"], [self.targetYaw, self.targetPitch], 0.5);
                        self.last_face_angle_pitch = self.targetPitch
                        self.last_face_angle_yaw = self.targetYaw

                    # print "  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
                    # print "  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])

            except Exception as e:
                print ("Faces detected, but it seems obtained data is invalid. ALValue =")
                print (val)
                print ("Error msg %s" % (str(e)))
        else:
            # print "No face detected"
            self.noface = self.noface + 1
            self.currentYaw = self.get_memory_data("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
            self.currentPitch = self.get_memory_data("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
            if (math.fabs(self.currentYaw - self.randomYaw) < self.threshold and math.fabs(
                    self.currentPitch - self.randomPitch) < self.threshold):
                self.randomYaw = random.uniform(-0.6, 0.6)
                # self.randomPitch = random.uniform(-0.1,0.4)       #(-0.43,0.1)
                self.randomPitch = random.uniform(-0.45, 0.1)
            # print randomYaw
            # print randomPitch
            if (self.noface > 10):
                setting_time_yaw = (math.fabs(self.randomYaw - self.currentYaw) / 0.48)
                setting_time_pitch = (math.fabs(self.randomPitch - self.currentPitch) / 0.48)
                if (setting_time_yaw >= setting_time_pitch):
                    setting_time = setting_time_yaw
                else:
                    setting_time = setting_time_pitch
                self.set_angles(["HeadYaw", "HeadPitch"], [self.randomYaw, self.randomPitch], setting_time);

    def get_sensor_values(self, sensor_names):
        if self.session != "robot":
            return
        full_sensor_names = []
        # get the correct key names
        for name in sensor_names:
            full_sensor_names.append(self.sensor_prefix + name + self.sensor_suffix)

        return self.mem_proxy.getListData(full_sensor_names)

    def get_memory_data(self, key):
        if self.session != "robot":
            return
        return self.mem_proxy.getData(key)

    ################ MOTION MANAGER MODULE FUNCTIONS ################

    def release_stiffness(self):
        if self.session != "robot":
            return
        self.motion_manager_proxy.setStiffnesses('Body', 0.0)

    def set_head_stiffness(self):
        if self.session != "robot":
            return
        self.motion_manager_proxy.setStiffnesses(['HeadYaw', 'HeadPitch'], [0.8, 0.8])

    def disable_static_motion_joint_names(self, names):
        pass

    def enable_static_motion_joint_names(self, names):
        pass

    def run_static_motion(self, motion_name):
        if self.session != "robot":
            return
        self.currentMotion = motion_name

    def reset_current_static_motion(self):
        pass

    def stop_current_static_motion(self):
        pass

    def is_static_motion_finished(self):
        pass

    def set_angles(self, names, angles, duration):
        if self.session != "robot":
            return
        # angles in radian, duration in seconds
        self.fraction_maxspeed = 0.1
        self.motion_manager_proxy.setAngles(names, angles, self.fraction_maxspeed)

    def goto_posture(self, posture_name, duration):
        if self.session != "robot":
            return
        # available postures: StandInit, SitRelax, StandZero, LyingBelly, LyingBack, Stand, Crouch, Sit
        self.motion_manager_proxy.setStiffnesses('Body', 0.9)
        time.sleep(0.5)
        self.posture_proxy.goToPosture(posture_name, duration)

    def random_arm_motion(self):
        if self.session != "robot":
            return
        indexes = random.sample(range(0, 5), 3)
        names = []
        angles = []
        for ind in indexes:
            names.append(self.arm_joints_names[ind])
            angles.append(random.uniform(self.max_arm_joint_angles[ind], self.min_arm_joint_angles[ind]))

        self.motion_manager_proxy.angleInterpolation(names, angles, 0.75, True)

    def express_emotion(self, pre_emotion):
        if self.session != "robot":
            return
        if pre_emotion:
            emotion_to_show = 'hope'
            emotion_motion_file = emotion_to_show
            names, times, keys = self.read_emotion_file(emotion_motion_file, self.hope_level)
            self.motion_manager_proxy.angleInterpolation(names, keys, times, True)
        else:
            emotion_to_show = 'joy'
            emotion_motion_file = emotion_to_show
            if self.predicted_obj[0] == self.ground_t_obj[0]:
                print("Nao learned the object!")
                prediction_success = 1
            else:
                print("Nao could not learn the object!")
                prediction_success = 0

            random_speech_id = random.randint(0, len(self.notification_speech_list[prediction_success]) - 1)
            speech_text = self.notification_speech_list[prediction_success][random_speech_id]
            if self.joy_level < 0.3 or self.joy_level > 0.7:
                random_exclamation_id = random.randint(0, len(self.emotion_exclamations[prediction_success]) - 1)
                speech_text = self.emotion_exclamations[prediction_success][
                                  random_exclamation_id] + " \\pau=100\\ " + speech_text
                if prediction_success == 1:
                    emotion_motion_file = "pride" + str(random.randint(0, 4))
                else:
                    emotion_motion_file = "shame" + str(random.randint(0, 5))

            names, times, keys = self.read_emotion_file(emotion_motion_file, self.joy_level)
            # TODO: check for motion-speech synchronization
            self.say_text(speech_text)
            self.motion_manager_proxy.angleInterpolation(names, keys, times, True)
            self.modulate_hope_parameter()

    def read_emotion_file(self, emotion_motion_file, emotion_level):
        funct_name = "robot.emotion.motions." + emotion_motion_file + "." + "save_to_joints"
        modn, funcn = funct_name.rsplit('.', 1)
        if modn not in sys.modules:
            # __import__(modn)
            importlib.import_module(modn)
        func = getattr(sys.modules[modn], funcn)
        return func(emotion_level)

    ############## AFFECTIVE FUNCTIONS #############
    def modulate_hope_parameter(self):
        self.motion_speed = self.hope_level  # [0, 1]
        self.motion_amplitude = self.hope_level  # [0, 1]
        self.speech_volume = int(self.hope_level * (80 - 30) + 30)  # [30, 80]
        self.speech_pitch = int(self.hope_level * (105 - 90) + 90)  # [90, 105]
        self.speech_speed = int(self.hope_level * (95 - 55) + 55)  # [55, 95]
        self.blinking_count = int((1 - self.hope_level) * (12 - 7) + 7)  # [7,12] it was 10

    def modulate_joy_parameter(self):
        self.motion_speed = self.joy_level  # [0, 1]
        self.motion_amplitude = self.joy_level  # [0, 1]
        self.speech_volume = int(self.joy_level * (80 - 30) + 30)  # [30, 80]
        self.speech_pitch = int(self.joy_level * (105 - 90) + 90)  # [90, 105]
        self.speech_speed = int(self.joy_level * (95 - 55) + 55)  # [55, 95]
        self.blinking_count = int((1 - self.joy_level) * (12 - 7) + 7)  # [7,12] it was 10

    def decide_emotion(self):
        self.hope_level, self.joy_level = self.emotion_fuzzy_system.compute_emotion(float(self.last_acc[0]))
        self.modulate_joy_parameter()

    ############## KILL FUNCTIONS ##############
    def restart(self):
        restart_pub = self.rospy.Publisher("restart", String)
        restart_pub.publish(String("true"))

    def killall(self):
        if self.session != "robot":
            return

        # self.stop_current_static_motion()
        # self.reset_current_static_motion()
        # sit_angles = [0.0,0.0,                     # head
        #     1.545, 0.33, -1.57, -0.486,            # left arm
        #     -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,         # left leg
        #     -0.3, 0.057, -0.744, 2.192, -1.122, -0.035,        # right leg
        #     1.545, -0.33, 1.57, 0.486]            # right arm
        # nao_names=["HeadYaw","HeadPitch","LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipYawPitch","RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll","RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"]
        # if(self.currentMotion  != 'leftKneeUpDown' and self.currentMotion != 'rightKneeUpDown'
        #    and self.currentMotion  != 'leftAnkleUpDown' and self.currentMotion  != 'leftAnkleUpDown'):
        #     self.motion_manager_proxy.angleInterpolation(nao_names, sit_angles, 3.5, True)
        #     time.sleep(3.5)

        self.goto_posture('Crouch', 3.0)
        time.sleep(1.0)
        self.release_stiffness()
        time.sleep(1.0)
        self.motion_manager_proxy = None
        self.posture_proxy = None
        self.speech_proxy = None
        if self.face_proxy:
            self.face_proxy.unsubscribe("Test_Face")
        self.face_proxy = None
        self.mem_proxy = None
        self.sr_proxy = None

        if self.driver:
            self.driver.close()


if __name__ == '__main__':
    robot = Robot(None, False)
    emotion_motion_file_names = ['pride0', 'pride1', 'pride2', 'pride3', 'pride4', 'shame0', 'shame1', 'shame2',
                                 'shame3', 'shame4', 'shame5']
    robot.goto_posture('Crouch', 1.0)
    time.sleep(1.0)
    for f_name in emotion_motion_file_names:
        names, times, keys = robot.read_emotion_file(f_name, 0.0)
        robot.motion_manager_proxy.angleInterpolation(names, keys, times, True)
