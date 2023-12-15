# this is the script to run motion files on the robot independent of the overall system.
from naoqi import ALProxy
import os
import sys
import importlib
import time

def read_emotion_file(emotion_motion_file, emotion_level):
    funct_name = emotion_motion_file +"." +"save_to_joints"
    modn, funcn = funct_name.rsplit('.', 1)
    if modn not in sys.modules:
        # __import__(modn)
        importlib.import_module(modn)
    func = getattr(sys.modules[modn], funcn)
    return func(emotion_level)

try:
    emotion_motion_file = "hope"
    joy_level = 0.9
    names, times, keys = read_emotion_file(emotion_motion_file, joy_level)
    motion = ALProxy("ALMotion", "10.0.1.3", 9559)
    posture = ALProxy("ALRobotPosture", "10.0.1.3", 9559)
    motion.setStiffnesses('Body', 0.9)
    time.sleep(0.5)
    posture.goToPosture("Crouch", 1.0)
    motion.angleInterpolation(names, keys, times, True)
    motion.setStiffnesses('Body', 0.0)
except BaseException as err:
    print(err)