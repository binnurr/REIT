#!/usr/bin/env python


import sys,os
import argparse
import roslib
roslib.load_manifest('fsc_py_planner')
import rospy
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('fsc_py_planner'))


# planner imports
from planner.fsc_planner import FSCPlanner

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--user', type=str,
                        help='user name')
    parser.add_argument('--agent', type=str,
                        help='agent type')
    parser.add_argument('--scenario', type=int,
                        help='scenario id')
    parser.add_argument('--behavioral_feedback', type=bool,
                        help='to activate behavioral feedback')

    args = parser.parse_args()
    user_name = args.user
    session_mode = args.agent
    scenario = args.scenario
    behavioral_feedback = args.behavioral_feedback
    
    if session_mode == "tts":
        from robot.robotTeachableExp_tts import Robot
    else:
        from robot.robotTeachableExp import Robot
    # init ros node
    rospy.init_node("py_planner", anonymous=True, xmlrpc_port=45100, tcpros_port=45101)
    behavior_name = rospy.get_param("~behavior_name", "RERobot")

    # create robot class to manage whole robotic stuff
    # such as handling naoqi connection and other ros nodes
    robot = Robot(rospy, session_mode, scenario, behavioral_feedback)
    rospy.on_shutdown(robot.restart)
    # create planner
    planner = FSCPlanner()
    planner.set_robot(robot)
    planner.load_fsc(behavior_name)
    
    # set loop frequency
    rate = rospy.Rate(10) #0.1Hz
    # control loop
    try:
        while not rospy.is_shutdown():
            planner.run()
            rate.sleep()
    except Exception as e:
        robot.killall()
        raise
    
    robot.killall()
    
    
