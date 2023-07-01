#!/usr/bin/env python


# ros imports1
import sys,os
import roslib
roslib.load_manifest('fsc_py_planner')
import rospy
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('fsc_py_planner'))

#sys.path.append(os.path.abspath('src/fsc_py_planner'))

# import for robot
#from robot.robot import Robot



# planner imports
from planner.fsc_planner import FSCPlanner

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    session_mode = args[1]
    scenario = args[2]
    if session_mode == "tts":
        from robot.robotTeachableExp_tts import Robot
    else:
        from robot.robotTeachableExp import Robot
    # init ros node
    rospy.init_node("py_planner", anonymous=True)
    behavior_name = rospy.get_param("~behavior_name", "RERobot")

    # create robot class to manage whole robotic stuff
    # such as handling naoqi connection and other ros nodes
    robot = Robot(rospy, session_mode, scenario)
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
    
    
