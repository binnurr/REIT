cmake_minimum_required(VERSION 2.4.6)
project(fsc_py_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  convert_positions
)


catkin_python_setup()


add_service_files(
  FILES
  GoogleSTT.srv
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GUItext.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
 CATKIN_DEPENDS message_runtime convert_positions
)
install(PROGRAMS
   main/main.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

