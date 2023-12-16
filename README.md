# Exploring the REIT Architecture for Requirements Elicitation Interview Training with Robotic and Virtual Tutors
## Introduction
RoREIT: Robotic Requirements Elicitation Interview Trainer  
VoREIT: Voice-based Requirements Elicitation Interview Trainer  

We propose REIT, an extensible architecture for Requirements Elicitation Interview Trainer system based on emerging technologies for education. It has two separate phases. The first is the interview phase, where the student acts as an interviewer and the system as an interviewee. The second is the feedback phase, where the system evaluates the student's performance and provides contextual and behavioral feedback to enhance their interviewing skills. We demonstrate the applicability of REIT by implementing two instances: RoREIT with an embodied physical robotic agent and VoREIT with a virtual voice-only agent. This repository contains the code base for implementation of RoREIT and VoREIT.

## Architecture Overview
<img src="https://github.com/binnurr/REIT/assets/10512261/cf442485-b3a1-475b-98d8-b34b6213fe9b" width=85% height=85%>

## Structure of the Repository
### Behavioral Feedback Evaluator
The Behavioral Feedback Evaluator folder includes the source code of the Behavioral Feedback Evaluator and its core components.
### Ros Noetic Ws
The Ros Noetic Ws folder includes the source code of the stream recorder, dialogue displayer, and contextual feedback analyzer. It also includes the speech synthesizer component of the VoREIT agent.
### Ros Kinetic Ws
The Ros Kinetic Ws folder includes the source code of the interaction engine and scenario and feedback databases. It also includes trainer agent implementations.

## Installation
We use two different distributions of ROS: ROS Kinetic and ROS Noetic. ROS Kinetic is an older distribution but works with Python2, which is required to run the NAOqi SDK of the Nao robot. 

### Step 1:

Install miniforge which is a minimal installer for Conda and Mamba specific to conda-forge (conda package manager)
https://github.com/conda-forge/miniforge

Install Docker:
https://docs.docker.com/get-docker/

Install OBS Studio (required for video streaming of user's video in Zoom):
https://obsproject.com/download

Install ROS (Robot Operating System) 
..* For Linux: https://wiki.ros.org/noetic/Installation
..* For Mac: https://robostack.github.io/GettingStarted.html
 
Download and install NAOqi http://doc.aldebaran.com/2-8/dev/python/install_guide.html

### Step 2:
```sh
mamba activate ros_env
conda install pandas=2.1.4
conda install seaborn=0.13.0
conda install pyyaml=5.4.1
conda install pydub=0.25.1
conda install gtts=2.4.0
conda install pyqt=5.15.7
pip install obs-websocket-py
mamba install ros-noetic-rqt-ez-publisher
mamba deactivate
```

### Step 3:
```sh
conda create --name BehavioralFeedback python=3.10
conda activate BehavioralFeedback 
conda install h5py=3.9.0
conda install keras=2.12.0
conda install numpy=1.26.2
conda install opencv=4.7.0
conda install scipy=1.11.4
conda install tensorflow=2.12.1
conda install pytz=2023.3.post1
conda install matplotlib=3.8.2
pip install deffcode==0.2.5
```
```console
touch /Users/binnurgorer/miniforge3/envs/BehavioralFeedbackEval/lib/python3.10/site-packages/reit.pth
add your_working_directory/REIT/BehavioralFeedbackEvaluator/resources
conda deactivate
```

### Step 4: ROS Kinetic Environment
We use [Docker for ROS](https://hub.docker.com/_/ros) approach to create ROS Kinetic environment. Pull `ros:kinetic-ros-base-xenial` image. For more details, please refer to [here](https://hub.docker.com/layers/library/ros/kinetic-ros-base-xenial/images/sha256-a42bae4b8b66c2e256a047bf329f11730265a12a3ed29b10631f15591087112d).
```console
docker pull ros:kinetic-ros-base-xenial
```
```console
binnurgorer@Binnurs-MBP-Lab REIT % cd ros_kinetic_ws
binnurgorer@Binnurs-MBP-Lab ros_kinetic_ws % docker build -t ros_kinetic_demo_app .
```

## Execution of VoREIT
```console
```console
binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % mamba activate ros_env
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_HOSTNAME=Binnurs-MBP-Lab.home    
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % roscore
```
```

```console
binnurgorer@Binnurs-MBP-Lab ros_kinetic_ws % docker run -p 45100:45100 -p 45101:45101 -it --rm --mount type=bind,source="$(pwd)",target=/home/catkin_ws ros_kinetic_demo_app
root@075f362243b6:/# export ROS_HOSTNAME=Binnurs-MBP-Lab.home
root@075f362243b6:/# export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
root@075f362243b6:/# cd /home/catkin_ws/
root@075f362243b6:/home/catkin_ws# catkin_make
root@075f362243b6:/home/catkin_ws# cd devel/
root@075f362243b6:/home/catkin_ws/devel# source setup.bash 
root@075f362243b6:/home/catkin_ws/devel# cd ..
root@075f362243b6:/home/catkin_ws# rosrun fsc_py_planner main.py user tts 2 output="screen"
```

```console
binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % mamba activate ros_env
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_HOSTNAME=Binnurs-MBP-Lab.home    
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % catkin_make
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % cd devel 
(ros_env) binnurgorer@Binnurs-MBP-Lab devel % source setup.bash 
(ros_env) binnurgorer@Binnurs-MBP-Lab devel % cd ..
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % roslaunch obs_control re_exp.launch uname:=user session:=tts
```
## Execution of RoREIT
```console
conda activate BehavioralFeedback
(BehavioralFeedbackEval) binnurgorer@Binnurs-MacBook-Pro-Lab resources % python scripts/runFromFramesExp.py --subject_name=user --session=robot
```
```console
binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % mamba activate ros_env
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_HOSTNAME=Binnurs-MBP-Lab.home    
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % roscore
```

```console
binnurgorer@Binnurs-MBP-Lab ros_kinetic_ws % docker run -p 45100:45100 -p 45101:45101 -it --rm --mount type=bind,source="$(pwd)",target=/home/catkin_ws ros_kinetic_demo_app
root@075f362243b6:/# export ROS_HOSTNAME=Binnurs-MBP-Lab.home
root@075f362243b6:/# export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
root@075f362243b6:/# cd /home/catkin_ws/
root@075f362243b6:/home/catkin_ws# catkin_make
root@075f362243b6:/home/catkin_ws# cd devel/
root@075f362243b6:/home/catkin_ws/devel# source setup.bash 
root@075f362243b6:/home/catkin_ws/devel# cd ..
root@075f362243b6:/home/catkin_ws# rosrun fsc_py_planner main.py user robot 2 output="screen"
```

```console
binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % mamba activate ros_env
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_HOSTNAME=Binnurs-MBP-Lab.home    
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % catkin_make
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % cd devel 
(ros_env) binnurgorer@Binnurs-MBP-Lab devel % source setup.bash 
(ros_env) binnurgorer@Binnurs-MBP-Lab devel % cd ..
(ros_env) binnurgorer@Binnurs-MBP-Lab ros_noetic_ws % roslaunch obs_control re_exp.launch uname:=user session:=robot
```

## Roadmap
### Implementation Design Decisions and Further Developments
The Naoqi Python SDK contains native libraries compiled for Linux and amd64 (aka Intel x64) processors. The M1 Macbook has an arm64 processor.
The executable library `_qi.so` is reported as non-existent because it cannot execute it. 
The Naoqi docker solution mentioned [here](https://github.com/remcorakers/naoqi-docker) does not work
with our Kinetic docker image executed on the M1 processor. If your architecture is amd64, adding the Naoqi installations to the Dockerfile for the Kinetic environment
would work. An alternative approach would use ROS Naoqi driver (https://index.ros.org/p/naoqi_driver/) to access the Naoqi API of the robot. We will try it out soon.
For the current implementation, we use a separate Native Linux on the amd64 processor to execute RoREIT's robotic agent module and control the Nao robot.

## License 

All the examples in this repository are distributed under a Non-Commercial license. If you use this environment, you have to agree with the following items:

- To cite our associated references in any of your publications that use these examples.

- To use the environment for research purposes only.

- To not provide the environment to any second parties.

## Contact

Binnur Gorer - binnur.gorer@boun.edu.tr

## Acknowledgements

* Solution to get ROS messages from ROS Master running on the host from ROS on Docker as the slave node as ```--net=host``` option does not work in Mac: https://medium.com/@yasuhirachiba/specifying-port-to-be-used-by-ros1-node-bd9dfd8643c6
* Facial expression analyzer library: https://github.com/pablovin/FaceChannel/
