# Exploring the REIT Architecture for Requirements Elicitation Interview Training with Robotic and Virtual Tutors
## Introduction
RoREIT: Robotic Requirements Elicitation Interview Trainer  
VoREIT: Voice-based Requirements Elicitation Interview Trainer  

We propose REIT, an extensible architecture for Requirements Elicitation Interview Trainer system based on emerging technologies for education. It has two separate phases. The first is the interview phase, where the student acts as an interviewer and the system as an interviewee. The second is the feedback phase, where the system evaluates the student's performance and provides contextual and behavioral feedback to enhance their interviewing skills. We demonstrate the applicability of REIT by implementing two instances: RoREIT with an embodied physical robotic agent and VoREIT with a virtual voice-only agent. This repository contains the code base for the implementation of RoREIT and VoREIT.

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
Install miniforge which is a minimal installer for Conda and Mamba specific to conda-forge (conda package manager):
- https://github.com/conda-forge/miniforge

Install Docker:
- https://docs.docker.com/get-docker/

Install [OBS Studio](https://obsproject.com/) (required for video streaming of user's video in Zoom):
- https://github.com/obsproject/obs-studio/releases/tag/28.0.0
> [!NOTE]  
> OBS v28.0.0 comes with obs-websocket v5, which is incompatible with ```obs-websocket-py``` library we use in our system. We need to separately install version 4 of the WebSocket plugin from https://github.com/obsproject/obs-websocket/releases/tag/4.9.1-compat 

Install ROS (Robot Operating System):

We employ two ROS versions: ROS Kinetic and ROS Noetic. The NAO robotic agent relies on NAOqi, which is compatible only with Python version 2. Consequently, the agent module operates within the ROS Kinetic environment, specifically designed to support Python version 2. We use ROS Noetic (based on Python v3) for the other modules implemented in Python v3. For the installation of ROS Noetic, please refer to https://robostack.github.io/GettingStarted.html. For ROS Noetic, we must use [native installation](https://wiki.ros.org/noetic/Installation) on a Linux environment to [install the NAOqi](http://doc.aldebaran.com/2-8/dev/python/install_guide.html) required to connect the Nao robot (for RoREIT). We can execute ROS Kinetic in a Docker environment which can  
> [!NOTE]
>  Use Python version 3.9.18 while creating the Python virtual environment in the Ros Noetic installation from Robostack.
>
> ```sh mamba create -n ros_env python=3.9.18```

> [!NOTE]
> We use NAOqi version ```naoqi-sdk-2.1.4.13```

### Step 2: Install Repo
You can just navigate to your working path and clone the repo.
```sh
clone git@github.com:binnurr/REIT.git
cd REIT
pwd
```
Save this path; we will set this path as ```$REIT_HOME```below. 

### Step 3: Behavioral Feedback Module Environment
Setup the Python virtual environment

```sh
conda create --name BehavioralFeedback python=3.8
conda activate BehavioralFeedback
conda install -c apple tensorflow-deps
pip install tensorflow-macos == 2.8.0
pip install tensorflow-metal == 0.4.0
pip install opencv-python
pip install scipy
pip install pytz
pip install matplotlib
pip install deffcode
```

> [!IMPORTANT]  
> For M1 Mac users, execute the above commands in a terminal that is started by enabling Rosetta.

> [!WARNING]
> Most recent version of ```tensorflow-metal``` produces incorrect results with the preloaded models on M1 Mac (check the reported issues [here]()). Please be careful with the versions.
> To check the compatible ```tensorflow-metal``` version, please refer to https://pypi.org/project/tensorflow-metal/

Add the path ```($REIT_HOME)/BehavioralFeedbackEvaluator/resources``` to the BehavioralFeedback virtual environment.
Create a file like ```.../miniforge3/envs/BehavioralFeedbackEval/lib/python3.8/site-packages/reit.pth``` and add the path inside.
Then, reactive the environment to have the changes applied.


### Step 4: ROS Noetic Environment
Install the required packages into the ROS Noetic environment
```sh
mamba activate ros_env
conda install pandas=2.1.4
conda install seaborn=0.13.0
conda install pyyaml=5.4.1
conda install pydub=0.25.1
conda install gtts=2.4.0
conda install pyqt=5.15.7
pip install obs-websocket-py==0.5.3
mamba install ros-noetic-rqt-ez-publisher
mamba deactivate
```

### Step 5: ROS Kinetic Environment
We use [Docker for ROS](https://hub.docker.com/_/ros) approach to create a ROS Kinetic environment. Pull the image as per the command shown in Docker Hub Pull. For more details, please refer to [here](https://hub.docker.com/layers/library/ros/kinetic-ros-base-xenial/images/sha256-a42bae4b8b66c2e256a047bf329f11730265a12a3ed29b10631f15591087112d).
```sh
docker pull ros:kinetic-ros-base-xenial
```
Then, build the Docker image provided in the ```ros_kinetic_ws``` folder.
```sh
cd ($REIT_HOME)/ros_kinetic_ws
docker build -t ros_kinetic_demo_app .
```

### Step 6: OBS Configuration:
* For the obs-websocket document, please refer to https://github.com/obsproject/obs-websocket/blob/4.x-compat/docs/generated/protocol.md
* Set the output recording path as below:
```sh
($REIT_HOME)/BehavioralFeedbackEvaluator/resources/experiment/videos
```
* Set the values for Base Resolution, Output Resolution, and Common FPS values as shown in the figure.
![obs_config](https://github.com/binnurr/REIT/assets/10512261/48cd2ed2-28b2-4832-8325-5171ccdbb194)
* Ensure Server port and password (can be accessed via OBS->Tools->Web server settings) are the same as defined in
  https://github.com/binnurr/REIT/blob/58de3c65b3b6016ba063ea0dd86bd7d2ec042e79/ros_noetic_ws/src/obs_control/scripts/obs_control.py#L13-L14
  
## Execution of REIT
> [!IMPORTANT]
> First of all, start Docker Deamon and OBS Studio.

You need to initiate the Behavioral Feedback module to use the Behavioral Feedback feature. 
```sh
export REIT_HOME = your_REIT_home_directory
cd ($REIT_HOME)/BehavioralFeedbackEvaluator/resources
conda activate BehavioralFeedback
python scripts/runFromFramesExp.py --user=user --agent=tts
```

Use ROS Noetic environment to start roscore. 
```sh
mamba activate ros_env
hostname  % get your hostname
export ROS_HOSTNAME=your_hostname   
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
roscore
```

execute ROS Kinetic environment
* for text to speech agent
```sh
export REIT_HOME = your_REIT_home_directory
cd ($REIT_HOME)/ros_kinetic_ws
docker run -p 45100:45100 -p 45101:45101 -it --rm --mount type=bind,source="$(pwd)",target=/home/catkin_ws ros_kinetic_demo_app
[inside docker] export ROS_HOSTNAME=your_hostname
[inside docker] export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
[inside docker] cd /home/catkin_ws/
[inside docker] catkin_make
[inside docker] source devel/setup.bash
[inside docker] rosrun fsc_py_planner main.py --user user --agent tts --scenario 2 --behavioral_feedback True
```
* for robotic agent
  

execute ROS Noetic environment
```sh
export REIT_HOME = your_REIT_home_directory
cd ($REIT_HOME)/ros_noetic_ws
mamba activate ros_env
export ROS_HOSTNAME=your_hostname    
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
catkin_make
cd devel 
source setup.bash 
cd ..
roslaunch obs_control re_exp.launch user:=user agent:=tts scenario:=2 behavioral_feedback:=True
```
### VoREIT Configuration:
VoREIT uses text-to-speech agent, which can be set using "tts" for the agent argument. Moreover, by design, we do not use video modality in VoREIT, and the behavioral_feedback argument is set to False.

### RoREIT Configuration:
RoREIT uses the embodied robotic agent Nao, which can be set using "robot" for the agent argument. We use video modality in RoREIT, and the behavioral_feedback argument is set to True.

## Automatization of REIT execution
The execution steps of REIT can be automated by scripting. We share the apple scripts in the repo to execute [VoREIT]() and [RoREIT](https://github.com/binnurr/REIT/blob/4d679f1589d8dd17021619cfb42a10e85415b63f/roreit_run.scpt) in the repo. You can modify the script for your environment.

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
* Using ROS in a Docker container
  - https://medium.com/@vikramsetty169/using-ros-in-a-docker-container-862bcbd9d4bf
* Solution to get ROS messages from ROS Master running on the host from ROS on Docker as the slave node as ```--net=host``` option does not work in Mac
  - https://medium.com/@yasuhirachiba/specifying-port-to-be-used-by-ros1-node-bd9dfd8643c6
* Installation of Tensorflow on M1 Mac:
  - https://caffeinedev.medium.com/how-to-install-tensorflow-on-m1-mac-8e9b91d93706
  - https://stackoverflow.com/questions/72964800/what-is-the-proper-way-to-install-tensorflow-on-apple-m1-in-2022
* Facial expression analyzer library
  - https://github.com/pablovin/FaceChannel/
 
<!---
| :exclamation:  This is very important   |
|-----------------------------------------|

| :zap:        Ignore at your own risk!   |
|-----------------------------------------|

| :memo:        | Take note of this       |
|---------------|:------------------------|

| :point_up:    | Remember to not forget! |
|---------------|:------------------------|

| :warning: WARNING          |
|:---------------------------|
| I should warn you ...      |

| :boom: DANGER              |
|:---------------------------|
| Will explode when clicked! |  
-->
