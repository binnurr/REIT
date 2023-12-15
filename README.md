# Exploring the REIT Architecture for Requirements Elicitation Interview Training with Robotic and Virtual Tutors
## Introduction
RoREIT: Robotic Requirements Elicitation Interview Trainer  
VoREIT: Voice-based Requirements Elicitation Interview Trainer  

We propose REIT, an extensible architecture for Requirements Elicitation Interview Trainer system based on emerging technologies for education. It has two separate phases. The first is the interview phase, where the student acts as an interviewer and the system as an interviewee. The second is the feedback phase, where the system evaluates the student's performance and provides contextual and behavioral feedback to enhance their interviewing skills. We demonstrate the applicability of REIT by implementing two instances: RoREIT with an embodied physical robotic agent and VoREIT with a virtual voice-only agent. This repository contains the code base for implementation of RoREIT and VoREIT.

## Architecture Overview


<img src="https://github.com/binnurr/REIT/blob/main/architecture/IT4RE.pdf" width="800" />

## Structure of the Repository


## Installation

### Step 1:

### Step 2:




```console
foo@ros-ws % roslaunch fsc_py_planner run_exp.launch uname:=user session:=robot scenario:=2
```

```console
foo@ros-ws % roslaunch obs_control re_exp.launch uname:=user session:=robot
foo@FaceChannel-master % docker run -it --rm  --name my-facechannel-app --mount type=bind,source="$(pwd)"/resources,target=/usr/share/resources -p 8080:80 facechannel_app python examples/runFromFramesExp.py --subject_name=user --session=robot
```

Download and install naoqi http://doc.aldebaran.com/2-8/dev/python/install_guide.html

**License**

All the examples in this repository are distributed under a Non-Commercial license. If you use this environment, you have to agree with the following items:

- To cite our associated references in any of your publication that make any use of these examples.

- To use the environment for research purpose only.

- To not provide the environment to any second parties.



**Contact**

Binnur Gorer - binnur.gorer@boun.edu.tr