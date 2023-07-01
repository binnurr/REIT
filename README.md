# RoREIT (Robotic Requirements Elicitation Interview Trainer)
# VoREIT (Voice-based Requirements Elicitation Interview Trainer)


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