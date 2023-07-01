# RoIT4RE-VoIT4RE
RoIT4RE-VoIT4RE is created for


```console
foo@ros-ws % roslaunch obs_control re_exp.launch uname:=user session:=robot
foo@FaceChannel-master % docker run -it --rm  --name my-facechannel-app --mount type=bind,source="$(pwd)"/resources,target=/usr/share/resources -p 8080:80 facechannel_app python examples/runFromFramesExp.py --subject_name=user --session=robot
```
