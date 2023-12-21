FROM ros:kinetic-ros-base-xenial

RUN apt-get update && apt-get install -y \
      iproute2 \
      telnet \
      iputils-ping \
      curl \
      libblas3 \
      liblapack3 \
      gcc \
      gfortran \
      libc6 \
      ffmpeg \
  #    ros-kinetic-naoqi-libqi \
  #    ros-kinetic-naoqi-libqicore \
  #    ros-kinetic-naoqi-bridge_msgs \
  #    ros-kinetic-naoqi-driver \
      ros-kinetic-rqt \
      ros-kinetic-rqt-common-plugins&& \
    rm -rf /var/lib/apt/lists/*

#WORKDIR /naoqi
#ADD naoqi-sdk-2.1.4.13-linux64.tar.gz /naoqi/
#ENV PYTHONPATH=${PYTHONPATH}:/naoqi/naoqi-sdk-2.1.4.13-linux64/lib/
#ENV LD_LIBRARY_PATH="/naoqi/naoqi-sdk-2.1.4.13-linux64:$LD_LIBRARY_PATH"


RUN curl "https://bootstrap.pypa.io/pip/2.7/get-pip.py" -o "get-pip.py"
RUN python get-pip.py
#RUN pip install --upgrade pip
RUN pip install selenium
#RUN pip install playsound
RUN pip install pydub
RUN pip install gtts
RUN pip install pyvis
#RUN sudo apt-get install python3-scipy
#RUN pip install scikit-fuzzy
RUN mkdir -p /home/catkin_ws/src

#ENV ROS_IP=10.0.1.23
#ENV ROS_MASTER_URI=http://10.0.1.23:11311
