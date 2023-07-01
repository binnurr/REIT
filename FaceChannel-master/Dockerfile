FROM python:3.7.13-slim-buster

RUN apt-get update
RUN apt-get install ffmpeg libsm6 libxext6  -y

WORKDIR /usr/share/resources

COPY Requirements.txt ./
RUN pip install --no-cache-dir -r Requirements.txt
RUN pip install matplotlib
RUN pip install pytz
#RUN pip install facechannel

#COPY resources/examples ./examples
#COPY resources /usr/share/resources

ENV PYTHONPATH "${PYTHONPATH}:/usr/share/resources/examples"

CMD [ "python", "./examples/FaceChannelV1_WebCam.py" ]