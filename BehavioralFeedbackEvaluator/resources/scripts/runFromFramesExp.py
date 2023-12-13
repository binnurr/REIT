import numpy
import cv2
import os
import glob
import csv
import argparse
import time
import datetime
import pytz

from FaceChannel.FaceChannelV1.imageProcessingUtil import imageProcessingUtil
from FaceChannel.FaceChannelV1.FaceChannelV1 import FaceChannelV1
from Utils import GUIController

faceChannelDim = FaceChannelV1("Dim", loadModel=True)

class FacialExpAnalyser:
    def __init__(self, subject_name, session_type, load_video_fp, result_fp, video_name):
        self.subject_name = subject_name
        self.session_type = session_type
        self.video_name = video_name
        #self.faceChannelCat = FaceChannelV1("Cat", loadModel=True)

        self.imageProcessing = imageProcessingUtil()
        self.faceSize = (64, 64)

        self.load_video_fp = load_video_fp

        self.result_fp = os.path.join(result_fp, self.subject_name, self.session_type)
        self.overall_result_fp = os.path.join(self.result_fp, "overall_results.csv")
        try:
            os.remove(self.overall_result_fp)
        except OSError:
            pass
        os.makedirs(os.path.dirname(self.overall_result_fp), exist_ok=True)

        self.GUIController = GUIController.GUIController()

    def get_lastest_file(self, path):
        list_of_files = glob.glob(os.path.join(path, '*.mkv'))
        latest_file = max(list_of_files, key=os.path.getmtime)
        return latest_file

    def process_request(self, question, start_time, end_time, time_relative=False):
        if self.video_name:
            video_name = self.video_name
        else:
            video_name = os.path.split(self.get_lastest_file(self.load_video_fp))[1]
        print(video_name)

        if not time_relative:
            video_ts = video_name.split('.')[0]
            video_time = datetime.datetime.strptime(video_ts, '%Y-%m-%d_%H-%M-%S')
            print(video_time)
            video_time_sec = datetime.timedelta(hours=video_time.hour, minutes=video_time.minute, seconds=video_time.second).\
                total_seconds()

            tz = pytz.timezone('Turkey')
            start_time_dt = datetime.datetime.fromtimestamp(start_time, tz=tz)

            start_time_sec = datetime.timedelta(hours=start_time_dt.hour, minutes=start_time_dt.minute, seconds=start_time_dt.second).\
                total_seconds()

            print(start_time_sec)
            print(video_time_sec)
            video_rel_start_time = start_time_sec - video_time_sec
        else:
            video_rel_start_time = start_time

        print("video_rel_start_time", video_rel_start_time)

        result_fp = os.path.join(self.result_fp, question + ".csv")
        try:
            os.remove(self.result_fp)
        except OSError:
            pass
        os.makedirs(os.path.dirname(result_fp), exist_ok=True)
        arousals = []
        valences = []
        with open(result_fp, mode="a") as employee_file:
            employee_writer = csv.writer(employee_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            employee_writer.writerow(['Frame', 'Arousal', 'Valence'])
            cap = cv2.VideoCapture(os.path.join(self.load_video_fp, video_name))  # open the video
            fps = cap.get(cv2.CAP_PROP_FPS)

            cap.set(cv2.CAP_PROP_POS_MSEC, video_rel_start_time * 1000)
            #fps = cap.get(cv2.CAP_PROP_FPS)
            #cap.set(cv2.CAP_PROP_POS_FRAMES, video_rel_start_time * fps)
            #cap.set(1, int(video_rel_start_time * fps))
            cap.read()
            total = int((end_time - start_time) * fps)
            print("total_frame", total)
            frame_count = 0
            valid_frame_count = 0
            arousal_sum = 0.0
            valence_sum = 0.0
            while cap.isOpened() and not frame_count == total:
                ret, frame = cap.read()
                #print("***", cap.get(cv2.CAP_PROP_POS_FRAMES))
                frame_count = frame_count + 1

                if frame_count % 4 != 0:
                    continue

                start_time = time.time()
                frame = cv2.resize(frame, (640, 480))
                cv2.imwrite(os.path.join(os.getcwd(), 'experiment/frames', str(question)+'_'+str(frame_count) + ".png"), frame)
                facePoints, face = self.imageProcessing.detectFace(frame)  # detect a face

                if face is not None:  # If a face is detected
                    face = self.imageProcessing.preProcess(face, imageSize=self.faceSize)  # pre-process the face
                    dimensionalRecognition = numpy.array(
                        faceChannelDim.predict(face, preprocess=False))  # Obtain dimensional classification
                    #categoricalRecognition = self.faceChannelCat.predict(face, preprocess=False)[0]
                    '''
                    frame = GUIController.createDimensionalEmotionGUI(dimensionalRecognition, frame, categoricalReport=[],
                                                                      categoricalDictionary=None)
    
                    arousals.append(dimensionalRecognition[0][0][0])
                    valences.append(dimensionalRecognition[1][0][0])
        
                    frame = GUIController.createDimensionalPlotGUI(arousals, valences, frame)
                    cv2.imwrite(saveNewFrames + "/" + frameName, frame)
                    '''
                    valid_frame_count = valid_frame_count + 1
                    arousal_sum = arousal_sum + dimensionalRecognition[0][0][0]
                    valence_sum = valence_sum + dimensionalRecognition[1][0][0]
                    arousals.append(dimensionalRecognition[0][0][0])
                    valences.append(dimensionalRecognition[1][0][0])
                else:  # if there is no face
                    dimensionalRecognition = [[[-99]], [[-99]]]

                employee_writer.writerow([frame_count, dimensionalRecognition[0][0][0], dimensionalRecognition[1][0][0]])
                #print("FPS: ", 1.0 / (time.time() - start_time))  # FPS = 1 / time to process loop

            arousal_mean, arousal_variance, valence_mean, valence_variance = 0, 0, 0, 0
            if valid_frame_count > 0:
                arousal_mean = arousal_sum/valid_frame_count
                valence_mean = valence_sum/valid_frame_count
                arousal_variance = sum([((x - arousal_mean) ** 2) for x in arousals]) / valid_frame_count
                arousal_variance = arousal_variance ** 0.5

                valence_variance = sum([((x - valence_mean) ** 2) for x in valences]) / valid_frame_count
                valence_variance = valence_variance ** 0.5
            else:
                print("NO VALID FRAME IN THIS INTERVAL!!!")
            with open(self.overall_result_fp, mode="a") as overall_res_file:
                overall_res_writer = csv.writer(overall_res_file, delimiter=',')
                overall_res_writer.writerow([question, arousal_mean, arousal_variance, valence_mean, valence_variance])
            cap.release()


if __name__ == "__main__":
    print("Program started")
    parser = argparse.ArgumentParser(description='Facial expression analysis.')
    parser.add_argument('--subject_name', type=str, help='subject name')
    parser.add_argument('--session', type=str, help='session type; tts or robot')

    args = parser.parse_args()
    subject_name = args.subject_name
    session_type = args.session

    load_video_fp = os.path.join(os.getcwd(), "experiment/videos_exp2")
    request_fp = os.path.join(os.getcwd(), "experiment/requests/exp2_post")
    result_fp = os.path.join(os.getcwd(), "experiment/results/exp2_post")

    offline = True
    time_relative = True

    if offline:
        subject_to_file_name = {
            "s01_exp1": "s01_exp1.mp4",
            "s02_exp1": "s02_exp1.mp4",
            "s03_exp1": "s03_exp1.mp4",
            "s04_exp1": "s04_exp1.mp4",
            "s05_exp1": "s05_exp1.mp4",
            "s06_exp1": "s06_exp1.mp4",
            "s07_exp1": "s07_exp1.mp4",
            "s08_exp1": "s08_exp1.mp4",
            "s09_exp1": "s09_exp1.mp4",
            "s10_exp1": "s10_exp1.mp4",
            "s11_exp1": "s11_exp1.mp4",
            "s12_exp1": "s12_exp1.mp4"
        }

        subject_to_file_name = {}
        for i in range(10, 10+1):
            subject_to_file_name["S"+str(i)] = "S" + f"{i:02}" + "_exp2.mkv"
    else:
        subject_to_file_name = {subject_name: None}

    for subject_name, video_name in subject_to_file_name.items():
        print("Running for subject ", subject_name)
        fea = FacialExpAnalyser(subject_name, session_type, load_video_fp, result_fp, video_name)

        #s_request_fp = os.path.join(request_fp, subject_name, session_type, "request.csv")
        s_request_fp = os.path.join(request_fp, subject_name + ".csv")
        while not os.path.exists(s_request_fp):
            time.sleep(1)

        request_f = open(s_request_fp, 'r')
        try:
            while True:
                request = request_f.readline()
                if not request:
                    if not offline:
                        time.sleep(1)
                        print('Nothing New', time.time())
                    else:
                        break
                else:
                    print('Run analysis for request: ', request)
                    [question, start_time, end_time] = request.split(',')
                    fea.process_request(question, float(start_time), float(end_time), time_relative=time_relative)
                    print('Analysis completed for request: ', request)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
