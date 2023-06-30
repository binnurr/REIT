#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from std_msgs.msg import String
from obswebsocket import obsws, requests  # noqa: E402


class ObsControl:
    def __init__(self, subject_name, session_type):
        self.host = "localhost"
        self.port = 4444
        self.password = "rerobot123"
        self.ws = obsws(self.host, self.port, self.password)
        self.ws.connect()
        self.source_settings = {
                'owner_name': 'zoom.us',
                'window_name': 'zoom floating video window'
            }
        rospy.on_shutdown(self.shutdown_obs)
        self.start_recording_sub = rospy.Subscriber("is_start", String, self.start_recording)
        self.stop_recording_sub = rospy.Subscriber("stop_obs_recording", String, self.stop_recording)

    def start_recording(self, msg):
        self.ws.call(requests.CreateSource(sourceName='new_zoom',
                                      sourceKind='window_capture',
                                      sceneName='my_scene',
                                      sourceSettings=self.source_settings,
                                      setVisible=True))
        scene_item_prop = self.ws.call(requests.GetSceneItemProperties(item='new_zoom', scene_name='my_scene'))
        bounds = scene_item_prop.getBounds()
        bounds['type'] = 'OBS_BOUNDS_SCALE_INNER'
        bounds['x'] = 1280
        bounds['y'] = 720
        self.ws.call(requests.SetSceneItemProperties(item='new_zoom', scene_name='my_scene', bounds=bounds))
        self.ws.call(requests.StartRecording())

    def stop_recording(self, msg):
        self.ws.call(requests.StopRecording())

    def shutdown_obs(self):
        try:
            self.ws.call(requests.StopRecording())
        finally:
            self.ws.disconnect()


if __name__ == '__main__':
    rospy.init_node('obs_control', anonymous=True)
    args = rospy.myargv(argv=sys.argv)
    subject_name = args[1]
    session_type = args[2]
    obs_control = ObsControl(subject_name, session_type)
    rospy.spin()
