import random
import math


def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    if emotion_level > 0.5:
        # normalize [0.5, 1] to headpitch[0, -24] & headyaw[-20, 20]
        head_pitch = ((emotion_level-0.5)/0.5)*-24
        head_yaw = ((emotion_level-0.5)/0.5)*20
        if random.random() > 0.5:
            head_yaw = head_yaw*-1
    else:
        # normalize [0, 0.5] to headpitch[0, 25] & headyaw[-9, 9]
        head_pitch = ((0.5-emotion_level)/0.5)*25
        head_yaw = ((0.5-emotion_level)/0.5)*20
        if random.random() > 0.5:
            head_yaw = head_yaw*-1

    head_pitch = math.radians(head_pitch)
    head_yaw = math.radians(head_yaw)

    #TODO: time can be parametric according to the emotion_level
    names.append("HeadPitch")
    times.append([0.5, 2, 4])
    keys.append([0.0, head_pitch, 0.0])

    names.append("HeadYaw")
    times.append([0.5, 2, 4])
    keys.append([0.0, head_yaw, 0.0])

    return names, times, keys

