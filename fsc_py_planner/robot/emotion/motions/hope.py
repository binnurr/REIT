# TODO: check if head motion should be overwritten by tracking


def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    arousal = 2 * emotion_level - 1  # arousal between [-1,1]
    valence = 0.6 * emotion_level - 0.3  # valence between [-0.3, 0.3]

    # times [0.5, 2, 4]
    interval1 = 2 - emotion_level
    interval2 = 3 - 2 * emotion_level

    head_pitch = arousal / 2 * -1

    names.append("HeadPitch")
    times.append([0.5, 0.5 + interval1, 0.5 + interval1 + interval2])
    keys.append([0.0, head_pitch, 0.0])

    shoulder_pitch = 1.4 - valence * 0.5

    names.append("LShoulderPitch")
    times.append([0.5, 0.5 + interval1, 0.5 + interval1 + interval2])
    keys.append([1.45726, shoulder_pitch, 1.45726])

    names.append("RShoulderPitch")
    times.append([0.5, 0.5 + interval1, 0.5 + interval1 + interval2])
    keys.append([1.4, shoulder_pitch, 1.4])

    shoulder_roll = valence * 0.8

    names.append("LShoulderRoll")
    times.append([0.5, 0.5 + interval1, 0.5 + interval1 + interval2])
    keys.append([0.5, -shoulder_roll, 0.3])

    names.append("RShoulderRoll")
    times.append([0.5, 0.5 + interval1, 0.5 + interval1 + interval2])
    keys.append([-0.5, shoulder_roll, -0.3])

    return names, times, keys
