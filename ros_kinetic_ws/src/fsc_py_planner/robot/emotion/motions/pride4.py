def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("LElbowRoll")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([-0.673468, -0.641253, -0.581429, -0.569155, -0.556884, -0.556884])

    names.append("LElbowYaw")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([-0.592082, -0.960242, -0.926494, -0.819114, -0.599753, -0.58748])

    names.append("LHand")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([0.2016, 0.2016, 0.2016, 0.2016, 0.2016, 0.2016])

    names.append("LShoulderPitch")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([0.802324, -1.09063, -1.11518, 0.234743, 1.06924, 1.14441])

    names.append("LShoulderRoll")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([-0.00916195, 0.392746, 0.437231, -0.0398422, -0.159494, -0.139552])

    names.append("LWristYaw")
    times.append([0.4, 0.92, 1.4, 1.92, 2.4, 2.92])
    keys.append([-0.383458, -0.381923, -0.383458, -0.383458, -0.384992, -0.384992])

    return names, times, keys
