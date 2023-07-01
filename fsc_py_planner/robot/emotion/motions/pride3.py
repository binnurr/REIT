def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("RElbowRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([0.671934, 0.673468, 0.641253, 0.581429, 0.569155, 0.556884, 0.556884])

    names.append("RElbowYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([0.561403, 0.592082, 0.960242, 0.926494, 0.819114, 0.599753, 0.58748])

    names.append("RHand")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([0.2016, 0.2016, 0.2016, 0.2016, 0.2016, 0.2016, 0.2016])

    names.append("RShoulderPitch")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([1.20883, 0.802324, -1.09063, -1.11518, 0.234743, 1.06924, 1.14441])

    names.append("RShoulderRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([0.0168321, 0.00916195, -0.392746, -0.437231, 0.0398422, 0.159494, 0.139552])

    names.append("RWristYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68, 3.2])
    keys.append([0.381923, 0.383458, 0.381923, 0.383458, 0.383458, 0.384992, 0.384992])

    return names, times, keys
