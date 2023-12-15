def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("LElbowRoll")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([-0.757754, -0.757754, -0.750085, -0.762356, -0.731677, -0.713267])

    names.append("LElbowYaw")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([-0.725624, -0.983336, -1.11833, -0.944986, -0.70875, -0.702614])

    names.append("LHand")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.0132, 0.0132, 0.0132, 0.0132, 0.0132, 0.0132])

    names.append("LShoulderPitch")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([1.15046, -0.30224, -0.707216, 0.351244, 1.08143, 1.17347])

    names.append("LShoulderRoll")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([-0.00464392, 0.194775, 0.23466, 0.0628521, -0.021518, -0.0261199])

    names.append("LWristYaw")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([-0.648924, -0.648924, -0.648924, -0.648924, -0.650458, -0.648924])

    names.append("RElbowRoll")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.61671, 0.618244, 0.619779, 0.601371, 0.604439, 0.602905])

    names.append("RElbowYaw")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.658045, 0.816046, 0.819114, 0.74088, 0.612024, 0.605888])

    names.append("RHand")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.2016, 0.2016, 0.2016, 0.2016, 0.2016, 0.2016])

    names.append("RShoulderPitch")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([1.08918, -0.384992, -0.806841, 0.190258, 1.04163, 1.15361])

    names.append("RShoulderRoll")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.0275701, -0.107422, -0.170316, -0.038392, 0.0291041, 0.0291041])

    names.append("RWristYaw")
    times.append([0.28, 0.8, 1.28, 1.8, 2.28, 2.8])
    keys.append([0.331302, 0.332836, 0.331302, 0.332836, 0.332836, 0.332836])

    return names, times, keys
