def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("LElbowRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([-0.820649, -0.854396, -0.85593, -0.858999, -0.862065, -0.83292])

    names.append("LElbowYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([-0.842209, -0.856014, -1.16128, -1.27173, -0.983336, -0.849878])

    names.append("LHand")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.0132, 0.0132, 0.0132, 0.0132, 0.0132, 0.0132])

    names.append("LShoulderPitch")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([1.30386, 1.26704, 0.380391, 0.240796, 0.931096, 1.29772])

    names.append("LShoulderRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([-0.0261199, -0.0261199, 0.0705221, 0.095066, 0.0152981, 0.00609397])

    names.append("LWristYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([-0.636652, -0.636652, -0.648924, -0.648924, -0.648924, -0.648924])

    names.append("RElbowRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.83147, 0.865217, 0.87749, 0.87749, 0.917375, 0.898967])

    names.append("RElbowYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.771559, 0.786901, 1.15046, 1.20108, 0.926494, 0.719404])

    names.append("RHand")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.2016, 0.2016, 0.2016, 0.2016, 0.2016, 0.2016])

    names.append("RShoulderPitch")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([1.30394, 1.23491, 0.340591, 0.220938, 0.875956, 1.31775])

    names.append("RShoulderRoll")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.00609397, 0.00455999, -0.15651, -0.199461, -0.0767419, -0.0859461])

    names.append("RWristYaw")
    times.append([0.2, 0.68, 1.2, 1.68, 2.2, 2.68])
    keys.append([0.309826, 0.314428, 0.332836, 0.331302, 0.331302, 0.332836])

    return names, times, keys