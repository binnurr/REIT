def save_to_joints(emotion_level):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([0.076658, 0.15796, 0.392662, 0.474777, 0.240796, 0.00455999, 0.00455999])

    names.append("HeadYaw")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([-0.016916, -0.016916, -0.177985, -0.2102, -0.214803, -0.0828778, -0.07214])

    names.append("RElbowRoll")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([0.79312, 0.81613, 1.08305, 1.18582, 1.04776, 0.944986, 0.914306])

    names.append("RElbowYaw")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([0.322099, 0.424876, 0.492371, 0.490837, 0.450955, 0.176367, 0.0705221])

    names.append("RHand")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([0.0396, 0.0396, 0.0396, 0.0396, 0.0396, 0.0396, 0.0396])

    names.append("RShoulderPitch")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([1.01555, 0.0291878, -0.285283, -0.291418, -0.248467, 0.612108, 0.941918])

    names.append("RShoulderRoll")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([-0.066004, -0.032256, 0.121144, 0.0843279, 0.033706, -0.0353239, -0.01845])

    names.append("RWristYaw")
    times.append([0.2, 0.72, 1.2, 1.72, 2.2, 2.72, 3.2])
    keys.append([0.493905, 0.507713, 0.507713, 0.507713, 0.510779, 0.501576, 0.501576])

    return names, times, keys
