import time
import h5py
import numpy as np
from inspire_hand import InspireHand


# inspire hand port name
left_hand_port = 'COM12'
right_hand_port = 'COM13'

# h5 file path
H5_FILE_PATH = './h5_data/2021-10-14/yumi/inference.h5'


"""
parse h5 file to joint angles
"""
def parse_h5(filename, group_name='group1'):
    f = h5py.File(filename, 'r')
    l_joint_angles = f[group_name+'/l_joint_angle'][:]
    r_joint_angles = f[group_name+'/r_joint_angle'][:]
    l_glove_angles = f[group_name+'/l_glove_angle'][:]
    r_glove_angles = f[group_name+'/r_glove_angle'][:]
    return l_joint_angles, r_joint_angles, l_glove_angles, r_glove_angles

"""
convert rad to degree
"""
def rad2degree(angle):
    return angle * 180.0 / np.pi

"""
convert glove angles to electrical signal
"""
def convert_glove_angles(glove_angles):
    ndata = glove_angles.shape[0]
    elec_signal = np.zeros((ndata, 6), dtype="int32")
    optim_fourfin_range = [0, -1.6-1.7]
    optim_thumbroll_range = [0.0, 0.4+0.4+1.0]
    optim_thumbrot_range = [0.3, -1.0]
    max_elec = 2000
    min_elec = 0

    # conversion
    for i in range(ndata):
        # four fingers (for inspire hand electrical signal, order is: pinky->ring->middle->index)
        elec_signal[i, 3] = round((glove_angles[i, 0] + glove_angles[i, 1] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 2] = round((glove_angles[i, 2] + glove_angles[i, 3] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 1] = round((glove_angles[i, 4] + glove_angles[i, 5] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 0] = round((glove_angles[i, 6] + glove_angles[i, 7] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        # thumb roll
        elec_signal[i, 4] = round((glove_angles[i, 9] - glove_angles[i, 10] - glove_angles[i, 11] - optim_thumbroll_range[0]) / (optim_thumbroll_range[1] - optim_thumbroll_range[0]) * (max_elec - min_elec))
        # thumb rot
        elec_signal[i, 5] = round((glove_angles[i, 8] - optim_thumbrot_range[0]) / (optim_thumbrot_range[1] - optim_thumbrot_range[0]) * (max_elec - min_elec))
        
    return elec_signal


if __name__ == '__main__':
    # inspire hands
    left_hand_controller = InspireHand(left_hand_port, 115200)
    right_hand_controller = InspireHand(right_hand_port, 115200)
    left_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)
    right_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)

    # parse h5 file
    l_joint_angles, r_joint_angles, l_glove_angles, r_glove_angles = parse_h5(H5_FILE_PATH)
    l_glove_signal = convert_glove_angles(l_glove_angles)
    r_glove_signal = convert_glove_angles(r_glove_angles)
    total_frames = len(l_joint_angles)
    print('total frame', total_frames)
    t = 0
    last_hand_command_time = -float('Inf')

    while True:
        # set inspire hand pos
        if time.time() - last_hand_command_time > 0.05:
            left_hand_controller.setpos(l_glove_signal[t, 0], l_glove_signal[t, 1], l_glove_signal[t, 2], l_glove_signal[t, 3], l_glove_signal[t, 4], l_glove_signal[t, 5])
            right_hand_controller.setpos(r_glove_signal[t, 0], r_glove_signal[t, 1], r_glove_signal[t, 2], r_glove_signal[t, 3], r_glove_signal[t, 4], r_glove_signal[t, 5])
            last_hand_command_time = time.time()
            # next frame
            t += 1
            # reset all for new loop
            if t == total_frames:
                t = 0
