import cv2
import numpy as np
import math
import os, sys
sys.path.append('../..')

heatmap_width = 92
heatmap_height = 92

"""
Joints Explained
14 joints:
0-right shoulder, 1-right elbow, 2-right wrist, 3-left shoulder, 4-left elbow, 5-left wrist, 
6-right hip, 7-right knee, 8-right ankle, 9-left hip, 10-left knee, 11-left ankle, 
12-top of the head and 13-neck

                     12                     
                     |
                     |
               0-----13-----3
              /     / \      \
             1     /   \      4
            /     /     \      \
           2     6       9      5
                 |       |
                 7       10
                 |       |
                 8       11
"""

JOINT_LIMB = [[0, 1], [1, 2], [3, 4], [4, 5], [6, 7], [7, 8], [9, 10], [10, 11], [12, 13], [13, 0], [13, 3], [13, 6], [13, 9]]
COLOR = [[0, 255, 255], [0, 255, 255],[0, 255, 255],[0, 255, 255],[0, 255, 0],[0, 255, 0],[0, 255, 0],[0, 255, 0], [0, 0, 255], [255, 0, 0],[255, 0, 0],[255, 0, 0], [255, 0, 0]]

def decode_body_pose(heatmaps, scale, image_original):

    # obtain joint list from heatmap
    # joint_list: a python list of joints, joint_list[i] is an numpy array with the (x,y) coordinates of the i'th joint (refer to the 'Joints Explained' in this file, e.g., 0th joint is right shoulder)  
    joint_list = [peak_index_to_coords(heatmap)*scale for heatmap in heatmaps]
    command = get_rc_command(joint_list, int(image_original.shape[1]))
    print(command)

    # plot the pose on original image
    canvas = image_original

    # # Don't bother drawing if no hand gesture is detected/hand gesture doesn't pass validation
    if command == "STOP":
        return canvas, command

    for idx, limb in enumerate(JOINT_LIMB):
        joint_from, joint_to = joint_list[limb[0]], joint_list[limb[1]]
        canvas = cv2.line(canvas, tuple(joint_from.astype(int)), tuple(joint_to.astype(int)), color=COLOR[idx], thickness=4)
    return canvas, command 


def peak_index_to_coords(peak_index):
    '''
    @peak_index is the index of max value in flatten heatmap
    This function convert it back to the coordinates of the original heatmap 
    '''
    peak_coords = np.unravel_index(int(peak_index),(heatmap_height, heatmap_width))
    return np.flip(peak_coords)


def get_rc_command(joint_list, width):

    x_arr = []
    y_arr = []

    for joint in joint_list:
        x_arr.append(joint[0])
        y_arr.append(joint[1])
    

    # left_arm_up = left_arm_status(x_arr, y_arr)
    # right_arm_up = right_arm_status(x_arr, y_arr)


    # if validate(x_arr, y_arr, width) == False:
    #     return "STOP"

    # elif (left_arm_up and right_arm_up) :
    #     return "ACTIVATE"

    # elif (left_arm_up and (not right_arm_up)) :
    #     return "FOLLOW"

    # elif ((not left_arm_up) and right_arm_up) :
    #     return "TAKE A PICTURE"

    # elif ((not left_arm_up) and (not right_arm_up)) :
    #     return "DEACTIVATE"

    # else:
    #     return "HAND!"


    bent_thres_min = 80
    bent_thres_max = 100
    straight_thres_min = 170
    straight_thre_max = 190

    straight = 180
    bent = 90
    threshold = 10

    left_elbow_up = left_elbow_status(y_arr)
    left_arm_angle = left_arm_bent(x_arr, y_arr)
    left_wrist_up = left_wrist_status(y_arr)

    right_elbow_up = right_elbow_status(y_arr)
    right_arm_angle = right_arm_bent(x_arr, y_arr)
    right_wrist_up = right_wrist_status(y_arr)

    print (left_arm_angle)


    if validate(x_arr, y_arr, width) == False or (not left_elbow_up) or (not right_elbow_up):
        return "STOP"

    # T pose
    elif abs(left_arm_angle - straight) <= threshold and abs(right_arm_angle - straight) <= threshold:
        return "TAKE A PICTURE"
    
    # fork 
    elif abs(left_arm_angle - bent) <= threshold and left_wrist_up and abs(right_arm_angle - bent) <= threshold and right_wrist_up:
        return "ACTIVATE"

    # angry
    elif abs(left_arm_angle - bent) <= threshold and (not left_wrist_up) and abs(right_arm_angle - bent) <= threshold and (not right_wrist_up):
        return "DEACTIVATE"

    # |_o_
    #     |
    elif abs(left_arm_angle - bent) <= threshold and (not left_wrist_up) and abs(right_arm_angle - bent) <= threshold and right_wrist_up:
        return "FOLLOW"

    #  _o_|
    # |
    elif abs(left_arm_angle - bent) <= threshold and left_wrist_up and abs(right_arm_angle - bent) <= threshold and (not right_wrist_up):
        return "STOP FOLLOW"
    
    # |_o__
    elif abs(left_arm_angle - straight) <= threshold and abs(right_arm_angle - bent) <= threshold and right_wrist_up:
        return "FORWARDS"

    # __o_|
    elif abs(left_arm_angle - bent) <= threshold and left_wrist_up and abs(right_arm_angle - straight) <= threshold:
        return "BACKWARDS"

    #  _o__
    # |
    elif abs(left_arm_angle - straight) <= threshold and abs(right_arm_angle - bent) <= threshold and (not right_wrist_up):
        return "SPIN LEFT"
    
    # __o_
    #     |
    elif abs(left_arm_angle - bent) <= threshold and (not left_wrist_up) and abs(right_arm_angle - straight) <= threshold:
        return "SPIN RIGHT"

    else:
        return "ARMS!"


def validate(x_arr, y_arr, width):

    x_max_threshold = int((1200 / 1280) * width)
    x_min_threshold = int((50 / 1280) * width)
    
    if ((max(x_arr) > x_max_threshold) or (min(x_arr) < x_min_threshold)):
          return False
    elif (y_arr.index(min(y_arr)) != 8) and (y_arr.index(min(y_arr)) != 12) and (y_arr.index(min(y_arr)) != 16) and (y_arr.index(min(y_arr)) != 20):
          return False
    else: 
          return True


def left_elbow_status(y_arr):

    horiZon_threshold = 50

    left_elbow_y = y_arr[4]
    body_y = y_arr[13]

    print(abs(left_elbow_y - body_y))

    # if left elbow horiZontal, then return 1
    if abs(left_elbow_y - body_y) <= horiZon_threshold:
        return 1
    else:
        return 0


def right_elbow_status(y_arr):

    horiZon_threshold = 50

    right_elbow_y = y_arr[4]
    body_y = y_arr[13]

    # if right elbow horiZontal, then return 1
    if abs(right_elbow_y - body_y) <= horiZon_threshold:
        return 1
    else:
        return 0


def left_arm_bent(x_arr, y_arr):

    # get vectors of left arms
    left_elbow_x = x_arr[3] - x_arr[4]
    left_elbow_y = y_arr[3] - y_arr[4]
    left_wrist_x = x_arr[5] - x_arr[4]
    left_wrist_y = y_arr[5] - y_arr[4]

    # calculate unit vectors
    unit_vec_left_elbow = [left_elbow_x, left_elbow_y] / np.linalg.norm([left_elbow_x, left_elbow_y])
    unit_vec_left_wrist = [left_wrist_x, left_wrist_y] / np.linalg.norm([left_wrist_x, left_wrist_y])

    # calculate dot product
    dot = np.dot(unit_vec_left_elbow, unit_vec_left_wrist)

    # calculate angle between left eblow and wrist
    angle = math.acos(round(dot, 3)) * 180.0 / 3.1415926

    return angle

    # angle_threshold_min = 80
    # angle_threshold_max = 100

    # # if it is about 90 degrees, then return 1
    # if angle > angle_threshold_min and angle < angle_threshold_max:
    #     return 1
    # else:
    #     return 0


def right_arm_bent(x_arr, y_arr):

    # get vectors of right arms
    right_elbow_x = x_arr[0] - x_arr[1]
    right_elbow_y = y_arr[0] - y_arr[1]
    right_wrist_x = x_arr[2] - x_arr[1]
    right_wrist_y = y_arr[2] - y_arr[1]

    # calculate unit vectors
    unit_vec_right_elbow = [right_elbow_x, right_elbow_y] / np.linalg.norm([right_elbow_x, right_elbow_y])
    unit_vec_right_wrist = [right_wrist_x, right_wrist_y] / np.linalg.norm([right_wrist_x, right_wrist_y])

    # calculate dot product
    dot = np.dot(unit_vec_right_elbow, unit_vec_right_wrist)

    # calculate angle between right eblow and wrist
    angle = math.acos(round(dot, 3)) * 180.0 / 3.1415926

    return angle

    # angle_threshold_min = 80
    # angle_threshold_max = 100

    # # if it is about 90 degrees, then return 1
    # if angle > angle_threshold_min and angle < angle_threshold_max:
    #     return 1
    # else:
    #     return 0


def left_wrist_status(y_arr):

    left_wrist_y = y_arr[5]
    left_elbow_y = y_arr[4]

    # if left wrist is up, then return 1
    if left_wrist_y > left_elbow_y:
        return 1
    else:
        return 0


def right_wrist_status(y_arr):

    right_wrist_y = y_arr[2]
    right_elbow_y = y_arr[1]

    # if right wrist is up, then return 1
    if right_wrist_y > right_elbow_y:
        return 1
    else:
        return 0