import cv2
import numpy as np
import math
import os, sys
sys.path.append('../..')

heatmap_width = 64
heatmap_height = 64

###########################
### DETERMINE BODY POSE ###
###########################

def decode_pose(heatmaps, scale, xmin, xmax, ymin, ymax, img_original, cropped_img, active):
  
    # Argmax of heatmap of output from handpose model are 21 coordinates representing points on the hand
    hand_point_list = [peak_index_to_coords(heatmap)*scale for heatmap in heatmaps]

    command = get_rc_command(hand_point_list, int(cropped_img.shape[1]))
    state = "None"
    if active:
        state = "ACTIVE"
    else:
        state = "INACTIVE"

    # plot the handpose on original image
    canvas = img_original

    # Write hand gesture command onto image in top left corner
    commandSize=cv2.getTextSize(command,cv2.FONT_HERSHEY_COMPLEX,1,2)

    # Write robot state onto image in bottom left corner
    stateSize=cv2.getTextSize(state,cv2.FONT_HERSHEY_COMPLEX,1,2)

    _x1 = 30
    _y1 = 30
    _x2 = _x1+commandSize[0][0]
    _y2 = _y1-int(commandSize[0][1])
    _x3 = 30
    _y3 = 720 - 30
    _x4 = _x3+stateSize[0][0]
    _y4 = _y3-int(stateSize[0][1])

    cv2.rectangle(canvas,(_x3 ,_y3 ),(_x4 ,_y4),(255,255,255),cv2.FILLED)
    cv2.putText(canvas,state,(_x3,_y3),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),2)
    if active:
        cv2.rectangle(canvas,(_x1 ,_y1 ),(_x2 ,_y2),(255,255,255),cv2.FILLED)
        cv2.putText(canvas,command,(_x1,_y1),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),2)


    # # Don't draw if no hand gesture is detected/hand gesture doesn't pass validation
    if command == "STOP":
        return canvas, command

    count = 0

    # Draw hand gesture model output on image
    for hand_point in hand_point_list:
      hand_point = np.array([hand_point[0] + xmin, hand_point[1] + ymin])
      canvas = cv2.circle(canvas, tuple(hand_point.astype(int)), radius=2, color=(0, 255, 0), thickness=4)
      if count == 8:
          canvas = cv2.circle(canvas, tuple(hand_point.astype(int)), radius=2, color=(255, 0, 0), thickness=4)
      elif count == 7:
          canvas = cv2.circle(canvas, tuple(hand_point.astype(int)), radius=2, color=(0, 0, 255), thickness=4)
      count += 1

    # Draw hand detection output on image, as well, if one was found
    if not (xmin * xmax * ymin * ymax) == 0:
      cv2.rectangle(canvas,(xmin,ymin),(xmax,ymax),(0,255,0),2)
      
    return canvas, command
##############

#def peak_index_to_coords(peak_index, scale):
def peak_index_to_coords(peak_index):
    '''
    @peak_index is the index of max value in flatten heatmap
    This function convert it back to the coordinates of the original heatmap 
    '''
    peak_coords = np.unravel_index(int(peak_index),(heatmap_height, heatmap_width))
    return np.flip(peak_coords)

######################################
### DETERMINE HAND GESTURE COMMAND ###
######################################

def get_rc_command(hand_point_list, width):

    x_arr = []
    y_arr = []

    for hand_point in hand_point_list:
       x_arr.append(hand_point[0])
       y_arr.append(hand_point[1])

    left_thresh = -500;
    left_thresh_2 = -900;
    right_thresh = 500;
    right_thresh_2 = 900;

    angle = get_fingers_angle_scale(x_arr, y_arr)
    thumb_bool = thumb_status(x_arr, y_arr)

    if validate(x_arr, y_arr, width) == False:
        return "STOP"
    
    elif((left_thresh < angle < right_thresh) and thumb_bool):
        return "BACKWARDS"

    elif(angle > right_thresh):
        return "TAKE A PICTURE"
    
    elif(angle < left_thresh):
        return "SPIN"
    
    else:
        return "FORWARDS"
    
##############

def validate(x_arr, y_arr, width):

    x_max_threshold = int((1200 / 1280) * width)
    x_min_threshold = int((50 / 1280) * width)
    
    if ((max(x_arr) > x_max_threshold) or (min(x_arr) < x_min_threshold)):
          return False
    elif (y_arr.index(min(y_arr)) != 8) and (y_arr.index(min(y_arr)) != 12) and (y_arr.index(min(y_arr)) != 16) and (y_arr.index(min(y_arr)) != 20):
          return False
    else: 
          return True
      
##############

def thumb_status(x_arr, y_arr):

    thumb_threshold = 5

    # get vector of thumb
    vec_thumb_x = x_arr[4] - x_arr[3]
    vec_thumb_y = y_arr[4] - y_arr[3]
    # get vector of index finger
    vec_index_x = x_arr[8] - x_arr[7]
    vec_index_y = y_arr[8] - y_arr[7]
    # length of thumb and index finger vector
    len_thumb = math.sqrt(pow(vec_thumb_x,2)+pow(vec_thumb_y,2))
    len_index = math.sqrt(pow(vec_index_x,2)+pow(vec_index_y,2))
    # thumb inward (direction = -1) or outward (direction = 1), using cross product of vectors
    if  (vec_thumb_x*vec_index_y-vec_index_x*vec_thumb_y)>0 :
       direction = -1
    else: 
       direction = 1 

    # get angle between thumb and index figure vector
    thumb_index_angle =direction* math.acos(round((vec_thumb_x*vec_index_x+vec_thumb_y*vec_index_y)/len_thumb/len_index,3))*180.0/3.1415926

    if (thumb_index_angle < thumb_threshold):
        return 1
    
    else: 
        return 0

##############

def get_fingers_angle_scale(x_arr, y_arr):

    finger_top_total = x_arr[8] + x_arr[12] + x_arr[16] + x_arr[20]
    finger_bottom_total = x_arr[5] + x_arr[9] + x_arr[13] + x_arr[17]
    angle_scale = finger_top_total - finger_bottom_total

    return angle_scale;









