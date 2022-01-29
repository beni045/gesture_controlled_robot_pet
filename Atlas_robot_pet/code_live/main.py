import random
import os
import cv2
import numpy as np
import argparse
import sys
sys.path.append('..')
from model_processor import handpose_ModelProcessor
from model_processor import face_detection_ModelProcessor
from model_processor import hand_detection_ModelProcessor
from model_processor import body_pose_ModelProcessor
from atlas_utils.camera import Camera
from atlas_utils import presenteragent
from atlas_utils.acl_image import AclImage
import acl
from acl_resource import AclResource
import socket
#from Serial_servo_v1 import *
import threading
import time
import io
import struct
import pickle
import zlib

MODEL_PATH_HAND_POSE = "../model/handpose_argmax_bgr.om"
MODEL_PATH_FACE_DETECTION = "../model/face_detection.om"
MODEL_PATH_HAND_DETECTION = "../model/Hand_detection.om"
MODEL_PATH_BODY_POSE = "../model/body_pose.om"
BODYPOSE_CONF="../param.conf"
CAMERA_FRAME_WIDTH = 1280
CAMERA_FRAME_HEIGHT = 720

## Socket Initialization

#UDP_IP = '10.0.0.165' # Set to destination IP, RPi in this case
UDP_IP = '192.168.8.117'
#UDP_IP = '192.168.0.4'
UDP_PORT = 12345      # Port #, make sure same as on RPi

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

connection_established = 0

# If not connected to Raspberry Pi first time, keep trying to connect
while connection_established == 0:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((UDP_IP, UDP_PORT))
        connection_established = 1
    except:
        connection_established = 0


def send_serial_command():

 global command
 global RGBMatrix
 global current_command_state

 while(True):

  time.sleep(0.0000000001)

  data = b""
  payload_size = struct.calcsize(">L")
  print("payload_size: {}".format(payload_size))

  ####################################################################
  ### Sending Servo commands to Raspberry Pi for Camera Adjustment ###
  ####################################################################

  if command != "nothing" and current_command_state == "none":

    if command =='u':
        print("Sending: Servo Up")
        data = pickle.dumps("servo up\n", 0)
    
    elif command =='d':
        print("Sending: Servo Down")
        data = pickle.dumps("servo down\n", 0)
    
    elif command =='l':
        print("Sending: Servo Left")
        data = pickle.dumps("servo left\n", 0)
    
    elif command =='r':
        print("Sending: Servo Right")
        data = pickle.dumps("servo right\n", 0)

    command = "nothing"

  #####################################################
  ### Sending hand gesture commands to Raspberry Pi ###
  #####################################################

  elif current_command_state == "send_take_a_picture":
      print("Sending: Take a Picture")
      data = pickle.dumps("take a picture\n", 0)

  elif current_command_state == "send_forward":
      print("Sending: Forward")
      data = pickle.dumps("forward\n", 0)

  elif current_command_state == "send_backward":
      print("Sending: Backward")
      data = pickle.dumps("backward\n", 0)

  elif current_command_state == "send_spin":
      print("Sending: Spin")
      data = pickle.dumps("spin\n", 0)

  # Send image for Camera View if "Take a Picture" command is executing
  elif current_command_state == "take_a_picture":
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, send_frame = cv2.imencode('.jpg', RGBMatrix, encode_param)
        data = pickle.dumps(send_frame, 0)

  if data == b"":
       print("Sending: None")
       data = pickle.dumps("none\n", 0)

 ##################################################### 

  # Send to Raspberry Pi
  size = len(data)
  sock.sendall(struct.pack(">L", size) + data)

  # Reset data for receiving response from Raspberry Pi
  data = b""

  #################################################
  ### Receive acknowledgement from Raspberry Pi ###
  #################################################

  while len(data) < payload_size:
     print("Recv: {}".format(len(data)))
     data += sock.recv(4096)

  print("Done Recv: {}".format(len(data)))
  packed_msg_size = data[:payload_size]
  data = data[payload_size:]
  msg_size = struct.unpack(">L", packed_msg_size)[0]
  print("msg_size: {}".format(msg_size))

  while len(data) < msg_size:
     data += sock.recv(4096)

  frame_data = data[:msg_size]
  data = data[msg_size:]
  data=pickle.loads(frame_data, fix_imports=True, encoding="bytes")

  print(data)

  ######################################################
  ### Check if Raspberry Pi received serial commands ###
  ######################################################

  if data == "received take a picture\n":
       current_command_state = "take_a_picture"
       print(current_command_state)

  if data == "received forward\n":
       current_command_state = "forward"
       print(current_command_state)

  if data == "received backward\n":
       current_command_state = "backward"
       print(current_command_state)

  if data == "received spin\n":
       current_command_state = "spin"
       print(current_command_state)

  if data == "done command\n":
       current_command_state = "none"
       print(current_command_state)

###################

# Initialize some global variables

global command
command = "nothing"

blank_image = np.zeros((1,1,3), np.uint8)
global RGBMatrix
RGBMatrix = blank_image

global current_command_state #none, forwards, backwards, takeapicture
current_command_state = "none"

# Start serial communication with Raspberry Pi (a separate thread)
t1 = threading.Thread(target = send_serial_command,args=())
t1.start()

############
### Main ###
############

def execute(model_path):

    ## Initialization ##
    #initialize acl runtime 
    acl_resource = AclResource()
    acl_resource.init()

    # Global variables
    global command
    global current_command_state

    ## Prepare Model ##
    # parameters for model path and model inputs
    handpose_model_parameters = {
        'model_dir': model_path,
        'width': 256, # model input width      
        'height': 256, # model input height
    }

    hand_detection_model_parameters = {
        'model_dir': MODEL_PATH_HAND_DETECTION,
    }

    face_detection_model_parameters = {
        'model_dir': MODEL_PATH_FACE_DETECTION,
    }

    body_pose_model_parameters = {
        'model_dir': MODEL_PATH_BODY_POSE,
        'width': 368,
        'height': 368
    }

    # Prepare model instance: init (loading model from file to memory)
    # Model_processor: preprocessing + model inference + postprocessing
    handpose_model_processor = handpose_ModelProcessor(acl_resource, handpose_model_parameters)
    hand_detection_model_processor = hand_detection_ModelProcessor(acl_resource, hand_detection_model_parameters)
    face_detection_model_processor = face_detection_ModelProcessor(acl_resource, face_detection_model_parameters)
    body_pose_model_processor = body_pose_ModelProcessor(acl_resource, body_pose_model_parameters)
    
    ## Get Input ##
    # Initialize Camera
    cap = Camera(id = 0, fps = 10)

    ## Set Output ##
    # open the presenter channel
    chan = presenteragent.presenter_channel.open_channel(BODYPOSE_CONF)
    if chan == None:
        print("Open presenter channel failed")
        return


    while True:
        time.sleep(0.00000000001)
        ## Read one frame from Camera ## 
        img_original = cap.read()
        if not img_original:
            print('Error: Camera read failed')
            break

        # Camera Input (YUV) to BGR Image
        image_byte = img_original.tobytes()
        image_array = np.frombuffer(image_byte, dtype=np.uint8)
        img_original, RGBMatrix = YUVtoRGB(image_array)
        img_original = cv2.flip(img_original,1)

        ## Model Prediction ##
        # model_processor.predict: processing + model inference + postprocessing
        # canvas: the picture overlayed with human body joints and limbs
        canvas, xmin, xmax, ymin, ymax = hand_detection_model_processor.predict(img_original)
        canvas1, command = face_detection_model_processor.predict(img_original)  
        canvas, hg_command = handpose_model_processor.predict(canvas, xmin, xmax, ymin, ymax, canvas1)
        canvas = body_pose_model_processor.predict(canvas)  
        canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB, 3)  

        # Update hand gesture command (if necessary) to send to Raspberry Pi through serial
        if current_command_state == "none" or current_command_state == "send_take_a_picture" or current_command_state == "send_forward" or current_command_state == "send_backward" or current_command_state == "send_spin":
            if hg_command == "TAKE A PICTURE":
                current_command_state = "send_take_a_picture"
            if hg_command == "FORWARDS":
                current_command_state = "send_forward"
                #canvas, hg_command = handpose_model_processor.predict(canvas, xmin, xmax, ymin, ymax, canvas_body)
            if hg_command == "BACKWARDS":
                current_command_state = "send_backward"
            if hg_command == "SPIN":
                current_command_state = "send_spin"
            if hg_command == "STOP":
                current_command_state = "none"

        print("command state: " + str(current_command_state))
        
        ## Present Result ##
        # convert to jpeg image for presenter server display
        _,jpeg_image = cv2.imencode('.jpg',canvas)
        # construct AclImage object for presenter server
        jpeg_image = AclImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)
        # send to presenter server
        chan.send_detection_data(img_original.shape[0], img_original.shape[1], jpeg_image, [])

    # release the resources
    cap.release() 

##############

def YUVtoRGB(byteArray):

    global RGBMatrix #Send RGB image to RPi for Camera View

    e = 1280*720
    Y = byteArray[0:e]
    Y = np.reshape(Y, (720,1280))

    s = e
    V = byteArray[s::2]
    V = np.repeat(V, 2, 0)
    V = np.reshape(V, (360,1280))
    V = np.repeat(V, 2, 0)

    U = byteArray[s+1::2]
    U = np.repeat(U, 2, 0)
    U = np.reshape(U, (360,1280))
    U = np.repeat(U, 2, 0)

    Matrix = (np.dstack([Y,U,V])).astype(np.uint8)

    # YUV2BGR does not work well, do YUV2RGB then RGB2BGR
    RGBMatrix = cv2.cvtColor(Matrix, cv2.COLOR_YUV2RGB, 3)
    BGRMatrix = cv2.cvtColor(RGBMatrix, cv2.COLOR_RGB2BGR) 

    return BGRMatrix, RGBMatrix

##############   

if __name__ == '__main__':   

    description = 'Load a model for handpose'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--model', type=str, default=MODEL_PATH_HAND_POSE)
    args = parser.parse_args()

    execute(args.model)
