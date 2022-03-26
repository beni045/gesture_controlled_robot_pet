#!/usr/bin/env python3
import random
import rospy
from std_msgs.msg import String
import os
import cv2
import numpy as np
import argparse
import sys
sys.path.append('../hand_gesture_controlled_robot_pet/Atlas_robot_pet')
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

MODEL_PATH_BODY_POSE = "../hand_gesture_controlled_robot_pet/Atlas_robot_pet/model/body_pose.om"
BODYPOSE_CONF="../hand_gesture_controlled_robot_pet/Atlas_robot_pet/param.conf"

class body_pose_launch():
    def __init__(self, model_path):
    	# initiliaze
        rospy.init_node('body_pose_launch', anonymous=True)
        self.pub = rospy.Publisher('body_pose', String, queue_size=10)
        #self.present_pub = rospy.Publisher('presenter', someobject, queue_size=10)
        self.acl_resource, self.cap, self.chan = rospy.get_param("/init_params")
        self.Subscriber()
    def Subscriber(self):
        rospy.Subscriber("pet_gestures", String, body_pose)
        rospy.spin()
    def body_pose(self, data):
        if (data.data == "FORWARDS"):
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            body_pose_model_parameters = {
                'model_dir': MODEL_PATH_BODY_POSE,
                'width': 368,
                'height': 368
            }
            blank_image = np.zeros((1,1,3), np.uint8)
            global RGBMatrix
            RGBMatrix = blank_image
            body_pose_model_processor = body_pose_ModelProcessor(acl_resource, body_pose_model_parameters)
            while not rospy.is_shutdown():
                img_original = self.cap.read()
                if not img_original:
                    print('Error: Camera read failed')
                    break

                # Camera Input (YUV) to BGR Image
                image_byte = img_original.tobytes()
                image_array = np.frombuffer(image_byte, dtype=np.uint8)
                img_original, RGBMatrix = YUVtoRGB(image_array)
                img_original = cv2.flip(img_original,1)
                
                canvas = body_pose_model_processor.predict(canvas)  
                canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB, 3)

                _,jpeg_image = cv2.imencode('.jpg',canvas)
                jpeg_image = AclImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)

                #presenter_serverobj = someobject
                #presenter_serverobj.a = SimpleNamespace()
                #params = [img_original, jpeg_image]
                #for p in params:
                #    setattr(presenter_serverobj.a, p, value)
                #self.present_pub.pub(presenter_serverobj)
                self.chan.send_detection_data(img_original.shape[0], img_original.shape[1], jpeg_image, [])
                rate.sleep()
            cap.release()


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
if __name__ == '__main__':
    try:
        description = 'Load a model for handpose'
        parser = argparse.ArgumentParser(description=description)
        parser.add_argument('--model', type=str, default=MODEL_PATH_BODY_POSE)
        args = parser.parse_args()
        launch = body_pose_launch(args.model)
    except rospy.ROSInterruptException:
        pass
