#!/usr/bin/env python3
import random
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
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
from queue import Queue
from acl_resource import AclResource
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException, ROSSerializationException, ROSInitException, ROSInterruptException
import socket
#from Serial_servo_v1 import *
import threading
import time
import io
import struct
import pickle
import zlib

MODEL_PATH_FACE_DETECTION = "../hand_gesture_controlled_robot_pet/Atlas_robot_pet/model/face_detection.om"
BODYPOSE_CONF="../hand_gesture_controlled_robot_pet/Atlas_robot_pet/param.conf"


class face_detection_launch():
    def __init__(self):
    	# initiliaze
        rospy.init_node('face_detection_launch', anonymous=True)
        self.pub = rospy.Publisher('face', String, queue_size=10)
        self.faceimgpub = rospy.Publisher('face_camera', Image, queue_size=10)
        self.image_queue = Queue(maxsize=1)
        self.face_center_count = 0
        self.acl_resource = AclResource()
        self.acl_resource.init()
        face_detection_model_parameters = {
            'model_dir': MODEL_PATH_FACE_DETECTION,
        }
        self.face_detection_model_processor = face_detection_ModelProcessor(self.acl_resource,face_detection_model_parameters)
        self.Subscriber()
        # self.Subscriber()
    def Subscriber(self):
        #rospy.Subscriber("pet_gestures", String, self.face_detection)
        rospy.Subscriber("camera", Image, self.process_image, queue_size = 10, buff_size = 2 ** 24)
        #rospy.spin()
    
    # publisher for img from object to robot pet launch        
    def convert_and_pubish(self, image_data):
        rospy.loginfo(image_data.shape)
        try:
            img_msg = CvBridge().cv2_to_imgmsg(image_data, "rgb8")
            rospy.loginfo(type(img_msg))

            self.faceimgpub.publish(img_msg)
        except CvBridgeError as err:
            rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
            raise err
            
        except ROSSerializationException as err:
            rospy.logerr("Ran into exception when serializing message for publish. See error below:")
            raise err
        except ROSException as err:
            raise err
        except ROSInterruptException as err:
            rospy.loginfo("ROS Interrupt.")
            raise err

    def face_detection(self): #, data):
        blank_image = np.zeros((1,1,3), np.uint8)
        global RGBMatrix
        RGBMatrix = blank_image
        face_detection_flag = rospy.get_param("/face_detection_flag")
        adjust_at_start = True
        while not rospy.is_shutdown():
            face_detection_flag = rospy.get_param("/face_detection_flag")
            if face_detection_flag:
                rospy.set_param('/rpi_signal_flag', 1)

                if adjust_at_start:
                    #send move fwd & lookup to pi
                    self.pub.publish("u")
                    adjust_at_start = False
                else:
                    try:
                        if not self.image_queue.empty():
                            img_original = self.image_queue.get()
                            canvas, command = self.face_detection_model_processor.predict(img_original)

                            if (command == "centerd"):
                                self.face_center_count += 1
                                if (self.face_center_count < 5):
                                    command = "nothing"
                                else:
                                    self.face_center_count = 0
                            else:
                                self.face_center_count = 0
                                
                            self.convert_and_pubish(canvas)
                            rospy.loginfo(command)
                            rospy.set_param('/rpi_signal_flag', 1)
                            self.pub.publish(command)

                            _,jpeg_image = cv2.imencode('.jpg',canvas)
                            jpeg_image = AclImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)
                            rospy.loginfo("----Face Detected----")
                    except(ROSException, CvBridgeError, ROSSerializationException) as err:
                        rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
                        raise err
            else:
                adjust_at_start = True
    def process_image(self, data):
  
        if not self.image_queue.full():
            try:
                rgb_img = CvBridge().imgmsg_to_cv2(data, "rgb8")
                self.image_queue.put(rgb_img)
            except CvBridgeError as err:
                raise err


if __name__ == '__main__':
    try:
        face_detection_launch_node = face_detection_launch()
        face_detection_launch_node.face_detection()
    except rospy.ROSInterruptException:
        pass
