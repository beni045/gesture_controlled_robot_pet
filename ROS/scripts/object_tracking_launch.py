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
from model_processor import body_pose_ModelProcessor
from model_processor import object_tracking_ModelProcessor
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

MODEL_PATH_OBJECT_TRACKING = "../hand_gesture_controlled_robot_pet/models/mot_v2.om"
BODYPOSE_CONF="../hand_gesture_controlled_robot_pet/Atlas_robot_pet/param.conf"

class object_tracking_launch():
    def __init__(self):
    	# initiliaze
        rospy.init_node('object_tracking_launch', anonymous=True)
        self.pub = rospy.Publisher('object_tracking', String, queue_size=10)
        self.objimgpub = rospy.Publisher('objcamera', Image, queue_size=10)
        self.image_queue = Queue(maxsize=1)
        self.acl_resource = AclResource()
        self.acl_resource.init()
        object_tracking_model_parameters = {
            'model_dir': MODEL_PATH_OBJECT_TRACKING,
        }
        self.object_tracking_model_processor = object_tracking_ModelProcessor(self.acl_resource,object_tracking_model_parameters)
        self.stop_follow_flag = False
        self.Subscriber()
    def Subscriber(self):
        rospy.Subscriber("camera", Image, self.process_image, queue_size = 10, buff_size = 2 ** 24)
        #rospy.spin()
# publisher for img from object to robot pet launch        
    def convert_and_pubish(self, image_data):
        rospy.loginfo(image_data.shape)
        try:
            img_msg = CvBridge().cv2_to_imgmsg(image_data, "rgb8")
            rospy.loginfo(type(img_msg))

            self.objimgpub.publish(img_msg)
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

    def object_tracking(self):
        blank_image = np.zeros((1,1,3), np.uint8)
        curr_tid = -1
        count_notrack = 0
        global RGBMatrix
        RGBMatrix = blank_image
        while not rospy.is_shutdown():
            object_tracking_flag = rospy.get_param('/object_tracking_flag')
            if object_tracking_flag:
                coords = rospy.get_param('/coords')
                rospy.set_param('/rpi_signal_flag', 1)
                try:
                    if not self.image_queue.empty():
                        img_original = self.image_queue.get()
                        canvas, command, next_tid = self.object_tracking_model_processor.predict(img_original, coords, curr_tid)
                        if next_tid == -1:
                            if self.stop_follow_flag:
                                command = "STOP FOLLOW"
                            else:
                                count_notrack += 1
                                if count_notrack == 50:
                                    count_notrack = 0
                                    curr_tid = -1
                                    self.stop_follow_flag = True
                        else:
                            curr_tid = next_tid
                            count_notrack = 0

                        self.convert_and_pubish(canvas)

                        rospy.loginfo(command)
                        self.pub.publish(command)

                        _,jpeg_image = cv2.imencode('.jpg',canvas)
                        jpeg_image = AclImage(jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size)
                        rospy.loginfo("----Object Tracked----")
                except(ROSException, CvBridgeError, ROSSerializationException) as err:
                    rospy.logerr("Ran into exception when converting message type with CvBridge. See error below:")
                    raise err
            else:
                curr_tid = -1
                count_notrack = 0
                self.stop_follow_flag = False
    
    def process_image(self, data):
        if not self.image_queue.full():
            try:
                rgb_img = CvBridge().imgmsg_to_cv2(data, "rgb8")
                self.image_queue.put(rgb_img)
            except CvBridgeError as err:
                raise err


if __name__ == '__main__':
    try:
        object_tracking_launch_node = object_tracking_launch()
        object_tracking_launch_node.object_tracking()
    except rospy.ROSInterruptException:
        pass
