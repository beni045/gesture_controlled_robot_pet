#!/usr/bin/env python3
import random
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
import subprocess
import sys
from queue import Queue
from std_msgs.msg import UInt8
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

sys.path.append("../hand_gesture_controlled_robot_pet/Atlas_robot_pet")
# from model_processor import handpose_ModelProcessor
# from model_processor import face_detection_ModelProcessor
# from model_processor import hand_detection_ModelProcessor
# from model_processor import body_pose_ModelProcessor
# from atlas_utils.camera import Camera
# from atlas_utils import presenteragent
# from atlas_utils.acl_image import AclImage
# import acl
# from acl_resource import AclResource
import socket

# from Serial_servo_v1 import *

import struct
import pickle

MODEL_PATH_BODY_POSE = "../hand_gesture_controlled_robot_pet/models/body_pose.om"
BODYPOSE_CONF = "../hand_gesture_controlled_robot_pet/Atlas_robot_pet/param.conf"


class rpi_signal_launch:
    def __init__(self):
        # initiliaze
        rospy.init_node("rpi_signal_launch", anonymous=True)
        self.UDP_IP = "192.168.8.117"  # Set to destination IP, RPi in this case
        self.UDP_PORT = 12345  # Port #, make sure same as on RPi

        # If not connected to Raspberry Pi first time, keep trying to connect
        connection_established = 0
        while connection_established == 0:
            try:
                rospy.loginfo("socket.create")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                rospy.loginfo("sock.connect")
                self.sock.connect((self.UDP_IP, self.UDP_PORT))
                self.sock.settimeout(30.0)
                connection_established = 1
                rospy.loginfo("CONNECTED")
            except:
                connection_established = 0
                rospy.loginfo("trying to connect...")

        rospy.loginfo("!!!!!!!!!!!!peeko YAY !!!!!!!!!!!!!")
        blank_image = np.zeros((1, 1, 3), np.uint8)
        self.RGBMatrix = blank_image
        self.face_centered = 0
        self.image_queue = Queue(maxsize=1)
        self.data = b""
        self.hg_sig = "none"
        self.received_sig = "none"
        self.face_sig = "nothing"
        self.payload_size = struct.calcsize(">L")
        self.Subscriber()
        self.face_Sub()
        self.rgbSubscriber()
        self.follow_Sub()

    def Subscriber(self):
        rospy.Subscriber("pet_gestures", String, self.hg_signal)

    def hg_signal(self, sig_data):
        # Ignore user command (except STOP FOLLOW) when it's in FOLLOW mode
        if sig_data.data == "STOP FOLLOW" or not rospy.get_param("/object_tracking_flag"):
            # Ignore user command if the program is still resetting
            if not rospy.get_param("/reset_flag"):
                self.hg_sig = sig_data.data

    def face_Sub(self):
        rospy.Subscriber("face", String, self.face_signal)

    def face_signal(self, sig_data):
        # Ignore user command if the program is still resetting
        if not rospy.get_param("/reset_flag"):
            self.face_sig = sig_data.data

    def follow_Sub(self):
        rospy.Subscriber("object_tracking", String, self.follow_signal)

    def follow_signal(self, sig_data):
        # The commands are either camera rotation [UP, DOWN] OR
        # car movement commands [FORWARD, BACKWARD, LEFT, RIGHT]
        if sig_data.data in ["UP", "DOWN"]:
            self.face_sig = sig_data.data
        else:
            self.hg_sig = sig_data.data
            self.face_sig = "NOTHING"

    def rgbSubscriber(self):
        rospy.Subscriber("RGBmatrix", Image, self.rgb_signal)

    def rgb_signal(self, sig_data):
        if not self.image_queue.full():
            try:
                rgb_img = CvBridge().imgmsg_to_cv2(sig_data, "rgb8")
                self.image_queue.put(rgb_img)
            except CvBridgeError as err:
                raise err

    def rpi_signal(self):
        while not rospy.is_shutdown():
            rospy.loginfo("waiting for flag")
            rpi_signal_flag = rospy.get_param("/rpi_signal_flag")

            # Only process the commands when rpi_signal_flag is on
            if rpi_signal_flag:

                # If the reset flag is on, send only "RESET" command
                if rospy.get_param("/reset_flag"):
                    self.face_centered = 0
                    self.received_sig = "STOP"

                    self.face_sig = "NOTHING"
                    self.hg_sig = "RESET"

                face_detection_flag = rospy.get_param("/face_detection_flag")
                object_tracking_flag = rospy.get_param("/object_tracking_flag")
                rospy.loginfo("flag received")
                self.data = b""
                self.payload_size = struct.calcsize(">L")
                if not self.image_queue.empty():
                    self.RGBMatrix = self.image_queue.get()
                else:
                    self.RGBMatrix = np.zeros((1, 1, 3), np.uint8)

                if self.face_centered and self.received_sig == "take_a_picture":
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                    result, send_frame = cv2.imencode(".jpg", self.RGBMatrix, encode_param)
                    self.data = pickle.dumps(send_frame, 0)

                elif self.face_sig != "NOTHING":  # and self.hg_sig  == "STOP":

                    if self.face_sig == "UP":
                        rospy.loginfo("Sending: Servo Up")
                        self.data = pickle.dumps("servo up\n", 0)

                    elif self.face_sig == "DOWN":
                        rospy.loginfo("Sending: Servo Down")
                        self.data = pickle.dumps("servo down\n", 0)

                    elif self.face_sig == "LEFT":
                        rospy.loginfo("Sending: Servo Left")
                        self.data = pickle.dumps("servo left\n", 0)

                    elif self.face_sig == "RIGHT":
                        rospy.loginfo("Sending: Servo Right")
                        self.data = pickle.dumps("servo right\n", 0)

                    elif self.face_sig == "CENTERED":
                        rospy.loginfo("Face centered")
                        self.face_centered = 1

                    self.face_sig = "NOTHING"

                #####################################################
                ### Sending hand gesture commands to Raspberry Pi ###
                #####################################################

                elif (
                    self.hg_sig == "ACTIVATE"
                    and self.received_sig != "take_a_picture"
                    and not object_tracking_flag
                ):
                    rospy.loginfo("Sending: Activate")
                    self.data = pickle.dumps("activate\n", 0)

                elif (
                    self.hg_sig == "DEACTIVATE"
                    and self.received_sig != "take_a_picture"
                    and not object_tracking_flag
                ):
                    rospy.loginfo("Sending: Deactivate")
                    self.data = pickle.dumps("deactivate\n", 0)

                elif (
                    self.hg_sig == "TAKE A PICTURE"
                    and self.received_sig != "take_a_picture"
                    and not object_tracking_flag
                ):
                    rospy.loginfo("Sending: Take a Picture")
                    self.data = pickle.dumps("take a picture\n", 0)

                elif (
                    self.hg_sig == "SPIN LEFT"
                    and self.received_sig != "take_a_picture"
                    and not object_tracking_flag
                ):
                    rospy.loginfo("Sending: Spin Left")
                    self.data = pickle.dumps("spin left\n", 0)

                elif (
                    self.hg_sig == "SPIN RIGHT"
                    and self.received_sig != "take_a_picture"
                    and not object_tracking_flag
                ):
                    rospy.loginfo("Sending: Spin Right")
                    self.data = pickle.dumps("spin right\n", 0)

                elif self.hg_sig == "FORWARDS" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Forward")
                    self.data = pickle.dumps("forward\n", 0)

                elif self.hg_sig == "BACKWARDS" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Backward")
                    self.data = pickle.dumps("backward\n", 0)

                elif self.hg_sig == "LEFT" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Left")
                    self.data = pickle.dumps("left\n", 0)

                elif self.hg_sig == "RIGHT" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Right")
                    self.data = pickle.dumps("right\n", 0)

                elif self.hg_sig == "FOLLOW" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Follow")
                    self.data = pickle.dumps("follow\n", 0)

                elif self.hg_sig == "STOP FOLLOW" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: Stop Follow")
                    self.data = pickle.dumps("stop_follow\n", 0)

                elif self.hg_sig == "center for picture" and self.received_sig != "take_a_picture":
                    rospy.loginfo("Sending: center for picture")
                    self.data = pickle.dumps("center for picture\n", 0)

                elif self.hg_sig == "BODY":
                    rospy.loginfo("Sending: Body")
                    self.data = pickle.dumps("body\n", 0)

                elif self.hg_sig == "RESET":
                    rospy.loginfo("Sending: Reset")
                    self.data = pickle.dumps("reset\n", 0)

                if self.data == b"":
                    self.data = pickle.dumps("none\n", 0)

                #####################################################

                rospy.loginfo(self.hg_sig)
                rospy.loginfo(self.face_sig)
                # Send to Raspberry Pi
                size = len(self.data)
                self.sock.sendall(struct.pack(">L", size) + self.data)

                # Reset data for receiving response from Raspberry Pi
                self.data = b""

                #################################################
                ### Receive acknowledgement from Raspberry Pi ###
                #################################################

                while len(self.data) < self.payload_size:
                    #  rospy.loginfo("Recv: {}".format(len(data)))
                    try:
                        self.data += self.sock.recv(4096)
                    except socket.timeout:
                        # Shutdown the program if socket timeout
                        # (Usually there's unrecoverable error)
                        subprocess.Popen(["pkill", "-9", "rosmaster"])
                        subprocess.Popen(["pkill", "-9", "roscore"])
                        subprocess.Popen(["pkill", "-9", "python3"])

                packed_msg_size = self.data[: self.payload_size]
                self.data = self.data[self.payload_size :]
                msg_size = struct.unpack(">L", packed_msg_size)[0]

                while len(self.data) < msg_size:
                    try:
                        self.data += self.sock.recv(4096)
                    except socket.timeout:
                        subprocess.Popen(["pkill", "-9", "rosmaster"])
                        subprocess.Popen(["pkill", "-9", "roscore"])
                        subprocess.Popen(["pkill", "-9", "python3"])

                frame_data = self.data[:msg_size]
                self.data = self.data[msg_size:]
                self.data = pickle.loads(frame_data, fix_imports=True, encoding="bytes")

                if str(self.data) != "got it \n":
                    rospy.loginfo("Received data -----------------------------------")
                    rospy.loginfo(self.data)

                ######################################################
                ### Check if Raspberry Pi received serial commands ###
                ######################################################

                if self.data == "received activate\n":
                    self.received_sig = "activate"
                    rospy.loginfo(self.received_sig)
                    rospy.set_param("/active_flag", 1)

                elif self.data == "received deactivate\n":
                    self.received_sig = "deactivate"
                    rospy.loginfo(self.received_sig)
                    rospy.set_param("/active_flag", 0)
                    rospy.set_param("/reset_flag", 1)

                elif self.data == "received take a picture\n":
                    self.received_sig = "take_a_picture"
                    rospy.loginfo(self.received_sig)
                    if self.face_centered:
                        rospy.set_param("/body_face_detection_flag", 0)

                elif self.data == "received spin left\n":
                    self.received_sig = "spin left"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received spin right\n":
                    self.received_sig = "spin right"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received forward\n":
                    self.received_sig = "forward"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received backward\n":
                    self.received_sig = "backward"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received left\n":
                    self.received_sig = "left"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received right\n":
                    self.received_sig = "right"
                    rospy.loginfo(self.received_sig)

                elif self.data == "received center\n":
                    self.received_sig = "center"
                    rospy.loginfo(self.received_sig)
                    rospy.set_param("/take_a_pic_flag", 1)

                elif self.data == "received follow\n":
                    self.received_sig = "follow"
                    rospy.loginfo(self.received_sig)
                    rospy.set_param("/object_tracking_flag", 1)

                elif self.data == "received stop_follow\n":
                    self.received_sig = "stop_follow"
                    rospy.set_param("/object_tracking_flag", 0)
                    rospy.loginfo(self.received_sig)

                elif self.data == "received reset\n":
                    self.received_sig = "reset"
                    rospy.loginfo(self.received_sig)
                    rospy.set_param("/reset_flag", 0)
                    rospy.set_param("/rpi_signal_flag", 0)

                elif self.data == "done command\n":
                    rospy.set_param("/face_detection_flag", 0)
                    rospy.set_param("/rpi_signal_flag", 0)
                    self.face_centered = 0
                    self.received_sig = "STOP"
                    rospy.loginfo(self.received_sig)
            else:
                self.data = pickle.dumps("none\n", 0)
                size = len(self.data)
                self.sock.sendall(struct.pack(">L", size) + self.data)

                # Reset data for receiving response from Raspberry Pi
                self.data = b""
                while len(self.data) < self.payload_size:
                    #  rospy.loginfo("Recv: {}".format(len(data)))
                    self.data += self.sock.recv(4096)

                #   rospy.loginfo("Done Recv: {}".format(len(data)))
                packed_msg_size = self.data[: self.payload_size]
                self.data = self.data[self.payload_size :]
                msg_size = struct.unpack(">L", packed_msg_size)[0]
                #   rospy.loginfo("msg_size: {}".format(msg_size))

                while len(self.data) < msg_size:
                    self.data += self.sock.recv(4096)

                frame_data = self.data[:msg_size]
                self.data = self.data[msg_size:]
                self.data = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
                self.data = b""


if __name__ == "__main__":
    try:
        launch = rpi_signal_launch()
        launch.rpi_signal()
    except rospy.ROSInterruptException:
        pass
