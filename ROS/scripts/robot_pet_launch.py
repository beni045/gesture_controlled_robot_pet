#!/usr/bin/env python3
import random

from cv2 import rotate
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
import subprocess
from queue import Queue
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import (
    ROSException,
    ROSSerializationException,
    ROSInitException,
    ROSInterruptException,
)
from std_msgs.msg import UInt8
from rospy.numpy_msg import numpy_msg

sys.path.append("../hand_gesture_controlled_robot_pet/Atlas_robot_pet")
from model_processor import body_pose_ModelProcessor
from atlas_utils.camera import Camera
from atlas_utils import presenteragent
from atlas_utils.acl_image import AclImage
import acl
from acl_resource import AclResource

from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import (
    ROSException,
    ROSSerializationException,
    ROSInitException,
    ROSInterruptException,
)

MODEL_PATH_BODY_POSE = "../hand_gesture_controlled_robot_pet/models/body_pose.om"
BODYPOSE_CONF = "../hand_gesture_controlled_robot_pet/Atlas_robot_pet/param.conf"
HG_COUNTER_MAX = 3
NOTHING_COUNTER_MAX = 10


class robot_pet_launch:
    def __init__(self):
        # initiliaze
        rospy.init_node("robot_pet", anonymous=True)
        self.pub = rospy.Publisher("pet_gestures", String, queue_size=10)
        self.camerapub = rospy.Publisher("camera", Image, queue_size=10)
        self.facepub = rospy.Publisher("face", String, queue_size=10)
        self.image_queue = Queue(maxsize=1)
        self.rgbpub = rospy.Publisher("RGBmatrix", Image, queue_size=10)

        self.cap = Camera(id=1, fps=10)
        self.chan = presenteragent.presenter_channel.open_channel(BODYPOSE_CONF)
        self.acl_resource = AclResource()
        self.acl_resource.init()

        self.curr_hg_command = "STOP"
        self.hg_counter = 0
        self.face_center_count = 0
        self.nothing_count = 0

        self.activate_flag = False
        self.take_a_pic_flag = False
        self.centering_flag = False
        self.face_detection_flag = 0
        rospy.set_param("/active_flag", 0)
        rospy.set_param("/keep_follow_flag", 0)
        rospy.set_param("/rpi_signal_flag", 0)
        rospy.set_param("/take_a_pic_flag", 0)
        rospy.set_param("/face_detection_flag", 0)
        rospy.set_param("/body_face_detection_flag", 0)
        rospy.set_param("/object_tracking_flag", 0)
        rospy.set_param("/hand_gestures_flag", 1)
        rospy.set_param("/reset_flag", 0)
        rospy.set_param("/coords", (-1, -1, -1, -1))
        self.Subscriber()
        self.face_subscriber()

    def Subscriber(self):
        rospy.Subscriber("objcamera", Image, self.process_image, queue_size=10, buff_size=2**24)

    def face_subscriber(self):
        rospy.Subscriber("face_camera", Image, self.process_image, queue_size=10, buff_size=2**24)

    def process_image(self, data):
        if not self.image_queue.full():
            try:
                rgb_img = CvBridge().imgmsg_to_cv2(data, "rgb8")
                self.image_queue.put(rgb_img)
            except CvBridgeError as err:
                raise err

    def convert_and_pubish(self, image_data):
        rospy.loginfo(image_data.shape)
        try:
            img_msg = CvBridge().cv2_to_imgmsg(image_data, "rgb8")
            rospy.loginfo(type(img_msg))

            self.camerapub.publish(img_msg)
        except CvBridgeError as err:
            rospy.logerr(
                "Ran into exception when converting message type with CvBridge. See error below:"
            )
            raise err

        except ROSSerializationException as err:
            rospy.logerr(
                "Ran into exception when serializing message for publish. See error below:"
            )
            raise err
        except ROSException as err:
            raise err
        except ROSInterruptException as err:
            rospy.loginfo("ROS Interrupt.")
            raise err

    def pet_gestures(self):
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(10)  # 10hz
        bodypose_model_parameters = {
            "model_dir": MODEL_PATH_BODY_POSE,
            "width": 368,  # model input width
            "height": 368,  # model input height
        }
        blank_image = np.zeros((1, 1, 3), np.uint8)
        global RGBMatrix
        RGBMatrix = blank_image
        bodypose_model_processor = body_pose_ModelProcessor(
            self.acl_resource, bodypose_model_parameters
        )
        while not rospy.is_shutdown():
            self.activate_flag = rospy.get_param("/active_flag")
            hand_gestures_flag = rospy.get_param("/hand_gestures_flag")
            self.take_a_pic_flag = rospy.get_param("/take_a_pic_flag")
            self.face_detection_flag = rospy.get_param("/body_face_detection_flag")
            rospy.loginfo(self.face_detection_flag)
            if hand_gestures_flag:
                img_original = self.cap.read()
                if not img_original:
                    rospy.loginfo("Error: Camera read failed")
                    break

                # Camera Input (YUV) to BGR Image
                image_byte = img_original.tobytes()
                image_array = np.frombuffer(image_byte, dtype=np.uint8)
                img_original, RGBMatrix = self.YUVtoRGB(image_array)

                # Try to convert and preprocess the image
                try:
                    RGBMatrix = CvBridge().cv2_to_imgmsg(RGBMatrix, "rgb8")
                    rospy.loginfo(type(RGBMatrix))

                    self.rgbpub.publish(RGBMatrix)
                except CvBridgeError as err:
                    rospy.logerr(
                        "Ran into exception when converting message type with CvBridge. See error below:"
                    )
                    raise err
                except ROSSerializationException as err:
                    rospy.logerr(
                        "Ran into exception when serializing message for publish. See error below:"
                    )
                    raise err
                except ROSException as err:
                    raise err
                except ROSInterruptException as err:
                    rospy.loginfo("ROS Interrupt.")
                    raise err

                img_original = cv2.flip(img_original, 1)
                img_original = cv2.cvtColor(img_original, cv2.COLOR_BGR2RGB, 3)

                # Publish the image such that other nodes can access as well
                self.convert_and_pubish(img_original)

                # Run the body gesture model to get
                # annotated image, gesture command, coords of the user, camera rotate command
                canvas, hg_command, coords, rotate = bodypose_model_processor.predict(
                    img_original, img_original, self.activate_flag
                )

                # We only register a command if the same commands have been detected HG_COUNTER_MAX times in a row
                # Otherwise the command will be ignored (treated as STOP)
                if hg_command == self.curr_hg_command:
                    self.hg_counter += 1
                    if self.hg_counter < HG_COUNTER_MAX:
                        hg_command = "STOP"
                    else:
                        self.hg_counter = 0
                else:
                    self.curr_hg_command = hg_command
                    hg_command = "STOP"
                    self.hg_counter = 0

                # Send ACTIVATE/DEACTIVATE to the Pi
                if hg_command == "ACTIVATE" or hg_command == "DEACTIVATE":
                    rospy.set_param("/rpi_signal_flag", 1)

                # Only process a command if the robot has been activated
                if self.activate_flag:
                    if self.take_a_pic_flag:
                        # Smart Photography - Moving Forward State
                        hg_command, rotate = self.check_nothing_count(hg_command, rotate)
                        ((x_up, y_up), (x_down, y_down)) = coords
                        width = x_down - x_up
                        height = y_down - y_up
                        if width * height < 0.048 * 1280 * 720:
                            # The robot will try to get closer to the user
                            rospy.loginfo(width)
                            rospy.loginfo(height)
                            if width * height != 0:
                                hg_command = "FORWARDS"
                                rospy.set_param("/rpi_signal_flag", 1)
                        else:
                            # If the user is close enough, move on to Centering State
                            self.nothing_count = 0
                            rospy.set_param("/take_a_pic_flag", 0)
                            hg_command = "STOP"
                            rospy.set_param("/body_face_detection_flag", 1)

                    elif self.centering_flag:
                        # Smart Photography - Sending "TAKE A PICTURE" command to the Pi
                        # Send command once and go back to the Centering State
                        # This will repeat until Pi is ready to take the picture
                        hg_command = "TAKE A PICTURE"
                        rotate = "NOTHING"
                        self.centering_flag = False
                        rospy.loginfo(rotate)
                        self.facepub.publish(rotate)

                    elif self.face_detection_flag:
                        # Smart Photography - Centering State
                        # Rotate the camera to center the user
                        hg_command, rotate = self.check_nothing_count(hg_command, rotate)
                        hg_command = "STOP"

                        # The user is considered as truely "centered" only if there are 5 consecutive "CENTERED"
                        if rotate == "CENTERED":
                            self.face_center_count += 1
                            if self.face_center_count < 5:
                                hg_command = "STOP"
                                rotate = "NOTHING"
                            else:
                                self.face_center_count = 0
                                self.centering_flag = True

                        rospy.loginfo(rotate)
                        rospy.set_param("/rpi_signal_flag", 1)
                        self.facepub.publish(rotate)

                    elif hg_command == "TAKE A PICTURE":
                        if not rospy.get_param("/object_tracking_flag"):
                            # If it's not in Follow mode, inform Pi that Smart Photography routine starts
                            hg_command = "center for picture"
                            rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "SPIN LEFT":
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "SPIN RIGHT":
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "FORWARDS":
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "STOP FOLLOW":
                        # Stop object tracking
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "FOLLOW":
                        # Start object tracking
                        if not rospy.get_param("/face_detection_flag"):
                            # Record the coordinates of the user that issued the command
                            rospy.set_param("/coords", coords)
                            rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "BACKWARDS":
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "ACTIVATE" or hg_command == "DEACTIVATE":
                        rospy.set_param("/rpi_signal_flag", 1)
                    elif hg_command == "BODY":
                        # This is used to indicated body is in the frame but no valid command detected
                        rospy.set_param("/rpi_signal_flag", 1)

                else:
                    self.nothing_count = 0
                    self.take_a_pic_flag = False
                    self.centering_flag = False
                    rospy.set_param("/body_face_detection_flag", 0)

                rospy.loginfo(hg_command)
                self.pub.publish(hg_command)

                # Get the annotated image from the object tracking model
                if rospy.get_param("/object_tracking_flag"):
                    canvas = self.image_queue.get()

                _, jpeg_image = cv2.imencode(".jpg", canvas)
                jpeg_image = AclImage(
                    jpeg_image, img_original.shape[0], img_original.shape[1], jpeg_image.size
                )

                # Send the image to the presenter server
                self.chan.send_detection_data(
                    img_original.shape[0], img_original.shape[1], jpeg_image, []
                )

                rate.sleep()

    # Used in Smart Photography routine
    # When no body is detected in the frame (user is out of the frame),
    # the rotate command will be "NOTHING".
    # If this persists for a while, the smart phtography routine should be terminated
    def check_nothing_count(self, hg_command, rotate):
        if rotate == "NOTHING":
            self.nothing_count += 1
            if self.nothing_count > NOTHING_COUNTER_MAX:
                # When the user is away for a while,
                # Reset all flags/counters and inform the Pi
                hg_command = "STOP"
                rotate = "NOTHING"
                self.nothing_count = 0
                self.centering_flag = False
                rospy.set_param("/reset_flag", 1)
                rospy.set_param("/rpi_signal_flag", 1)
                rospy.set_param("/take_a_pic_flag", 0)
                rospy.set_param("/body_face_detection_flag", 0)
        else:
            self.nothing_count = 0

        return hg_command, rotate

    def cleanup(self):
        self.cap.close()
        subprocess.Popen(["pkill", "-9", "rosmaster"])
        subprocess.Popen(["pkill", "-9", "roscore"])
        subprocess.Popen(["pkill", "-9", "python3"])

    def YUVtoRGB(self, byteArray):

        global RGBMatrix  # Send RGB image to RPi for Camera View

        e = 1280 * 720
        Y = byteArray[0:e]
        Y = np.reshape(Y, (720, 1280))

        s = e
        V = byteArray[s::2]
        V = np.repeat(V, 2, 0)
        V = np.reshape(V, (360, 1280))
        V = np.repeat(V, 2, 0)

        U = byteArray[s + 1 :: 2]
        U = np.repeat(U, 2, 0)
        U = np.reshape(U, (360, 1280))
        U = np.repeat(U, 2, 0)

        Matrix = (np.dstack([Y, U, V])).astype(np.uint8)

        # YUV2BGR does not work well, do YUV2RGB then RGB2BGR
        RGBMatrix = cv2.cvtColor(Matrix, cv2.COLOR_YUV2RGB, 3)
        BGRMatrix = cv2.cvtColor(RGBMatrix, cv2.COLOR_RGB2BGR)

        return BGRMatrix, RGBMatrix


if __name__ == "__main__":
    try:
        launch = robot_pet_launch()
        launch.pet_gestures()
    except rospy.ROSInterruptException:
        pass
