# Hand Gesture Controlled Robot Pet
Consider having a pet that is smart, responds to your commands without training, and even takes pictures for you! You don’t even have to pick up after or buy kibble for this pet! This is a dream come true for those craving an animal companion but cannot afford the time, money, or maintenance required for real animals. Robot pets are one example of how technology can assist in scenarios where companionship and fun are needed, but the handling and upkeep of real pets isn’t possible; especially in healthcare or senior homes. 

Apart from doing normal pet things such as moving around and performing a few tricks, it can also perform robot tasks, like capturing photos with the camera in its nose. It can also communicate its mood through animated eyes on a colored LCD screen mounted to its head.

Through its camera “eyes”, this robot pet uses a combination of machine learning and computer vision to see and respond to commands given through its owner’s hand gestures. The robot pet can track, recognize, and perform a task such as following the user while video capturing. The user is thus able to control the pet from a distance, enabling full autonomy.

<p align="center">
  <img width="300" src="/Images/robot.png">
</p>

# Repo
<p align="center">
  <img src="/Images/repo.png">
</p>

# Hardware
* [Huawei's Atlas 200DK](https://e.huawei.com/en/products/cloud-computing-dc/atlas/atlas-200)
* [Raspberry Pi (3+)](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)
* [Raspberry Pi Camera (Camera V2)](https://www.raspberrypi.org/products/camera-module-v2/)
* [Raspberry Pi LCD Screen (Kuman 3.5" Inch TFT LCD Display 480x320 RGB Pixels)](https://www.amazon.ca/Kuman-Display-480x320-Raspberry-Interface/dp/B01CQIPEO0/ref=sr_1_6?dchild=1&keywords=3.5+inch+raspberry+pi+screen&qid=1605052453&sr=8-6)
* [ELEGOO UNO Robot Car Base](https://www.amazon.ca/ELEGOO-Ultrasonic-Bluetooth-Intelligent-Educational/dp/B07485YQP8)
* Optional:
  * 3D printer
  * [2 Servo Motors (SG90)](https://www.amazon.ca/gp/product/B07Z16DWGW/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1)
* Miscellaneous:
  * [Raspberry Pi Camera Extension Cable](https://www.amazon.ca/gp/product/B07GWRZDH7/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)
  * Ethernet Cable
  * Jumper Cables
  * Screws

<p align="center">
  <img width="300" src="/Images/final.jpg">
</p>

### Setup and Connection Notes
* Build robot base without using the Arduino or the ultrasonic sensors, you can find the instructions [here](https://drive.google.com/file/d/1nSlkYJ7oCfMkG1p-KDfVHdLQt3B4Nmo5/view)
* Connect robot base parts to Raspberry Pi (see pinout in RaspberryPi_robot_pet/server_v9.py)
* Connect Raspberry Pi Camera to CAMERA1 on the Atlas 200 DK, you can follow this [guide](https://support.huaweicloud.com/intl/en-us/qs-atlas200dkappc32/atlased_04_0006.html)
* [Router connection](https://github.com/kylerhunag/gesture_controlled_robot_pet/wiki/Router-Connection-Setup-Guide) (After you've completed 1st step of [Atlas 200DK Setup](https://github.com/kylerhunag/gesture_controlled_robot_pet/edit/main/README.md#atlas-200dk-setup) and 2nd step of [Raspberry Pi Setup](https://github.com/kylerhunag/gesture_controlled_robot_pet/edit/main/README.md#raspberry-pi-setup))
  * Raspberry Pi connected to Atlas 200DK via wireless connection
  * Atlas 200DK connected to the router via Ethernet cord
  * Laptop connected to the router via wireless connection

# Setup
### Development Environment (PC)
Development PC requires Ubuntu 18.04 system (can be virtual machine), and have CANN installed. Please setup follow the this official [guide](https://support.huaweicloud.com/intl/en-us/environment-deployment-Atlas200DK202/atlased_04_0017.html) or this [wiki](https://gitee.com/ascend/samples/wikis/Setting%20Up%20the%20Development%20and%20Operating%20Environments?sort_id=3736936)

### Atlas 200DK Setup
1. Setup the Atlas 200 DK board, following this [wiki](https://gitee.com/ascend/samples/wikis/Setting%20Up%20the%20Development%20and%20Operating%20Environments?sort_id=3736936) or the official [guide](https://support.huaweicloud.com/intl/en-us/environment-deployment-Atlas200DK202/atlased_04_0001.html)

2. Power the Atlas DK board and set up the router connections. You can follow this [guide](https://github.com/kylerhunag/gesture_controlled_robot_pet/wiki/Router-Connection-Setup-Guide)

3. Attach raspberry Pi camera to the board (https://support.huaweicloud.com/intl/en-us/qs-atlas200dkappc32/atlased_04_0006.html).  

4. Set up RC Car by following instructions [here](https://drive.google.com/file/d/1nSlkYJ7oCfMkG1p-KDfVHdLQt3B4Nmo5/view).
  
### Raspberry Pi Setup
1. Install the latest version of Raspbian onto an SD card  
   Follow the steps in the link:  
   https://www.raspberrypi.org/documentation/installation/installing-images/ 
2. Insert SD card into Pi and power on the Pi by plugging it into a 5V power source.
3. Make sure you complete the [Router connection](https://github.com/kylerhunag/gesture_controlled_robot_pet/wiki/Router-Connection-Setup-Guide)
4. Update the system by opening a terminal and entering:  
   sudo apt-get update                                                        
   sudo apt-get upgrade
5. Install OpenCV for Python 3:  
   sudo apt-get install python3-opencv
7. Install the LCD driver  
   Follow instruction under the link:  
   https://github.com/goodtft/LCD-show 
6. Retrieve the files to be downloaded onto the Raspberry Pi, which are located in the folder “RaspberryPi_robot_pet”, either from the source code folder provided or from the GitHub repository. Move the following files onto the Raspberry Pi Desktop in the folder “RaspberryPi_robot_pet”:
    * Raspberry-Pi_robot_pet/happy.mp4
    * Raspberry-Pi_robot_pet/neu.mp4
    * Raspberry-Pi_robot_pet/white.jpeg
    * Raspberry-Pi_robot_pet/body_img.jpg
    * Raspberry-Pi_robot_pet/camera_img.jpg
    * Raspberry-Pi_robot_pet/deactivate.jpg
    * Raspberry-Pi_robot_pet/follow_img.jpg
    * Raspberry-Pi_robot_pet/server_v9.py

# Gestures
Below are the gestures included in the repository.

Command | Gesture | Description | Output
------- | ------- | ----------- | ------
`Activate` | <img src="/Images/activate.PNG" width="250" height="250"> | Robot starts functioning and responding to commands | <img src="/Images/activate.gif" width="250" height="250" />
`Deactivate` | <img src="/Images/deactivate.PNG" width="250" height="250"> | Robot goes to “sleep” state and can only be woken up by giving it activate command | <img src="/Images/deactivate.gif" width="250" height="250" />
`Move Forwards` | <img src="/Images/fwd.PNG" width="250" height="250"> | Moves the robot forwards | <img src="/Images/fwd.gif" width="250" height="250" />
`Move Backwards` | <img src="/Images/bwd.PNG" width="250" height="250"> | Moves the robot backwards | <img src="/Images/bwd.gif" width="250" height="250" />
`Spin Right` | <img src="/Images/spinr.PNG" width="250" height="250"> | Spins the robot right | <img src="/Images/spinr.gif" width="250" height="250" />
`Spin Left` | <img src="/Images/spinl.PNG" width="250" height="250"> | Spins the robot left | <img src="/Images/spinl.gif" width="250" height="250" />
`Take a Picture` | <img src="/Images/tap.PNG" width="250" height="250"> | Initiates the routine to take a photo | <img src="/Images/tap.gif" width="250" height="250" />
`Follow` | <img src="/Images/follow.PNG" width="250" height="250"> | Initiates the routine to follow the person who gave the follow gesture | <img src="/Images/follow.gif" width="250" height="250" />
`Stop Follow` | <img src="/Images/stopfollow.PNG" width="250" height="250"> | Stops the follow routine if the robot is in follow mode | <img src="/Images/stopfollow.gif" width="250" height="250" />


# Run the Robot

### Atlas 200DK 
1. Download/Clone this repo to your development PC as well as Atlas 200 DK.
1. Open a terminal on your development PC, and navigate to the project directory, for example: 
   cd hand_gesture_controlled_robot_pet/script/  
3. In the same terminal, run the presenter server (more information in the next section below):  
   bash ./run_presenter_server.sh
5. Open a second terminal, and SSH into the Atlas 200 DK:  
   ssh HwHiAiUser@192.168.1.2  
  (The IP address for USB connection of the Atlas 200 DK will normally be 192.168.1.2)  
  The default password to SSH is Mind@123.
4. Navigate to the folder which holds the main program of the application, for example: 
   cd /home/HwHiAiUser/HIAI_PROJECTS/Atlas_robot_pet/code_live/
6. Run the main program on the Atlas 200 DK:  
   python3 main.py
   
#### Presenter Server

Note, in the project, [Presenter Server](https://gitee.com/Atlas200DK/sdk-presenter) is used to demo the video captured from the camera.

Modify the configuration file, if you need to view the detection results using presenter server for the live input.

Modify <i>presenter_server_ip</i> and <i>presenter_view_ip</i> in <i>body_pose.conf</i> to the current ubuntu server and atlas200dk development board network port ip, presenter _agent_ip is the ip of the network port connected to the ubuntu server on the development board.

If you use USB connection, the USB network port ip of the development board is 192.168.1.2, and the network port ip of the virtual network card connected to the ubuntu server and the development board is 192.168.1.223, then the configuration file content is as follows:

<i>presenter_server_ip=192.168.1.223

presenter_view_ip=192.168.1.223

presenter_agent_ip=192.168.1.2</i>

Generally, when connecting via USB, atlas200dk_board_ip is the USB network port ip of the development board, and the default is 192.168.1.2.

If you need to view the detection results using presenter server for the live input or video source, log in to the Presenter Server website using the URL that was prompted when the Presenter Server service was started. Otherwise, skip this step.

Wait for the Presenter Agent to transmit data to the server, and click "Refresh" to refresh. When there is data, the status of the corresponding Channel turns green.

Click the corresponding View Name link on the right to view the results.

#### Stopping Presenter Server

The Presenter Server service will always be running after it is started. If you want to stop the Presenter Server service corresponding to the pose detection application, you can perform the following operations.

Execute the following command on the command line on the server where the process of the Presenter Server service is running:

<b>ps -ef | grep presenter</b>

<i>ascend@ubuntu:~/AscendProjects/hand_gesture_controlled_robot_pet
/Atlas_robot_pet/script$ ps -ef | grep presenter
<br/>ascend 9560 1342 0 02:19 pts/4  00:00:04   python3/home/ascend/AscendProjects/hand_gesture_controlled_robot_pet
/Atlas_robot_pet.bak/script/..//presenterserver/presenter_server.py --app Atlas_robot_pet
</i>

As shown above, 9650 is the process ID of the Presenter Server service corresponding to the Atlas_robot_pet application.

If you want to stop this service, execute the following command:

<b>kill -9 9650</b>

### Raspberry Pi
1. Start Raspberry Pi into Raspbian OS
2. Navigate to the directory where the main Raspberry Pi program is located:  
   cd /home/pi/Desktop/Raspberry-Pi_robot_pet/
4. Run the main program for the robot control:  
   python3 robot_server.py

# Dependencies and Third Party Links
* Hand Gesture Recognition Model: https://github.com/Atlas200dk/sample-handposeRCcar
* Hand Detection Model: https://github.com/victordibia/handtracking 
* Face Detection Model: https://github.com/Ascend-Huawei/OfflineModelSamples/tree/main/face_detection
* Raspbian image: https://www.raspberrypi.org/documentation/installation/installing-images/
* LCD driver: https://github.com/goodtft/LCD-show 
* OpenCV

Python libraries used for Atlas 200 DK:
* Random
* Os
* Cv2
* Numpy
* Argparse
* Sys
* Socket
* Threading
* Time
* Io
* Struct
* Pickle
* Zlib

For setup Python environment, you can refer to this [guide](https://gitee.com/ascend/samples/tree/master/python/environment).


Python libraries used for Raspberry Pi:
* Socket
* Sys
* Pickle
* Numpy
* Struct
* Zlib
* RPi.GPIO
* Time
* Os
* Cv2
* Math
* Threading
* Datetime


# Notes
Power supply not solved
