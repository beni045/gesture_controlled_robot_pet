# Gesture Controlled Robot Pet

Consider having a pet that is smart, responds to your commands without training, and even takes pictures for you! You don’t even have to pick up after or buy kibble for this pet! This is a dream come true for those craving an animal companion but cannot afford the time, money, or maintenance required for real animals. Robot pets are one example of how technology can assist in scenarios where companionship and fun are needed, but the handling and upkeep of real pets isn’t possible; especially in healthcare or senior homes.

Apart from doing normal pet things such as moving around and performing a few tricks, it can also perform robot tasks, like capturing photos with the camera in its nose. It can also remember you and follow you around for as long as you like. You can also deactivate this pet for some privacy and activate it back again when you feel like you need its company.

Through its camera “eyes”, this robot pet uses a combination of machine learning and computer vision techniques to see and respond to commands given through its owner’s body gestures. The robot pet can track, recognize, and perform a task such as following the user. The user is thus able to control the pet from a distance, enabling full autonomy.



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
* [2 Servo Motors (SG90)](https://www.amazon.ca/gp/product/B07Z16DWGW/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1)
* [Krisdonia Power Bank](https://www.amazon.ca/Krisdonia-25000mAh-Portable-External-Macbooks/dp/B076GYGR6M/ref=sr_1_7?dchild=1&keywords=laptop%2Bpower%2Bbank&qid=1615936374&sr=8-7&th=1)
* [GL.iNet Router](https://www.gl-inet.com/products/gl-ar750/)

* Miscellaneous:
  * 3D printer
  * [Raspberry Pi Camera Ribbon Cable](https://www.amazon.ca/Raspberry-Camera-Ribbon-Cable-Module/dp/B0981DPWD9/ref=sr_1_1?crid=15721LNB17OI9&keywords=Raspberry+Pi+Zero+v1.3+Camera+Cable+Raspberry+Pi+Zero+v1.3+Camera+Cable+Raspberry+Pi+Zero+v1.3+Camera+Cable+Raspberry+Pi+Zero+v1.3+Camera+Cable+Raspberry+Pi+Zero+v1.3+Camera+Cable&qid=1650660288&s=industrial&sprefix=raspberry+pi+zero+v1+3+camera+cable+raspberry+pi+zero+v1+3+camera+cable+raspberry+pi+zero+v1+3+camera+cable+raspberry+pi+zero+v1+3+camera+cable+raspberry+pi+zero+v1+3+camera+cable%2Cindustrial%2C260&sr=1-1)
  * 2 Ethernet Cables
  * Breadboard Jumper Cables
  * Screws


<p align="center">
  <img width="300" src="/Images/final.jpg">
</p>

### Setup and Connection Notes
* Build robot base without using the Arduino or the ultrasonic sensors, you can find the instructions [here](https://drive.google.com/file/d/1nSlkYJ7oCfMkG1p-KDfVHdLQt3B4Nmo5/view)
* Connect robot base parts to Raspberry Pi (see pinout in RaspberryPi_robot_pet/server_v9.py)
* Connect Raspberry Pi Camera to CAMERA1 on the Atlas 200 DK, you can follow this [guide](https://support.huaweicloud.com/intl/en-us/qs-atlas200dkappc32/atlased_04_0006.html)
* Set the output voltage of the power bank to 12 V. (Hold the power button and then double click a few times.)
  * The Atlas Board can be connected to the "DC-Out" port; the router and the Pi can be connected to the USB ports.
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
  
### Robot Operating System (ROS) Setup

The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. In order to set up ROS on the Atlas 200 DK board refer to the following guide below: 

[ROS Setup](https://hiascend.notion.site/UBC-Capstone-ROS-on-Atlas200DK-0720a3605a354f36a9cdbb0ce885ddf2)

Once the conda environment and ROS has been set up on the board we have to create a workspace and a ROS project such that we can run our code. The following code describes how to do so:
* [ROS Create Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* [ROS Create Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
* [ROS Build Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

### Raspberry Pi Setup
1. Install the latest version of Raspbian onto an SD card  
   Follow the steps in the link:  
   https://www.raspberrypi.org/documentation/installation/installing-images/ 
2. Insert SD card into Pi and power on the Pi by plugging it into a 5V power source.
3. Make sure you complete the [Router connection](https://github.com/kylerhunag/gesture_controlled_robot_pet/wiki/Router-Connection-Setup-Guide)
4. Update the system by opening a terminal and entering:  
   sudo apt-get update                                                        
   sudo apt-get upgrade                                                        
   sudo apt-get install python3-pip
5. Retrieve the files to be downloaded onto the Raspberry Pi, which are located in the folder “RaspberryPi_robot_pet”, either from cloning this repository or from the GitHub repository directly. Move the following files onto the Raspberry Pi Desktop in a folder called“RaspberryPi_robot_pet”:
    * Raspberry-Pi_robot_pet/happy.mp4
    * Raspberry-Pi_robot_pet/neu.mp4
    * Raspberry-Pi_robot_pet/white.jpeg
    * Raspberry-Pi_robot_pet/body_img.jpg
    * Raspberry-Pi_robot_pet/camera_img.jpg
    * Raspberry-Pi_robot_pet/deactivate.jpg
    * Raspberry-Pi_robot_pet/follow_img.jpg
    * Raspberry-Pi_robot_pet/server_v9.py
    * Raspberry-Pi_robot_pet/requirements.txt
6. Navigate to the directory where the main Raspberry Pi program is located, for example:  
   `cd /home/pi/Documents/Capstone/Raspberry-Pi_robot_pet/`
7. Install OpenCV and numpy for Python 3:  
   `pip3 install -r requirements.txt`
8. Install the LCD driver  
   Follow instruction under the link:  
   https://github.com/goodtft/LCD-show 

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

### Raspberry Pi
1. Start Raspberry Pi into Raspbian OS
2. ssh into the Pi 
3. Navigate to the directory where the main Raspberry Pi program is located, for example:  
   `cd /home/pi/Documents/Capstone/Raspberry-Pi_robot_pet/`
4. Ensure that Pi and the Atlas 200DK are in the same network. Run the main program for the robot control:  
   `python3 server_v9.py`

### Atlas 200DK 
1. Login to Atlas 200 DK from PC (Refer to this [guide](https://hiascend.notion.site/Atlas-200-DK-Setup-Guide-070b907c3c124381bdd6721618b81ef8) on how to setup and access). Note, it is required to use `VScode` with `Remote-SSH` extension to login remotely, otherwise you might not get the video stream to display on your PC.

2. On Atlas 200 DK, git clone this repo 
    (No internet access? Just try connecting Atlas 200 Dk to a router with Ethernet Cable. For details, check [official document](https://support.huaweicloud.com/intl/en-us/environment-deployment-Atlas200DK1012/atlased_04_0012.html) or [our router connection guide](https://github.com/kylerhunag/gesture_controlled_robot_pet/wiki/Router-Connection-Setup-Guide)) 

    `git clone https://github.com/kylerhunag/gesture_controlled_robot_pet.git`
   
3. Navigate to the project root directory. Install the required dependencies to run this project:

    `cd ~/gesture_controlled_robot_pet/Atlas_robot_pet`
    
    `pip3 install -r requirements.txt`

4. Navigate to the presenter server directory and start the presenter server: 

    `cd presenterserver`
    
    `bash run_presenter_server.sh body_pose.conf`
    

### Robot Operating System (ROS) on 200DK

Once you have created and built your ROS package you can now copy paste the `scripts` and `launch` folder from `~/gesture_controlled_robot_pet/ROS` to `~/catkin_ws/src/<package_name>/`

Here we used robot_pet as our package_name.
Once all the scripts and launch files are copied into the catkin_ws we need to make the files executable and as such do 
`chmod +x <file name>` for all the files that have been copied. Before running the ROS project ensure that the presenter server script is running and the Raspberry PI script is running. 

Activate the environment you just created. Install the dependencies listed in the `requirements.txt`. And lastly run:
  
`cd ~/catkin_ws` 
  
`catkin_make` to build the nodes.

The ROS project can now be run. In order to run the project refer to the following set of commands: 
  
`cd ~/catkin_ws` 
  
`source ./devel/setup.bash`
  
`roslaunch <package_name> robot_pet_launch.launch`


# Third Party Links
* Body Pose Model: Sample Body Pose https://github.com/Atlas200dk/sample_bodypose
* Object Detection Model: FairMOT model https://github.com/HardysJin/atlas-track
* Raspbian image: https://www.raspberrypi.org/documentation/installation/installing-images/ 
* Presenter server & ROS: https://github.com/Ascend-Huawei/HiFly_Drone
* LCD driver: https://github.com/goodtft/LCD-show

