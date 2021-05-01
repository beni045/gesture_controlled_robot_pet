# Hand Gesture Controlled Robot Pet
Consider having a pet that is smart, responds to your commands without training, and even takes pictures for you! You don’t even have to pick up after or buy kibble for this pet! This is a dream come true for those craving an animal companion but cannot afford the time, money, or maintenance required for real animals. Robot pets are one example of how technology can assist in scenarios where companionship and fun are needed, but the handling and upkeep of real pets isn’t possible; especially in healthcare or senior homes. 

Apart from doing normal pet things such as moving around and performing a few tricks, it can also perform robot tasks, like capturing photos with the camera in its nose. It can also communicate its mood through animated eyes on a colored LCD screen mounted to its head.

Through its camera “eyes”, this robot pet uses a combination of machine learning and computer vision to see and respond to commands given through its owner’s hand gestures. The robot pet can track, recognize, and perform a task such as following the user while video capturing. The user is thus able to control the pet from a distance, enabling full autonomy.

<p align="center">
  <img width="300" src="/Images/robot.png">
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
* Build robot base without using the Ardunio or the ultrasonic sensors
* Connect robot base parts to Raspberry Pi
* Raspberry Pi connected to Atlas 200DK via ethernet*

# Setup
### Computer
https://support.huaweicloud.com/intl/en-us/Atlas200DK202/

### Atlas 200DK Setup
1. Set up the Atlas 200 DK by following the steps in the link: https://support.huaweicloud.com/intl/en-us/usermanual-A200dk_3000/atlas200dk_02_0001.html
2. Start VMware Player and load the unzipped file:  
   VMware Player -> 'Open a Virtual Machine'  
   directory: VM_C73/ascend.vmx  
   (Choose 'I copied it', then select 'No' if VMware is looking for some storage; then, you will see the virtual machine ‘ascend’ added in VMware player)
4. Power on ’ascend’, and use following at the login page:  
   User: ascend  
   Password: ascend  
6. Connect the Atlas 200 DK to your computer via USB.
7. Retrieve the files to be downloaded onto the Atlas 200 DK, which are located in the folder “Atlas_robot_pet”, either from the source code folder provided or from the GitHub repository. Move these to the desktop of your VM.
8. Retrieve the three machine learning models from the folder “models”. Place them in the folder “/Atlas_robot_pet/model/” on the Desktop of your VM.
9. Open a terminal in the VM on your computer, and move the “Atlas_robot_pet” folder from your VM desktop to the Atlas 200 DK using secure copy protocol:  
   scp -r /home/ascend/Desktop/Atlas_robot_pet/  
   HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI_PROJECTS/  
   The password to SCP is Mind@123

### Raspberry Pi Setup
1. Install the latest version of Raspbian onto an SD card  
   Follow the steps in the link:  
   https://www.raspberrypi.org/documentation/installation/installing-images/ 
2. Insert SD card into Pi and power on the Pi by plugging it into a 5V power source.
3. Update the system by opening a terminal and entering:  
   sudo apt-get update                                                        
   sudo apt-get upgrade
5. Install OpenCV for Python 3:  
   sudo apt-get install python3-opencv
7. Install the LCD driver  
   Follow instruction under the link:  
   https://github.com/goodtft/LCD-show 
6. Retrieve the files to be downloaded onto the Raspberry Pi, which are located in the folder “Raspberry-Pi_robot_pet”, either from the source code folder provided or from the GitHub repository. Move the following files onto the Raspberry Pi Desktop in the folder “Raspberry-Pi_robot_pet”:
    * Raspberry-Pi_robot_pet/happy.mp4
    * Raspberry-Pi_robot_pet/neu.mp4
    * Raspberry-Pi_robot_pet/white.jpeg
    * Raspberry-Pi_robot_pet/robot_server.py

# Gestures
Below are the gestures included in the repository.

Command | Gesture | Description
------- | ------- | -----------
`Move Forwards` | <img src="/Images/gesture_example_1.png" width="250" height="250"> | Moves the robot forwards
`Move Backwards` | <img src="/Images/gesture_example_2.png" width="250" height="250"> | Moves the robot backwards
`Spin Around` | <img src="/Images/gesture_example_3.png" width="250" height="250"> | Spins the robot around
`Take a Picture` | <img src="/Images/gesture_example_4.png" width="250" height="250"> | Initiates the routine to take a photo

# Run the Robot

### Atlas 200DK 
1. Open a terminal on your setup VM (ascend), and navigate to the following directory:  
   cd /home/ascend/Desktop/Atlas_robot_pet/script/  
3. In the same terminal, run the presenter server:  
   bash ./run_presenter_server.sh
5. Open a second terminal on your setup VM (ascend), and SSH into the Atlas 200 DK:  
   ssh HwHiAiUser@192.168.1.2  
  (The IP address for USB connection of the Atlas 200 DK will normally be 192.168.1.2)  
  The default password to SSH is Mind@123.
4. Navigate to the folder which holds the main program of the application:  
   cd /home/HwHiAiUser/HIAI_PROJECTS/Atlas_robot_pet/code_live/
6. Run the main program on the Atlas 200 DK:  
   python3 main.py

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
