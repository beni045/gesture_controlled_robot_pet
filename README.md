# Hand Gesture Controlled Robot Pet
Consider having a pet that is smart, responds to your commands without training, and even takes pictures for you! You don’t even have to pick up after or buy kibble for this pet! This is a dream come true for those craving an animal companion but cannot afford the time, money, or maintenance required for real animals. Robot pets are one example of how technology can assist in scenarios where companionship and fun are needed, but the handling and upkeep of real pets isn’t possible; especially in healthcare or senior homes. 

Apart from doing normal pet things such as moving around and performing a few tricks, it can also perform robot tasks, like capturing photos with the camera in its nose. It can also communicate its mood through animated eyes on a colored LCD screen mounted to its head.

Through its camera “eyes”, this robot pet uses a combination of machine learning and computer vision to see and respond to commands given through its owner’s hand gestures. The robot pet can track, recognize, and perform a task such as following the user while video capturing. The user is thus able to control the pet from a distance, enabling full autonomy.

<p align="center">
  <img width="300" src="/Images/robot.png">
</p>

# Hardware Required
* Huawei's Atlas 200DK
* Raspberry Pi (3+)
* Raspberry Pi Camera (Camera V2)
* Raspberry Pi LCD Screen (Kuman 3.5" Inch TFT LCD Display 480x320 RGB Pixels)
* ELEGOO UNO Robot Car Base
* Optional:
  * 3D printer
  * 2 Servo Motors (SG90)
* Miscellaneous:
  * Raspberry Pi Camera Extension Cable 
  * Ethernet Cable
  * Jumper Cables
  * Screws

<p align="center">
  <img width="300" src="/Images/final.jpg">
</p>

# Setup
### Computer
1. Download the VMWare image to your host PC (size ~11GB): Google drive link:  
   https://drive.google.com/drive/folders/1rPXsv9p3cD_MpEpi2lggdW43nLFCy4v_?usp=sharing                
   Download the following three files: ‘VM_C73.part1.rar’, ‘VM_C73.part2.rar’ ‘VM_C73.part3.rar’ 
3. Unzip ‘VM_C73.part1.rar’ (Part 2 and Part 3 will automatically be unzipped).
4. Download and install the latest version of the VMware Workstation Player through the following link:  
   https://www.vmware.com/ca/products/workstation-player/workstation-player-evaluation.html 

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

Python libraries used:
* Socket
* Sys
* Pickle
* Numpy
* Struct
* Zlib
* RPi.GPIO
* Time
* Os
* Math
* Threading
* datetime


# Notes
Power supply not solved
