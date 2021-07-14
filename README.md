# RB5 Ball Follower Robot Using FASTCV 

## Introduction
This project is intended to build and deploy an ball follower robot application on the Qualcomm  RB5 Robotics platform that detects a ball object and track that detected ball using Qualcomm’s FastCV.

## Prerequisites
1. A Linux workstation with Ubuntu 18.04.
2. Install Android Platform tools (ADB, Fastboot). 
3. Download and install the [SDK Manager](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager).
4. [Flash](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/flash-images) the RB5 firmware image on to the board.
5. Setup the [Network](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/set-up-network).
6. Require python 3.6 or greater.
7. Setup invoke tool on  RB5.
8. Setup opencv for python on RB5.
9. The Turtlebot burger is assembled, operational and is connected to RB5.
10. Connect USB camera to RB5.

## Steps to install additional python libraries
### 1. Invoke tool 
Invoke tool is used to create python bindings for the fastcv function.
```sh
 python3 -m pip install invoke
```
### 2. OpenCV for Python
If we have FastCV, Then Why OpenCV? OpenCV for python is used for capturing video frames from a usb camera and also for video frames  processing.
```sh
python3 -m pip install --upgrade setuptools
python3 -m pip install --upgrade pip
python3 -m pip install opencv-python
```

## Steps to build and deploy RB5 Ball Follower Robot Application
1. Clone the  project repository from the github to RB5/Host System.
   ```sh
   git clone https://github.com/globaledgesoft/RB5-Ball-Follower-Robot-using-Qualcomm-FastCV.git
   ```
2. Building the FastCV Application on the Host System.
   1. Download FastCV SDK v1.7.1 for Linux Embedded using below link https://developer.qualcomm.com/software/fast-cv-sdk/tools
   2. Use below commands to install FastCV binary file    fastcv-installer-linuxembedded-1-7-1.bin. 
      ```sh
      chmod 777 fastcv-installer-linuxembedded-1-7-1.bin
      ./fastcv-installer-linuxembedded-1-7-1.bin
      ```
   3. Copy the static library of FastCV from libs provided in the fastcv sdk (fastcv-1-7-1_LinuxEmbedded\lib\64-bit\libfastcv.a)  to <ROOT_DIR_OF_PROJECT> .
   4. Copy fastcv.h from inc provided in the fastcv sdk (fastcv-1-7-1_LinuxEmbedded\inc\fastcv.h)  to <ROOT_DIR_OF_PROJECT> .
   5. Download the Linaro Cross Toolchain for aarch64 on x86_64 platform
https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/
   6. Extract the toolchain in <ROOT_DIR_OF_PROJECT>.
   7. Run the command given below in order to generate the shared library for ball_tracking on the host system.
      ```sh
      python3 build_shared_lib.py
      ```
3. Building the FastCV Application on the Qualcomm RB5 Robotics Platform.
   1. Clone the project to the RB5.
   2. Copy the fastcv.h in /usr/include inside the RB5 Platform from the host system.
   3. In the shell of RB5, navigate inside the ROOT DIR of Project.
   4. Run the command given below inside the ROOT DIR of the Project to compile the application & generate the Shared Library of Ball Tracking application.
      ```sh
      python3 build_sharedlib_on_rb5.py
      ```
      
## Installation of TurtleBot3 Package
For the setup we will be using the TurtleBot3 Burger, we need to install TurtleBot Packages for controlling the TurtleBot.
1. Setup the necessary packages by executing the following commands.
   ```sh 
   sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
   ```
2. Create a new directory for TurtleBot3.
   ```sh
   mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
   ```
3. Clone the necessary repositories and then access TurtleBot Folder
   ```sh
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
   git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
   cd ~/turtlebot3_ws/src/turtlebot3
   ```
4. Remove the folders that are not required for the current project.
   ```sh
   rm -r turtlebot3_cartographer turtlebot3_navigation2
   cd ~/turtlebot3_ws/
   ```
5. Source & Building the TurtleBot3 Setup file
   ```sh
   echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   colcon build --symlink-install --parallel-workers 1
   echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```
   
## Steps to flash ROS2 firmware into OpenCR
The Default firmware supports ROS 1. As we are using ROS Dashing (ROS 2 version), we need to upgrade OpenCR firmware.
1. Create a temp folder for Binaries
   ```sh
   mkdir /home/opencrbin/ && cd /home/opencrbin
   ```
2. Download the latest binaries & unzip
   ```sh
   wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
   tar -xjf ./opencr_update.tar.bz2
   ```
3. Set the OpenCR port & TurtleBot Model. Before flashing, check whether the ttyACM0 port exists.
   ```sh 
   export OPENCR_PORT=/dev/ttyACM0
   export OPENCR_MODEL=burger
   ```
4. Upload the latest firmware by following command:
   ```sh
   cd /home/opencrbin/opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
   ```

## Steps to Run RB5 Ball Follower Robot Application
1. Launch the ROS2 Turtlebot Burger, Please run the command below on the RB5.
   ```sh
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
2. Open another shell terminal & navigate to the ROOT DIR of the Project.
3. Run the command given below to launch the application
   ```sh
   python3 main.py
   ```
