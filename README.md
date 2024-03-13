
# Setup Kinect_V2 in ROS Noetic

This guide will walk you through the process of setting up Kinect_v2 in ROS Noetic


## Prerequisites

- Ubuntu 20.04

   ```bash
    lsb_release -a  #check ubuntu version
   ```
  ![Screenshot from 2024-03-13 16-30-56](https://github.com/5h1v4m-0/ros-noetic-kinectv1-setup/assets/99700035/586c59ec-9d29-469a-a443-7401c64cedb5)


- ROS Noetic installed ([installation instructions](http://wiki.ros.org/noetic/Installation))
  ```bash
  cd /opt/ros && ls #check ros distro
  ```
  ![Screenshot from 2024-03-13 16-41-35](https://github.com/5h1v4m-0/ros-noetic-kinectv1-setup/assets/99700035/2f59d81b-6be1-441f-bffd-183cfdfda13a)

- Kinect for Xbox one(Kinect v2)
-  OpenCV (version>=4)
   ```bash
     python3 -c "import cv2; print(cv2.__version__)" #check version
    ```



## Installation

1. **Update and upgrade.**:

 
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
2. **Install Dependencies**
   ```bash
   sudo apt install ros-noetic-rgbd-launch
   sudo apt-get install ros-noetic-openni-launch
   sudo apt-get install libfreenect-dev
   sudo apt-get install libusb-1.0-0-dev
   sudo apt-get install libturbojpeg0-dev
   sudo apt-get install build-essential cmake pkg-config
   sudo apt install libopencv-dev

   ```
3. **Install libfreenect2**

   This package is not available in the official ROS Noetic repositories, so we need to clone the GitHub repository and build it. Follow these steps to do so.

   - Clone the package and build
     ```bash
     cd ~
     git clone https://github.com/OpenKinect/libfreenect2.git
     cd libfreenect2
     mkdir build
     cd build
     cmake ..
     make
     sudo make install
     sudo ldconfig /usr/local/lib64/
     sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
     sudo udevadm control --reload-rules && udevadm trigger
     cd ~/libfreenect2/build/bin
     ./Protonect
     ```

       
 4. **Clone the iai_kinect2 package from GitHub**
    - Create a catkin workspace (if you haven't already):
      ```bash
         mkdir -p ~/catkin_ws/src
         cd ~/catkin_ws/src
      ```
    - clone and build
      ```bash
       git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git
       cd ~/catkin_ws
       rosdep install --from-paths src --ignore-src -r -y
       sudo rosdep init
       rosdep update
       catkin_make
      ```
      if you get this error in catkin make
      
      ![Screenshot from 2024-03-13 18-34-50](https://github.com/Shivam-Kumar-1/ros-noetic-kinect-v2_setup/assets/99700035/8d3b5c16-9c0e-483f-ae34-b2e98d816acd)

      follow this step to change the code of kinect viewer
      ```bash
         cd ~/catkin_ws/src/iai_kinect2_opencv4/kinect2_viewer
         ls
         sudo gedit CMakeLists.txt 
      ```
       add this two line in Cmakelist.txt file
        ```bash
        set(CMAKE_CXX_STANDARD 14)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        ```
        ![Screenshot from 2024-03-13 18-43-40](https://github.com/Shivam-Kumar-1/ros-noetic-kinect-v2_setup/assets/99700035/d36006b8-8c8b-4db3-8a39-198f251821cd)
      Now build
      ```bash
      cd ~/catkin_ws
      catkin_make
      ```



    - Source the setup script:
      ```bash
         source ~/catkin_ws/devel/setup.bash
      ```
    5. Launch 
      Now, we will launch the kinect2_bridge
      ```bash
      roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
      ```
      visualize the topics from Kinect on Rviz, open a new terminal and launch rviz.
      ```bash
         rviz
      ```
      
      Now need to setup some parameters on rviz to visualize the depth pointclod
      1.  In the ‘Global Options’ set the ‘Fixed Frame’ to ‘camera_link’.
      2.  Add ‘pointcloud2’ object and set the topic to ‘/kinect2/qhd/points’
      
      ![Screenshot from 2024-03-13 17-21-21](https://github.com/5h1v4m-0/ros-noetic-kinectv1-setup/assets/99700035/899d2cdd-3280-4b56-834b-41ee46b90b92)
      
      

      Now wait for a few seconds to get the points on display!
    
      for calibration see: ([kinect2_calibration](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration))
  
      

      
    
