
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

    - Source the setup script:
      ```bash
         source ~/catkin_ws/devel/setup.bash
      ```
      Now, we will launch the freenect example for depth registration, which allows you to obtain the point cloud with RGB data superimposed over it.
      ```bash
      roslaunch freenect_launch freenect.launch depth_registration:=true
      ```
      visualize the topics from Kinect on Rviz, open a new terminal and launch rviz.
      ```bash
         rviz
      ```
      Now need to setup some parameters on rviz to visualize the depth registration data.
      1.  In the ‘Global Options’ set the ‘Fixed Frame’ to ‘camera_link’.
      2.  Add ‘pointcloud2’ object and set the topic to ‘/camera/depth_registered/points’
      
      ![Screenshot from 2024-03-13 17-21-21](https://github.com/5h1v4m-0/ros-noetic-kinectv1-setup/assets/99700035/899d2cdd-3280-4b56-834b-41ee46b90b92)
      
      follow this step to change the code of kinect viewer
      ```bash
         cd ~/catkin_ws/src/iai_kinect2_opencv4/kinect2_viewer
         ls
         sudo gedit CMakeLists.txt 
      ```
       add this two line in 

      Now wait for a few seconds to get the points on display!
  5. **Refrences**
     
     https://aibegins.net/2020/11/22/give-your-next-robot-3d-vision-kinect-v1-with-ros-noetic/
     
     http://www.choitek.com/uploads/5/0/8/4/50842795/ros_kinect.pdf
     
     http://wiki.ros.org/ROS/Tutorials/CreatingPackage
     
     https://naman5.wordpress.com/2014/06/24/experimenting-with-kinect-using-opencv-python-and-open-kinect-libfreenect/
     
      

      
    
