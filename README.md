# camera_robot_calibration: short description

  This library was developed at ISIR lab, at the Sorbonne University with the sole objective of unifying the coordinate system between a 3D vision sensor and a robot. 

  Using ARUCO markers to find transformation matrix between kinect v1 or v2 cameras and the robot (BAXTER, or crustcrawler) thanks to forming and solving overdetermined  linear equation system. 
  
  One can't use this library directly to do the calibration, one needs to instatiate an instant of the "calibrator" defined in the library. This is done in the repository [test_camera_robot_calibration](https://github.com/robotsthatdream/test_camera_robot_calibration.git) so after installing this library into one's catkin_ws, one needs to install the other repository and use that one to do the calibration.

# Dependencies

  The library has been built to work under ROS environment, so one needs to install [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), and moreover it has been coded, installed and tested under ubuntu 14.04 trusty 64bit system. Beside these general requirements you need the following :
- Boost, OpenCV, tbb, Eigen3, b64, flann and ros indigo moveit.
- pcl 1.7.2 compiled with c++11.
- opencv_nonfree.
- camera_sdks.
- [aruco](https://github.com/robotsthatdream/aruco.git) : 

  This is a library I used to be able to recognize the QR codes that will be utilised later to achieve the calibration. So it isn't vital for the mehtod in general, if you another library you can install it but again you need to change the detection method inside the code.
- [baxter_sdks](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) : 

  One of the robots that this library is used with is the BAXTER robot so one needs to setup all BAXTER sdks in the catkin workspace to be able to get necessary information (mainly End Effector position) for the calibration to work.
- [crustcrawler_sdks](https://github.com/robotsthatdream/crustcrawler_dream.git) : 
  
  It is in-house developed sdks to deal with our customly built crustcrawler robot, and to integrate it with ROS. Again one needs to clone the whole thing within the catkin workspace and catkin_make install it.
- [image_processing](https://github.com/robotsthatdream/image_processing) : 
  
  This is an in-house built library by Leni Legoff to handle the vision processing part. So the received images from the vision sensor will be treated via this library along another one mentioned below, to give the exact point of the center of the marker used in the calibration. For this library you need to aske Dr. Stéphane Doncieux for access to the [robotsthatdream](https://github.com/robotsthatdream/image_processing) git repositories, or develop your own way of deducing the position "x, y, z" of the center of the marker.
- [rgbd_utils](https://project.isir.upmc.fr/redmine/projects/babbling_arm_exp/wiki/Install_instructions_for_the_dependencies) :

  Another in-house built library, it is part of redmine DREAM repository, you need to ask Dr. Stéphane for access to the repository "Babbling_arm_exp". This repository is a collection of useful packages one of them is *rgbd_utils*, so after cloning the repository you should put *rgbd_utils* in your catkin workspace and *catkin_make install* it.

# Install
- *[ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu).*
- *Boost, OpenCV, tbb, Eigen3, b64, flann and ros indigo moveit :*

  In a terminal enter the following :
```
:~$ sudo apt-get update
:~$ sudo apt-get install libboost-all-dev libopencv-dev libtbb-dev libeigen3-dev libb64-dev libflann-dev ros-indigo-moveit
```
- *pcl 1.7.2 :*

  Get the repository:
  ```
  :~$ wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
  ```
  
  Then add the following around line 22 of the CMakeLists.txt :
  ```
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  ```
  Then :
  ```
  :~$ cd pcl-pcl-1.7.2
  :~/pcl-pcl-1.7.2$ mkdir build && cd build
  :~/pcl-pcl-1.7.2/build$ cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..       (this will overwrite the current PCL installation)
  :~/pcl-pcl-1.7.2/build$ make -j8                                   (or -jn if you have n cores processors)
  :~/pcl-pcl-1.7.2/build$ sudo make install
  ```
- *opencv_nonfree :*
  In a terminal :
  ```
  :~$ sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
  :~$ sudo apt-get update
  :~$ sudo apt-get install libopencv-nonfree-dev
  ```
  
- *aruco :*

  Open a terminal and enter the following commands :
  ```
  :~$ git clone https://github.com/robotsthatdream/aruco.git
  :~$ cd aruco/
  :~/aruco$ mkdir build && cd build
  :~/aruco/build$ cmake ..
  :~/aruco/build$ make -j8               
  :~/aruco/build$ sudo make install      
  ```

- *camera_sdks :*
  
  If we are using *kinect v1* we should make sure to install *freenect* and/or *openni2* :
  ```
  :~$ source /opt/ros/indigo/setup.bash
  :~$ sudo apt-get install ros-indigo-freenect-*
  :~$ sudo apt-get install ros-indigo-openni2-*
  ```
  
  If we are to use *kinect v2* follow [these](https://github.com/code-iai/iai_kinect2) instructions.
  
- *[baxter_sdks](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).*

- *[crustcrawler_sdks](https://github.com/robotsthatdream/crustcrawler_dream.git) :*

  We assume your catkin workspace is in your home directory. In a terminal:
  ```
  :~$ cd ~/catkin_ws/src
  :~/catkin_ws/src$ git clone https://github.com/robotsthatdream/crustcrawler.git
  :~/catkin_ws/src$ cd ..
  :~/catkin_ws$ source devel/setup.bash
  :~/catkin_ws$ catkin_make install
  ```
  
- *image_processing :*

  In the terminal do the following :
  ```
  :~$ git clone git@github.com:robotsthatdream/image_processing.git
  :~$ cd image_processing
  :~/image_processing$ mkdir build && cd build
  :~/image_processing/build$ cmake .. && make -j && sudo make install
  ```

- *rgbd_utils :*
  
   Follow the following instructions in a terminal :
  ```
  :~$ git clone git clone https://project.isir.upmc.fr//git/babbling_arm_exp/                (You will need access from Dr.   Stéphane Doncioux)
  :~$ cd ~/catkin_ws/src
  :~/catkin_ws/src$ ln -s ~/babbling_arm_exp/rgbd_utils                                      (equivalent to copying the repository into src folder of catkin workspace)
  :~/catkin_ws/src$ cd ..
  :~/catkin_ws$ source devel/setup.bash
  :~/catkin_ws$ catkin_make -DCATKIN_WHITELIST_PACKAGES=rgbd_utils install
  ```
  
- *[camera_robot_calibration](https://github.com/robotsthatdream/camera_robot_calibration.git) :*
  
    In a terminal :
    ```
    :~$ cd ~/catkin_ws/src 
    :~/catkin_ws/src$ git clone https://github.com/robotsthatdream/camera_robot_calibration.git
    :~/catkin_ws/src$ cd ..
    :~/catkin_ws$ source devel/setup.bash
    :~/catkin_ws$ catkin_make -DCATKIN_WHITELIST_PACKAGES=rgbd_utils install
    ```
    
# Usage
  As indicated above, this repository serves as a class (named CALIBRATOR, check camera_robot_calibration.cpp in the repo) and one needs to code a node to instantiate the class and use it.
  
  This is done and explained in the repository *[test_camera_robot_calibration](https://github.com/robotsthatdream/test_camera_robot_calibration.git)*.
