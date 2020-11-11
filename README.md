# ROS-Object-Detection-2Dto3D-RealsenseD435
## Source code Version
[Object-Detection-and-location-RealsenseD435](https://github.com/Mazhichaoruya/Object-Detection-and-location-RealsenseD435.git)
## Object Dection with SLAM 
With SLAM(SC-Lego-LOAM)  ：  
[Perception-of-Autonomous-mobile-robot](https://github.com/Mazhichaoruya/Perception-of-Autonomous-mobile-robot)
## Requirements
Ubuntu18.04 OR 16.04  
ROS melodic(Has been Tested) Kinetic
C++ 11_std At least,I used the C++ 17 std  
Eigen3 :in absolutely Path /usr/local/eigen3  
PCL:I used ros-melodic's default config PCL(1.8.1)    
[Intel Realsense SDK >=2.0 ](https://github.com/IntelRealSense/librealsense.git)  
TensorRT: Go to [Object-Detection-and-location-RealsenseD435](https://github.com/Mazhichaoruya/Object-Detection-and-location-RealsenseD435.git) install the requirments of TensorRT.
## How to use
```Bash
git clone https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435.git
```
Next creat your ROS woekspace to us it.
```
cd ~
mkdir -p catkin_ws/src
cd ..
catkin_make
cd ~/ROS-Object-Detection-2Dto3D-RealsenseD435/
mv realsense_d435/ realsensen-ros/  ~/catkin_ws/src
catkin_make
mv engine/ ~/catkin_ws/src/realsense_d435
```
## Test 
```
roslaunch realsense_d435 run.launch
```
Then You can see the Pointcloud By Rviz(Add Display By topic,Choose The Pointcloud2)
## Example  
ROS version on 9-26:
The different color Point means different classes,such ad Person,mouse,book,Tv and soon on,We can see the 3D Point  in the camera coordinate system.
![Example1](https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435/blob/master/gif/realsense_1.gif)
![Example2](https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435/blob/master/gif/realsense_2.gif) 
With SLAM：  
[![Watch the video](https://github.com/Mazhichaoruya/Perception-of-Autonomous-mobile-robot/blob/master/image/image.png)](https://www.youtube.com/watch?v=VE7d3ZQzOLY&t=19s)
