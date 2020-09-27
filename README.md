# ROS-Object-Detection-2Dto3D-RealsenseD435
## Source code Version
[Object-Detection-and-location-RealsenseD435](https://github.com/Mazhichaoruya/Object-Detection-and-location-RealsenseD435.git)
## Requirements
Ubuntu18.04 OR 16.04  
ROS melodic(Has been Tested) Kinetic
[Opencv 4.x](https://github.com/opencv/opencv.git)  
C++ 11_std At least,I used the C++ 17 std  
Eigen3 :in absolutely Path /usr/local/eigen3  
PCL >=1.7.1,I used ros-melodic's default config PCL(1.8.1)    
[Intel Realsense SDK >=2.0 ](https://github.com/IntelRealSense/librealsense.git)  
[Yolov3 by Darknet](https://pjreddie.com/darknet/yolo/)  
## How to use
```Bash
git clone https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435.git
cd  ROS-Object-Detection-2Dto3D-RealsenseD435/Yolo_model
wget https://pjreddie.com/media/files/yolov3.weights ;wget https://pjreddie.com/media/files/yolov3-tiny.weights
```
For avoiding unreasonable troubles,I used the absolute path,so you have to change the path in src/Publisher_realsense.cpp   
on line 38-40
```cpp
String yolo_tiny_model ="/home/mzc/code/CLionProjects/DNN435/Yolo_model/yolov3.weights";
String yolo_tiny_cfg =  "/home/mzc/code/CLionProjects/DNN435/Yolo_model/yolov3.cfg";
String classname_path="/home/mzc/code/CLionProjects/DNN435/Yolo_model/object_detection_classes_yolov3.txt";
``` 
You can use your weight by Darknet or others supported by DNN too   
Next creat your ROS woekspace to us it.
```
cd ~
mkdir -p catkin_ws/src
cd ..
catkin_make
cd ~/ROS-Object-Detection-2Dto3D-RealsenseD435/
mv realsense_d435/ ~/catkin_ws/src
catkin_make
```
Attention:Default parameter on line 282 and 283 in src/Publisher_realsense.cpp   
```cpp
    net.setPreferableBackend(DNN_BACKEND_OPENCV);// DNN_BACKEND_INFERENCE_ENGINE DNN_BACKEND_CUDA
    net.setPreferableTarget(DNN_TARGET_CPU);//DNN_TARGET_CUDA
```
if you have IntelCore CPU you can chose "DNN_BACKEND_INFERENCE_ENGINE"to accelerate youe model--Openvino;<br>
But you should make sure your CPU is Intel and the Contrib of Opencv has been installed.  
If you have GPU(From Nvidia),You can Think about The Cuda acceleration.Before this you should reinstall Your Opencv(Version Most>4.2) with This:[OpenCV_DNN](https://medium.com/@sb.jaduniv/how-to-install-opencv-4-2-0-with-cuda-10-1-on-ubuntu-20-04-lts-focal-fossa-bdc034109df3)  
Open the Cuda Setting when CMake.
## Test 
New a Terminal on every command
```
roscore
rosrun realsense_d435_pub
rosrun realsense_d435_sub
rviz
```
Then You can see the Pointcloud By Rviz(Add Display By topic,Choose The Pointcloud2,and change the Fixed Frame as "Objections_Pointcloud")
## Example  
ROS version on 9-26:
The different color Point means different classes,such ad Person,mouse,book,Tv and soon on,We can see the 3D Point  in the camera coordinate system.
![Example1](https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435/blob/master/gif/realsense_1.gif)
![Example2](https://github.com/Mazhichaoruya/ROS-Object-Detection-2Dto3D-RealsenseD435/blob/master/gif/realsense_2.gif) 
## To be continue  
I will Take ROS-Pack with Lidar Slam and Tr to run on robot  next  week.
