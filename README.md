English | [Español](README_ES.md)
# Object Detection and 3D Localization with YOLOv8 in ROS
## Description
This project is based on yolov8_ros created by mgonzs13 and has been modified for use with ROS Noetic Ninjemyz. It is designed for object detection and localization using Ultralytics YOLOv8 with depth images.
The project has been tested with Kinect V1 and Kinect V2. For Kinect V2 usage, it was necessary to install the iai_kinect2 repository, which provides specific tools and libraries for integration with ROS.

https://github.com/CarlosVelasco210/DetObj_ROS/assets/69257527/91c92260-9149-4b85-bc3c-493b62194b34

## Environment:
- ROS Noetic Ninjemyz
## Requirements:
- Ultralytics YOLOv8
- iai_kinectV2
## Topics:
- /yolov8/dbg_image: Debug images showing detected and tracked objects. Can be visualized in Rviz.
- /yolov8/pub_tf: Publishes object position as a transform in 3D space. Allows visualization of object position in Rviz.
- /yolov8/detections: Objects detected by YOLOv8 using RGB images from the connected device. Each object contains a bounding box and class name.
## Parameters:
- yolo_model: YOLOv8 model to use. In this case, a custom-trained model was used for detecting the desired object.
- conf_thresh: Threshold setting for detection (Default value: 0.5)
- input_topic: Camera topic for RGB images
- depth_image: Topic providing depth images
- camera_depth_info: Camera topic providing depth-related information.
## Note:
The project is configured to be used with Kinect V2. If you wish to use your own device or camera, you can modify parameters in the yolo.launch file according to your requirements.
## Installation:
``` bash
cd <catkin_ws/src>
git clone https://github.com/CarlosVelasco210/DetObj_ROS.git
cd ~/catkin_ws
catkin_make
```
## Run:
```bash
roslaunch yolov8 yolo.launch
```
## Citation
```tex
@misc{iai_kinect2,
  author = {Wiedemeyer, Thiemo},
  title = {{IAI Kinect2}},
  organization = {Institute for Artificial Intelligence},
  address = {University Bremen},
  year = {2014 -- 2015},
  howpublished = {\url{https://github.com/code-iai/iai\_kinect2}},
  note = {Accessed June 12, 2015}
}
```
