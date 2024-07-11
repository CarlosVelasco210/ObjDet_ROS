cmake_minimum_required(VERSION 3.0.2)
project(yolov8)
set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    message_generation
    message_runtime
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
    vision_msgs
    visualization_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  YoloResult.msg
  Vector2.msg
  Point2D.msg
  Pose2D.msg
  Mask.msg
  KeyPoint3D.msg
  KeyPoint2D.msg
  BoundingBox2D.msg
  BoundingBox3D.msg
  Detection.msg
  DetectionArray.msg
  KeyPoint2DArray.msg
  KeyPoint3DArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  vision_msgs
)


catkin_package(
    CATKIN_DEPENDS
        cv_bridge
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_runtime
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf2_ros
        vision_msgs
        visualization_msgs
)

install(PROGRAMS
  script/tracker_node.py
  script/debug_node.py
  script/yolov8_node.py
  script/detect3d_node.py
  script/static_kinect.py
  script/transform_publish.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)