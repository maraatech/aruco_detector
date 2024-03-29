# Aruco Detector

ROS package for detecting aruco markers with RGB and RGBD cameras.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them.
Presumes you are using ~/catkin_ws as ROS workspace, if you are not make adjustments where required.

```
1) ROS Noetic

2) Install master version of cares_msgs: https://github.com/UoA-CARES/cares_msgs.git
   
3) Install tf2 ros messages: sudo apt-get install ros-noetic-tf2-sensor-msgs
```

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumed here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/maraatech/aruco_detector.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running the tests

Tests to be added

## Applications

### Marker Types Support
These package supports detection of the three marker types:
* Aruco: mono, stereo, depth
* Diamond Aruco: stereo, depth
* Charuo: mono

### RGB Detector
ROS node that will detect aruco markers in an RGB image and publish the pose transform relative to the color frame. 

##### Subscribed Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* sensor_msgs::Image
  * RGB image: /camera/color/image_raw
  * Depth aligned image: /camera/aligned_depth_to_color/image_raw
* sensor_msgs::CameraInfo
  * Camera info: camera/color/camera_info

##### Published Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* geometry_msgs::PoseArray
  * Marker Poses : markers

##### Broadcast Transforms
Broadcast pose of the markers
* geometry_msgs::TransformStamped
  * Pose of each marker N: "ns_MARKER_TYPE_N"

### Depth Detector
ROS node that will detect aruco markers in RGBD data and publish the pose transform relative to the depth frame.

##### Subscribed Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* sensor_msgs::Image
  * RGB image: /camera/color/image_raw
  * Depth aligned image: /camera/aligned_depth_to_color/image_raw
* sensor_msgs::CameraInfo
  * Camera info: camera/color/camera_info

##### Published Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* geometry_msgs::PoseArray
  * Marker Poses : markers

##### Broadcast Transforms
Broadcast pose of the markers
* geometry_msgs::TransformStamped
  * Pose of each marker N: "ns_MARKER_TYPE_N"

### Stereo Detector
ROS node that will detect aruco markers in stereo RGB data (undistorted currently) and publish the pose transform relative to the left frame.
Node can also be run as an aruco diamond detector.

##### Subscribed Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* sensor_msgs::Image - NOTE must be rectified images
  * Left RGB image: /camera/color/image_raw
  * Right RGB image: /camera/color/image_raw
* maara_msgs::StereoInfo
  * Stereo Info: /stereo/stereo_info

##### Published Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* geometry_msgs::PoseArray
  * Marker Poses : markers

##### Broadcast Transforms
Broadcast pose of the markers
* geometry_msgs::TransformStamped
  * Pose of each marker N: "ns_MARKER_TYPE_N"


### Marker Detector Services
Stereo and depth camera based marker detection can be run as a service as well.

#### Service Message

* cares_msgs/ArucoDetect.srv


## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments

Ben and Chris for figuring out the pose geometry for the depth calculation
