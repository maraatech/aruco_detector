# Aruco Detector

ROS package for detecting aruco markers with RGB and RGBD cameras.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them.
Presumes you are using ~/catkin_ws as ROS workspace, if you are not make adjustments where required.

```
1) ROS Melodic (no known reason why future versions will not work at this stage, Kenitic works fine too)

2) Pull master version of maara_msgs
   a) cd ~/catkin_ws/src
   b) git clone https://github.com/maraatech/maara_msgs.git
   
3) Compile all libraries
   a) cd ~/catkin_ws
   b) catkin_make

4) sudo apt-get install ros-<dist>-tf-sensor-msgs 
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
  * Pose of each marker N: "camera_aruco_N"

###### Launch File
```
roslaunch aruco_detector detector.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <arg name="ns"            default="camera"/>
  <arg name="image"         default="color/image_raw"/>
  <arg name="markers"       default="markers"/>
  <arg name="camera_info"   default="color/camera_info"/>
  <arg name="marker_size"   default="0.0315"/>
  <arg name="display"       default="true"/>

  <group ns="$(arg ns)">
      <node name="detector_node" pkg="aruco_detector" type="detector_node" output="screen">
        <param name="image"         value="$(arg image)" />
        <param name="markers"       value="$(arg markers)"/>
        <param name="camera_info"   value="$(arg camera_info)" />
        <param name="marker_size"   value="$(arg marker_size)" />
        <param name="tf_prefix"     value="$(arg ns)" />
        <param name="display"       value="$(arg display)"/>
      </node>
   </group>
</launch>
```

Name space that the node operates in. 

```
<arg name="ns"            default="camera"/>
```

Image and camera info topics to subscribe to.
```
<arg name="image"         default="color/image_raw"/>
<arg name="camera_info"   default="color/camera_info"/>
```

Marker array publisher
```
<arg name="markers"       default="markers"/>
```

Size of the marker, length of the square marker in mm

```
<param name="marker_size"   value="0.0315" />
```

Display detection results on screen
```
<param name="display"       value="$(arg display)"/>
```

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
  * Pose of each marker N: "camera_aruco_N"

###### Launch File
```
roslaunch aruco_detector depth_detector.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <arg name="ns"            default="camera"/>
  <arg name="image"         default="color/image_raw"/>
  <arg name="markers"       default="markers"/>
  <arg name="depth_image"   default="aligned_depth_to_color/image_raw"/>
  <arg name="camera_info"   default="color/camera_info"/>
  <arg name="display"       default="true"/>

  <group ns="$(arg ns)">
      <node name="depth_detector_node" pkg="aruco_detector" type="depth_detector_node" output="screen">
        <param name="image"         value="$(arg image)" />
        <param name="markers"       value="$(arg markers)"/>
        <param name="depth_image"   value="$(arg depth_image)" />
        <param name="camera_info"   value="$(arg camera_info)" />
        <param name="tf_prefix"     value="$(arg ns)" />
        <param name="display"       value="$(arg display)"/>
      </node>
   </group>
</launch>
```

Name space that the node operates in. 

```
<arg name="ns"            default="camera"/>
```

RGB Image, Depth Image, and camera info topics to subscribe to.

```
<arg name="image"         default="color/image_raw"/>
<arg name="depth_image"   default="aligned_depth_to_color/image_raw"/>
<arg name="camera_info"   default="color/camera_info"/>
```

Marker array publisher
```
<arg name="markers"       default="markers"/>
```

Display detection results on screen
```
<param name="display"       value="$(arg display)"/>
```

### Stereo Detector

ROS node that will detect aruco markers in stereo RGB data and publish the pose transform relative to the left frame.

##### Subscribed Topics
Topic names are all default names, they can be changed via setting parameters in the launch file.

* sensor_msgs::Image
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
  * Pose of each marker N: "camera_aruco_N"

###### Launch File
```
roslaunch aruco_detector stereo_detector.launch
```

## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments

Ben and Chris for figuring out the pose geometry for the depth calculation
