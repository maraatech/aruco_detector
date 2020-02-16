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

#### RGB Detector

ROS node that will detect aruco markers in an RGB image and publish the pose transform relative to the color frame. 

###### Launch File
```
roslaunch aruco_detector detector.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <node name="detector_node" pkg="aruco_detector" type="detector_node" output="screen">
    <param name="image"         value="camera/color/image_raw" />
    <param name="camera_info"   value="camera/color/camera_info" />
    <param name="marker_size"   value="0.0315" />
  </node>
</launch>
```

Image and camera info topics to subscribe to.

```
<param name="image" value="camera/color/image_raw" />
<param name="camera_info"   value="camera/color/camera_info" />
```

Size of the marker, length of the square marker in mm

```
<param name="marker_size"   value="0.0315" />
```

#### Depth Detector

ROS node that will detect aruco markers in RGBD data and publish the pose transform relative to the depth frame.

###### Launch File
```
roslaunch aruco_detector detector.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <node name="depth_detector_node" pkg="aruco_detector" type="depth_detector_node" output="screen">
    <param name="image"         value="camera/color/image_raw" />
    <param name="depth_image"   value="camera/aligned_depth_to_color/image_raw" />
    <param name="camera_info"   value="camera/color/camera_info" />
  </node>
</launch>
```

RGB Image, Depth Image, and camera info topics to subscribe to.

```
<param name="image" value="camera/color/image_raw" />
<param name="depth_image"   value="camera/aligned_depth_to_color/image_raw" />
<param name="camera_info"   value="camera/color/camera_info" />
```

## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments

Ben and Chris for figuring out the pose geometry for the depth calculation