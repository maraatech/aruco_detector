//
// Created by anyone on 18/05/21.
//

#include "../include/aruco_detector/diamond_detector.h"

bool hasId(int id, std::map<int, geometry_msgs::Pose> &poses){
  return !(poses.find(id) == poses.end());
}

std::map<int, geometry_msgs::Pose> DiamondDetector::processImages(Mat left_image, Mat right_image, cares_msgs::StereoCameraInfo stereo_info, bool display) {

  std::map<int, geometry_msgs::Pose> poses = MarkerDetector::processImages(left_image, right_image, stereo_info, display);

  std::map<int, geometry_msgs::Pose> diamond_poses;

  if(hasId(this->centre_id, poses)
  && hasId(this->top_left_id, poses)
  && hasId(this->top_right_id, poses)
  && hasId(this->bottom_right_id, poses)
  && hasId(this->bottom_left_id, poses)){
    geometry_msgs::Pose centre       = poses[this->centre_id];
    geometry_msgs::Pose top_left     = poses[this->top_left_id];
    geometry_msgs::Pose top_right    = poses[this->top_right_id];
    geometry_msgs::Pose bottom_right = poses[this->bottom_right_id];
    geometry_msgs::Pose bottom_left  = poses[this->bottom_left_id];

    geometry_msgs::Pose diamond_pose = this->calculatePose(centre, top_left, top_right, bottom_right, bottom_left);
    diamond_poses[centre_id] = diamond_pose;
  }
  return diamond_poses;
}

std::map<int, geometry_msgs::Pose> DiamondDetector::processImage(Mat image, Mat depth_image, sensor_msgs::CameraInfo camera_info, bool display, bool is_depth_in_meters){
  std::map<int, geometry_msgs::Pose> poses = MarkerDetector::processImage(image, depth_image, camera_info, display, is_depth_in_meters);

  std::map<int, geometry_msgs::Pose> diamond_poses;

  if(hasId(this->centre_id, poses)
     && hasId(this->top_left_id, poses)
     && hasId(this->top_right_id, poses)
     && hasId(this->bottom_right_id, poses)
     && hasId(this->bottom_left_id, poses)){
    geometry_msgs::Pose centre       = poses[this->centre_id];
    geometry_msgs::Pose top_left     = poses[this->top_left_id];
    geometry_msgs::Pose top_right    = poses[this->top_right_id];
    geometry_msgs::Pose bottom_right = poses[this->bottom_right_id];
    geometry_msgs::Pose bottom_left  = poses[this->bottom_left_id];

    geometry_msgs::Pose diamond_pose = this->calculatePose(centre, top_left, top_right, bottom_right, bottom_left);
    diamond_poses[centre_id] = diamond_pose;
  }
  return diamond_poses;
}
