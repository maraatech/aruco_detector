//
// Created by anyone on 18/02/20.
//

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/aruco/charuco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread.hpp>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include "cares_msgs/StereoCameraInfo.h"

#include "../include/aruco_detector/parameters.h"
#include "../include/aruco_detector/detector.h"
#include "../include/aruco_detector/diamond_detector.h"
#include "../include/aruco_detector/charuco_detector.h"

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

//StereoMarkerDetector detector;
MarkerDetector* detector;
std::string tf_ns = "_aruco_";
ros::Publisher pub_marker_pose;
bool display = true;

Mat convertToMat(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    //Get the time_start image in opencv Mat format
    Mat bgr_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

    //Convert RGB image to bgr if required
    if(msg->encoding == "rgb8"){//TODO put static type in here instead
      cvtColor(bgr_image, bgr_image, CV_RGB2BGR);
    }
    return bgr_image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(1);
  }
}

void callback(const sensor_msgs::ImageConstPtr &image_left_msg,
              const sensor_msgs::ImageConstPtr &image_right_msg,
              const cares_msgs::StereoCameraInfoConstPtr &stereo_camera_info){
  cv::Mat image_left = convertToMat(image_left_msg);
  cv::Mat image_right = convertToMat(image_right_msg);
  std::map<int, geometry_msgs::Pose> markers = detector->processImages(image_left, image_right, *stereo_camera_info, display);

  geometry_msgs::PoseArray poses;
  poses.header.stamp = image_left_msg->header.stamp;
  poses.header.frame_id = image_left_msg->header.frame_id;

  for(auto marker_info : markers){
    int id = marker_info.first;
    geometry_msgs::Pose marker = marker_info.second;
    poses.poses.push_back(marker);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped pose_tf;
    pose_tf.header.stamp    = image_left_msg->header.stamp;
    pose_tf.header.frame_id = image_left_msg->header.frame_id;
    pose_tf.child_frame_id  = tf_ns+std::to_string(id);

    pose_tf.transform.translation.x = marker.position.x;
    pose_tf.transform.translation.y = marker.position.y;
    pose_tf.transform.translation.z = marker.position.z;

    pose_tf.transform.rotation.x = marker.orientation.x;
    pose_tf.transform.rotation.y = marker.orientation.y;
    pose_tf.transform.rotation.z = marker.orientation.z;
    pose_tf.transform.rotation.w = marker.orientation.w;

    br.sendTransform(pose_tf);
  }
  pub_marker_pose.publish(poses);
}

void setArucoDetector(int dictionary_id){
  detector = new MarkerDetector(dictionary_id);
}

void setCharucoDetector(int dictionary_id){

  int board_width, board_height;
  double square_length, marker_length;

  ros::NodeHandle nh_private("~");
  if(!nh_private.getParam(cares::marker::BOARD_WIDTH_I, board_width)) {
    ROS_ERROR((cares::marker::BOARD_WIDTH_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::BOARD_HEIGHT_I, board_height)){
    ROS_ERROR((cares::marker::BOARD_HEIGHT_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::SQUARE_SIZE_D, square_length)){
    ROS_ERROR((cares::marker::SQUARE_SIZE_D + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::MARKER_SIZE_D, marker_length)){
    ROS_ERROR((cares::marker::MARKER_SIZE_D + " not set.").c_str());
    exit(1);
  }
  ROS_INFO("Board: %i %i", board_width, board_height);
  square_length /= 1000.0;
  marker_length /= 1000.0;
  ROS_INFO("Board Size: %f m %f m", square_length, marker_length);
  detector = new CharcuoDetector(dictionary_id, board_width, board_height, square_length, marker_length);
}

void setDiamondDetector(int dictionary_id){
//  centre_id       = 11;
//  top_left_id     = 6;
//  top_right_id    = 7;
//  bottom_left_id  = 15;
//  bottom_right_id = 16;
  int centre_id, top_left_id, top_right_id, bottom_left_id, bottom_right_id;

  ros::NodeHandle nh_private("~");
  if(!nh_private.getParam(cares::marker::CENTRE_I, centre_id)){
    ROS_ERROR((cares::marker::CENTRE_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::TOP_LEFT_I, top_left_id)){
    ROS_ERROR((cares::marker::TOP_LEFT_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::TOP_RIGHT_I, top_right_id)){
    ROS_ERROR((cares::marker::TOP_RIGHT_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::BOT_RIGHT_I, bottom_right_id)){
    ROS_ERROR((cares::marker::BOT_RIGHT_I + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::BOT_LEFT_I, bottom_left_id)){
    ROS_ERROR((cares::marker::BOT_LEFT_I + " not set.").c_str());
    exit(1);
  }
  ROS_INFO("C: %i TL: %i TR: %i BR: %i BL: %i", centre_id, top_left_id, top_right_id, bottom_right_id, bottom_left_id);
  detector = new DiamondDetector(dictionary_id, centre_id, top_left_id, top_right_id, bottom_right_id, bottom_left_id);
}

int main(int argc, char *argv[]) {
  ///<Initialize ROS
  ros::init (argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ///<Subscriber
  //Rail Status
  std::string image_left, image_right, marker, stereo_info, tf_prefix;
  if(!nh_private.getParam(cares::marker::IMAGE_LEFT_S, image_left)){
    ROS_ERROR((cares::marker::IMAGE_LEFT_S + " not set.").c_str());
    return 0;
  }
  if(!nh_private.getParam(cares::marker::IMAGE_RIGHT_S, image_right)){
    ROS_ERROR((cares::marker::IMAGE_RIGHT_S + " not set.").c_str());
    return 0;
  }
  if(!nh_private.getParam(cares::marker::MARKERS_S, marker)){
    ROS_ERROR((cares::marker::MARKERS_S + " not set.").c_str());
    return 0;
  }
  ROS_INFO(marker.c_str());
  ROS_INFO(image_left.c_str());
  ROS_INFO(image_right.c_str());
  if(!nh_private.getParam(cares::marker::STEREO_INFO_S, stereo_info)){
    ROS_ERROR((cares::marker::STEREO_INFO_S + " not set.").c_str());
    return 0;
  }
  ROS_INFO(stereo_info.c_str());
  if(!nh_private.getParam(cares::marker::TF_PREFIX_S, tf_prefix)){
    ROS_ERROR((cares::marker::TF_PREFIX_S + " not set.").c_str());
    return 0;
  }
  tf_ns = tf_prefix + tf_ns;
  ROS_INFO(tf_ns.c_str());
  nh_private.param(cares::marker::DISPLAY_B, display, true);

  int dictionary_id = 0;
  nh_private.param(cares::marker::DICTIONARY_I, dictionary_id, dictionary_id);
  ROS_INFO("Using dictionary: %i", dictionary_id);

  std:string type;
  nh_private.getParam("type", type);

  if(type.compare("diamond") == 0)
    setDiamondDetector(dictionary_id);
  else if(type.compare("charuco") == 0)
    setCharucoDetector(dictionary_id);
  else if(type.compare("aruco") == 0)
    setArucoDetector(dictionary_id);
  else{
    ROS_INFO("Type: %s not found", type.c_str());
    return 1;
  }

  Subscriber<sensor_msgs::Image> image_left_sub(nh, image_left, 1);
  Subscriber<sensor_msgs::Image> image_right_sub(nh, image_right, 1);
  Subscriber<cares_msgs::StereoCameraInfo> camera_info_sub(nh, stereo_info, 1);

  typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, cares_msgs::StereoCameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), image_left_sub, image_right_sub, camera_info_sub);
  synchronizer.registerCallback(callback);

  pub_marker_pose = nh.advertise<geometry_msgs::PoseArray>(marker, 10);
  ROS_INFO("Ready to find aruco markers");

  ros::spin();
}
