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
#include <cares_msgs/StereoCameraInfo.h>

#include "../include/aruco_detector/parameters.h"
#include "../include/aruco_detector/detector.h"
#include "../include/aruco_detector/diamond_detector.h"
#include "cares_msgs/ArucoDetect.h"

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

MarkerDetector* detector;
std::string tf_ns = "_aruco_";
bool display = true;

Mat convertToMat(const sensor_msgs::ImageConstPtr& msg){
  try{
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

bool callback(cares_msgs::ArucoDetect::Request &request, cares_msgs::ArucoDetect::Response &response){
    auto left_ptr = boost::make_shared<sensor_msgs::Image>(request.left_image);
    auto right_ptr = boost::make_shared<sensor_msgs::Image>(request.right_image);
    cv::Mat image_left = convertToMat(left_ptr);
    cv::Mat image_right = convertToMat(right_ptr);

    std::map<int, geometry_msgs::Pose> markers = detector->processImages(image_left,
                                                                        image_right,
                                                                        request.stereo_info,
                                                                        display);

    for(auto marker_info : markers){
      int id = marker_info.first;
      geometry_msgs::Pose marker = marker_info.second;

      geometry_msgs::TransformStamped pose_tf;
      pose_tf.header.stamp    = request.left_image.header.stamp;
      pose_tf.header.frame_id = request.left_image.header.frame_id;
      pose_tf.child_frame_id  = tf_ns+std::to_string(id);

      pose_tf.transform.translation.x = marker.position.x;
      pose_tf.transform.translation.y = marker.position.y;
      pose_tf.transform.translation.z = marker.position.z;

      pose_tf.transform.rotation.x = marker.orientation.x;
      pose_tf.transform.rotation.y = marker.orientation.y;
      pose_tf.transform.rotation.z = marker.orientation.z;
      pose_tf.transform.rotation.w = marker.orientation.w;

      response.ids.push_back(id);
      response.transforms.push_back(pose_tf);
    }
    return true;
}

void setArucoDetector(int dictionary_id){
  detector = new MarkerDetector(dictionary_id);
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
  ros::init (argc, argv, "detector_service");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string tf_prefix;
  if(!nh_private.getParam(cares::marker::TF_PREFIX_S, tf_prefix)){
    ROS_ERROR((cares::marker::TF_PREFIX_S + " not set.").c_str());
    return 0;
  }
  tf_ns = tf_prefix + tf_ns;
  ROS_INFO(tf_ns.c_str());

  int dictionary_id = 0;
  nh_private.param(cares::marker::DICTIONARY_I, dictionary_id, dictionary_id);
  ROS_INFO("Using dictionary: %i", dictionary_id);
  if(nh_private.hasParam(cares::marker::CENTRE_I)){
    setDiamondDetector(dictionary_id);
  }
  else{
    setArucoDetector(dictionary_id);
  }

  nh_private.param(cares::marker::DISPLAY_B, display, true);

  ros::ServiceServer aruco_service = nh.advertiseService("aruco_detector", callback);
  ROS_INFO("Service ready to find aruco markers");

  ros::spin();
}
