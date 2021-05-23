//
// Created by anyone on 1/03/17.
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

#include "../include/aruco_detector/detector.h"
#include "../include/aruco_detector/parameters.h"

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

MarkerDetector detector(0);
std::string tf_ns = "_aruco_";
ros::Publisher pub_marker_pose;
bool display = true;
double marker_size;

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

void callback(const sensor_msgs::ImageConstPtr &image_msg,
              const sensor_msgs::CameraInfoConstPtr &camera_info){
  cv::Mat image = convertToMat(image_msg);
//  std::map<int, std::vector<cv::Point> > markers = detector.processImage(image);
  std::map<int, geometry_msgs::Pose> markers = detector.processImage(image, *camera_info, marker_size,display);

  geometry_msgs::PoseArray poses;
  poses.header.stamp = image_msg->header.stamp;
  poses.header.frame_id = image_msg->header.frame_id;
  for(auto marker_info : markers){
    int id = marker_info.first;
    geometry_msgs::Pose marker = marker_info.second;
    poses.poses.push_back(marker);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped pose_tf;
    pose_tf.header.stamp    = image_msg->header.stamp;
    pose_tf.header.frame_id = image_msg->header.frame_id;
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

int main(int argc, char *argv[]) {
  ///<Initialize ROS
  ros::init (argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ///<Subscriber
  //Rail Status
  std::string image, marker, camera_info, tf_prefix;
  if(!nh_private.getParam(cares::marker::IMAGE_S, image)){
    ROS_ERROR((cares::marker::IMAGE_S + " not set.").c_str());
    return 0;
  }
  if(!nh_private.getParam(cares::marker::MARKERS_S, marker)){
    ROS_ERROR((cares::marker::MARKERS_S + " not set.").c_str());
    return 0;
  }
  ROS_INFO(marker.c_str());
  ROS_INFO(image.c_str());
  if(!nh_private.getParam(cares::marker::CAMERA_INFO_S, camera_info)){
    ROS_ERROR((cares::marker::CAMERA_INFO_S + " not set.").c_str());
    return 0;
  }
  ROS_INFO(camera_info.c_str());
  if(!nh_private.getParam(cares::marker::MARKER_SIZE_D, marker_size)){
    ROS_ERROR((cares::marker::MARKER_SIZE_D + " not set.").c_str());
    return 0;
  }
  ROS_INFO(std::to_string(marker_size).c_str());
  if(!nh_private.getParam(cares::marker::TF_PREFIX_S, tf_prefix)){
    ROS_ERROR((cares::marker::TF_PREFIX_S + " not set.").c_str());
    return 0;
  }
  tf_ns = tf_prefix + tf_ns;
  ROS_INFO("TF namespace: %s",tf_ns.c_str());
  nh_private.param(cares::marker::DISPLAY_B, display, true);

  Subscriber<sensor_msgs::Image> image_sub(nh, image, 1);
  Subscriber<sensor_msgs::CameraInfo> camera_info_sub(nh, camera_info, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> sync_three(SyncPolicy(10), image_sub, camera_info_sub);
  sync_three.registerCallback(boost::bind(callback, _1, _2));

  pub_marker_pose = nh.advertise<geometry_msgs::PoseArray>(marker, 10);

  ROS_INFO("Ready to find aruco markers");

  ros::spin();
}
