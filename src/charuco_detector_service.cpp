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

#include "cares_msgs/CharucoDetect.h"

// #include "aruco_detector/CharucoDetect.h"

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

//StereoMarkerDetector detector;
MarkerDetector detector;
std::string tf_ns = "_aruco_";
ros::Publisher pub_marker_pose;
bool display = true;

/**
 * Convert ROS image format to OpenCV image format
 * @param msg ROS image
 * @return OpenCV Mat
 */
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
/**
HI CHRIS! 

I have mmoved the charuco service thing to its own file to avoid mess with other detector - 
try keep live nodes and services seperated.

Aruco_detector Service should be under cares_msgs or maara_msgs 
(given how currently setup probably maara_msgs but will move later)
**/
bool charucoCallback(cares_msgs::CharucoDetect::Request &req, cares_msgs::CharucoDetect::Response &res){
    auto left_ptr = boost::make_shared<sensor_msgs::Image>(req.left_image);
    auto right_ptr = boost::make_shared<sensor_msgs::Image>(req.right_image);
    cv::Mat image_left = convertToMat(left_ptr);
    cv::Mat image_right = convertToMat(right_ptr);
    ROS_INFO("starting detection");
    std::map<int, geometry_msgs::Pose> markers = detector.processCharucoImages(image_left, image_right, req.stereo_info, true);
    ROS_INFO("done detection");
    static tf2_ros::TransformBroadcaster br;
    for(auto marker_info : markers){
      auto marker = marker_info.second;
       geometry_msgs::TransformStamped transform;
      transform.header.stamp    = req.left_image.header.stamp;
      transform.header.frame_id = req.left_image.header.frame_id;
      transform.child_frame_id  = tf_ns+std::to_string(0);
      transform.transform.translation.x = marker.position.x/-1000.0;
      transform.transform.translation.y = marker.position.y/-1000.0;
      transform.transform.translation.z = marker.position.z/-1000.0;

      transform.transform.rotation.x = marker.orientation.x;
      transform.transform.rotation.y = marker.orientation.y;
      transform.transform.rotation.z = marker.orientation.z;
      transform.transform.rotation.w = marker.orientation.w;

      geometry_msgs::TransformStamped pose_tf;
      pose_tf.header.stamp    = ros::Time::now();
      pose_tf.header.frame_id = "stereo1/left";//req.left_image.header.frame_id;
      pose_tf.child_frame_id  = tf_ns+std::to_string(0);

      pose_tf.transform.translation.x = marker.position.x/-1000.0;
      pose_tf.transform.translation.y = marker.position.y/-1000.0;
      pose_tf.transform.translation.z = marker.position.z/-1000.0;

      pose_tf.transform.rotation.x = marker.orientation.x;
      pose_tf.transform.rotation.y = marker.orientation.y;
      pose_tf.transform.rotation.z = marker.orientation.z;
      pose_tf.transform.rotation.w = marker.orientation.w;

      br.sendTransform(pose_tf);

      res.transform = transform; 
      //poses.poses.push_back(marker);

    }
    return true;
}

int main(int argc, char *argv[]) {
  ///<Initialize ROS
  ros::init (argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::ServiceServer charuco_service = nh.advertiseService("charuco_detector", charucoCallback);
  ROS_INFO("Service ready to find aruco markers");

  ros::spin();
}
