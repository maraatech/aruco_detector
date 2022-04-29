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
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cares_msgs/StereoCameraInfo.h>
#include "cares_msgs/ArucoDetect.h"

#include "../include/aruco_detector/common.h"
#include "../include/aruco_detector/parameters.h"
#include "../include/aruco_detector/detector.h"
#include "../include/aruco_detector/diamond_detector.h"
#include "../include/aruco_detector/charuco_detector.h"


using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

MarkerDetector* detector;
std::string tf_ns = "_aruco_";
ros::Publisher pub_marker_pose;
bool display = true;
bool is_depth_in_meters = true;
double marker_size = 0;

///< Service Callback Methods
bool monoService(cares_msgs::ArucoDetect::Request &request, cares_msgs::ArucoDetect::Response &response) {
  auto image_msg = boost::make_shared<sensor_msgs::Image>(request.image);
  cv::Mat image = convertToMat(image_msg);

  std::map<int, geometry_msgs::Pose> markers = detector->processImage(image, request.camera_info, marker_size, display);
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

bool depthService(cares_msgs::ArucoDetect::Request &request, cares_msgs::ArucoDetect::Response &response) {
  auto image_msg = boost::make_shared<sensor_msgs::Image>(request.image);
  auto depth_image_msg = boost::make_shared<sensor_msgs::Image>(request.depth_image);
  cv::Mat image = convertToMat(image_msg);
  cv::Mat depth_image = convertToMat(depth_image_msg);

  std::map<int, geometry_msgs::Pose> markers = detector->processImage(image, depth_image, request.camera_info, display, is_depth_in_meters);
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

bool stereoService(cares_msgs::ArucoDetect::Request &request, cares_msgs::ArucoDetect::Response &response){
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
    pose_tf.transform.rotation.x = marker.orientation.x;
    pose_tf.transform.rotation.y = marker.orientation.y;
    pose_tf.transform.rotation.z = marker.orientation.z;
    pose_tf.transform.rotation.w = marker.orientation.w;

    response.ids.push_back(id);
    response.transforms.push_back(pose_tf);
  }
  return true;
}

///< Callback Methods
void publishResults(std::map<int, geometry_msgs::Pose> markers, ros::Time time_stamp, std::string frame_id){
  geometry_msgs::PoseArray poses;
  poses.header.stamp    = time_stamp;
  poses.header.frame_id = frame_id;

  for(auto marker_info : markers){
    int id = marker_info.first;
    geometry_msgs::Pose marker = marker_info.second;
    poses.poses.push_back(marker);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped pose_tf;
    pose_tf.header.stamp    = time_stamp;
    pose_tf.header.frame_id = frame_id;
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

void monoCallback(const sensor_msgs::ImageConstPtr &image_msg,
                  const sensor_msgs::CameraInfoConstPtr &camera_info) {
  cv::Mat image = cares::convertToMat(image_msg);
  std::map<int, geometry_msgs::Pose> markers = detector->processImage(image, *camera_info, marker_size, display);

  publishResults(markers, image_msg->header.stamp, image_msg->header.frame_id);
}

void depthCallback(const sensor_msgs::ImageConstPtr &image_msg,
                   const sensor_msgs::ImageConstPtr &depth_image_msg,
                   const sensor_msgs::CameraInfoConstPtr &camera_info) {
  cv::Mat image       = cares::convertToMat(image_msg);
  cv::Mat depth_image = cares::convertToMat(depth_image_msg);
  std::map<int, geometry_msgs::Pose> markers = detector->processImage(image, depth_image, *camera_info, display, is_depth_in_meters);

  publishResults(markers, depth_image_msg->header.stamp, depth_image_msg->header.frame_id);
}

void stereoCallback(const sensor_msgs::ImageConstPtr &image_left_msg,
                    const sensor_msgs::ImageConstPtr &image_right_msg,
                    const cares_msgs::StereoCameraInfoConstPtr &stereo_camera_info){
  cv::Mat image_left = cares::convertToMat(image_left_msg);
  cv::Mat image_right = cares::convertToMat(image_right_msg);
  std::map<int, geometry_msgs::Pose> markers = detector->processImages(image_left, image_right, *stereo_camera_info, display);

  publishResults(markers, image_left_msg->header.stamp, image_left_msg->header.frame_id);
}

///< Service
void runService(std::string sensor_type){
  ros::NodeHandle nh;
  ros::ServiceServer aruco_service;
  std::string service_name = "/aruco_detector";//TODO parametise this name

  if(sensor_type.compare("mono") == 0) {
    aruco_service = nh.advertiseService(service_name, monoService);
  }
  else if(sensor_type.compare("stereo") == 0) {
    aruco_service = nh.advertiseService(service_name, stereoService);
  }
  else if(sensor_type.compare("depth") == 0) {
    aruco_service = nh.advertiseService(service_name, depthService);
  }
  ROS_INFO("Service ready to find aruco markers");
  ros::spin();
}

///< Sync Subscribers
void runMonoCallback(){
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string image, camera_info;
  if(!nh_private.getParam(cares::marker::IMAGE_S, image)){
    ROS_ERROR((cares::marker::IMAGE_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(image.c_str());
  if(!nh_private.getParam(cares::marker::CAMERA_INFO_S, camera_info)){
    ROS_ERROR((cares::marker::CAMERA_INFO_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(camera_info.c_str());

  if(!nh_private.getParam(cares::marker::MARKER_SIZE_D, marker_size)){
    ROS_ERROR((cares::marker::MARKER_SIZE_D + " not set.").c_str());
    exit(1);
  }
  marker_size /= 1000.0;
  ROS_INFO("Marker Size: %s",std::to_string(marker_size).c_str());

  Subscriber<sensor_msgs::Image> image_sub(nh, image, 1);
  Subscriber<sensor_msgs::CameraInfo> camera_info_sub(nh, camera_info, 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> sync_three(SyncPolicy(10), image_sub, camera_info_sub);
  sync_three.registerCallback(boost::bind(monoCallback, _1, _2));

  ros::spin();
}

void runDepthCallback(){
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string image, depth_image, camera_info;
  if(!nh_private.getParam(cares::marker::IMAGE_S, image)){
    ROS_ERROR((cares::marker::IMAGE_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(image.c_str());
  if(!nh_private.getParam(cares::marker::DEPTH_IMAGE_S, depth_image)){
    ROS_ERROR((cares::marker::DEPTH_IMAGE_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(depth_image.c_str());
  if(!nh_private.getParam(cares::marker::CAMERA_INFO_S, camera_info)){
    ROS_ERROR((cares::marker::CAMERA_INFO_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(camera_info.c_str());

  nh_private.param(cares::marker::IS_DEPTH_IN_METERS, is_depth_in_meters, is_depth_in_meters);
  std::string t = "Depth data expected in ";
  t += is_depth_in_meters ? "m" : "mm";
  ROS_INFO(t.c_str());

  Subscriber<sensor_msgs::Image> image_sub(nh, image, 1);
  Subscriber<sensor_msgs::Image> depth_image_sub(nh, depth_image, 1);
  Subscriber<sensor_msgs::CameraInfo> camera_info_sub(nh, camera_info, 1);

  typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> sync_three(SyncPolicy(10), image_sub, depth_image_sub, camera_info_sub);
  sync_three.registerCallback(boost::bind(depthCallback, _1, _2, _3));

  ros::spin();
}

void runStereoCallback(){
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string image_left, image_right, stereo_info;
  if(!nh_private.getParam(cares::marker::IMAGE_LEFT_S, image_left)){
    ROS_ERROR((cares::marker::IMAGE_LEFT_S + " not set.").c_str());
    exit(1);
  }
  if(!nh_private.getParam(cares::marker::IMAGE_RIGHT_S, image_right)){
    ROS_ERROR((cares::marker::IMAGE_RIGHT_S + " not set.").c_str());
    exit(1);
  }

  ROS_INFO(image_left.c_str());
  ROS_INFO(image_right.c_str());
  if(!nh_private.getParam(cares::marker::STEREO_INFO_S, stereo_info)){
    ROS_ERROR((cares::marker::STEREO_INFO_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(stereo_info.c_str());

  Subscriber<sensor_msgs::Image> image_left_sub(nh, image_left, 1);
  Subscriber<sensor_msgs::Image> image_right_sub(nh, image_right, 1);
  Subscriber<cares_msgs::StereoCameraInfo> camera_info_sub(nh, stereo_info, 1);

  typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, cares_msgs::StereoCameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), image_left_sub, image_right_sub, camera_info_sub);
  synchronizer.registerCallback(stereoCallback);

  ros::spin();
}

void getArucoDetector(int dictionary_id){
  detector = new MarkerDetector(dictionary_id);
}

void getCharucoDetector(int dictionary_id){
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

void getDiamondDetector(int dictionary_id){
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

///< Main
int main(int argc, char *argv[]) {
  ///<Initialize ROS
  ros::init (argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ///<Subscriber
  std::string marker, tf_prefix;
  if(!nh_private.getParam(cares::marker::MARKERS_S, marker)){
    ROS_ERROR((cares::marker::MARKERS_S + " not set.").c_str());
    exit(1);
  }
  ROS_INFO(marker.c_str());
  if(!nh_private.getParam(cares::marker::TF_PREFIX_S, tf_prefix)){
    ROS_ERROR((cares::marker::TF_PREFIX_S + " not set.").c_str());
    exit(1);
  }

  nh_private.param(cares::marker::DISPLAY_B, display, true);

  int dictionary_id = 0;
  nh_private.param(cares::marker::DICTIONARY_I, dictionary_id, dictionary_id);
  ROS_INFO("Using dictionary: %i", dictionary_id);

  std:string marker_type;
  nh_private.getParam("marker_type", marker_type);

  if(!nh_private.getParam(cares::marker::TF_PREFIX_S, tf_prefix)){
    ROS_ERROR((cares::marker::TF_PREFIX_S + " not set.").c_str());
    return 0;
  }
  tf_ns = tf_prefix+"_"+marker_type+"_";
  ROS_INFO(tf_ns.c_str());

  if(marker_type.compare("diamond") == 0)
    getDiamondDetector(dictionary_id);
  else if(marker_type.compare("charuco") == 0)
    getCharucoDetector(dictionary_id);
  else if(marker_type.compare("aruco") == 0)
    getArucoDetector(dictionary_id);
  else{
    ROS_ERROR("Marker Type: %s not found", marker_type.c_str());
    return 1;
  }

  pub_marker_pose = nh.advertise<geometry_msgs::PoseArray>(marker, 10);

  std::string sensor_type;
  nh_private.getParam("sensor_type", sensor_type);

  bool service = false;
  nh_private.param("service", service, service);

  if(service){
    runService(sensor_type);
  }
  else {
    if (sensor_type.compare("mono") == 0) {
      runMonoCallback();
    } else if (sensor_type.compare("stereo") == 0) {
      runStereoCallback();
    } else if (sensor_type.compare("depth") == 0) {
      runDepthCallback();
    }
  }
}
