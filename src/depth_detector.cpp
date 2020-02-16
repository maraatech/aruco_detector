///home/anyone/catkin_ws/src/maara_pc_mapping/CMakeLists.txt
//
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <rosconsole/macros_generated.h>
#include "depth_detector.h"

Point DepthMarkerDetector::getMarkerCenter(vector<Point2f> corners) {
  float x=0, y=0;
  for(int i=0;i<corners.size();i++)
  {
    x+=corners[i].x;
    y+=corners[i].y;
  }
  return Point(x/4,y/4);
}

geometry_msgs::Pose findDepthPoint(cv::Mat depth_image, sensor_msgs::CameraInfo camera_info, int pixel_x, int pixel_y) {
  //#     [fx  0 cx]
  //# K = [ 0 fy cy]
  //#     [ 0  0  1]
  double z = depth_image.at<unsigned short>(pixel_y, pixel_x);

  double x = (pixel_x - camera_info.K.at(2)) / camera_info.K.at(0);
  double y = (pixel_y - camera_info.K.at(5)) / camera_info.K.at(4);

  x = x * z / 1000.0;//mm -> m
  y = y * z / 1000.0;//mm -> m
  z = z / 1000.0;//mm -> m

  geometry_msgs::Pose pose_goal;
  pose_goal.position.x = x;
  pose_goal.position.y = y;
  pose_goal.position.z = z;
  pose_goal.orientation.x = 0;
  pose_goal.orientation.y = 0;
  pose_goal.orientation.z = 0;
  pose_goal.orientation.w = 1;

  return pose_goal;
}

geometry_msgs::Pose diff(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  geometry_msgs::Pose pose;
  pose.position.x = p1.position.x - p2.position.x;
  pose.position.y = p1.position.y - p2.position.y;
  pose.position.z = p1.position.z - p2.position.z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose ave(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  geometry_msgs::Pose pose;
  pose.position.x = (p1.position.x + p2.position.x)/2.0;
  pose.position.y = (p1.position.y + p2.position.y)/2.0;
  pose.position.z = (p1.position.z + p2.position.z)/2.0;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose crossProdict(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  geometry_msgs::Pose pose;
  pose.position.x = p1.position.y * p2.position.z - p1.position.z * p2.position.y;
  pose.position.y = p1.position.z * p2.position.x - p1.position.x * p2.position.z;
  pose.position.z = p1.position.x * p2.position.y - p1.position.y * p2.position.x;
  pose.orientation.w = 1.0;
  return pose;
}

bool isValid(geometry_msgs::Pose pose){
  return pose.position.z > 0;
}

std::map<int, geometry_msgs::Pose> DepthMarkerDetector::processImage(Mat image, Mat depth_image, sensor_msgs::CameraInfo camera_info,  bool display) {
  vector<int> ids;
  vector<vector<cv::Point2f> > marker_corners;
  detect(image, ids, marker_corners);

  if(display) {
    Mat image_copy;
    image.copyTo(image_copy);
    cv::aruco::drawDetectedMarkers(image_copy, marker_corners, ids);
    cv::imshow("Output", image_copy);
    cv::waitKey(10);
  }

  std::map<int, geometry_msgs::Pose> poses;
  for (int index = 0; index < ids.size(); index++) {
    int marker_id = ids[index];

    cv::Point centre = getMarkerCenter(marker_corners[index]);
    cv::Point c1 = marker_corners[index][0];
    cv::Point c2 = marker_corners[index][1];
    cv::Point c3 = marker_corners[index][2];
    cv::Point c4 = marker_corners[index][3];

    geometry_msgs::Pose p_centre  = findDepthPoint(depth_image, camera_info, centre.x, centre.y);
    geometry_msgs::Pose top_left  = findDepthPoint(depth_image, camera_info, c1.x, c1.y);
    geometry_msgs::Pose top_right = findDepthPoint(depth_image, camera_info, c2.x, c2.y);
    geometry_msgs::Pose bot_right = findDepthPoint(depth_image, camera_info, c3.x, c3.y);
    geometry_msgs::Pose bot_left  = findDepthPoint(depth_image, camera_info, c4.x, c4.y);

    if (!isValid(top_left) || !isValid(top_right) || !isValid(bot_right) || !isValid(bot_left))
      continue;

    top_left  = diff(top_left, p_centre);
    top_right = diff(top_right, p_centre);
    bot_right = diff(bot_right, p_centre);
    bot_left  = diff(bot_left, p_centre);

    geometry_msgs::Pose v1 = top_left;
    geometry_msgs::Pose v2 = bot_left;

    geometry_msgs::Pose X = ave(v2, v1);
    double length_x = sqrt(X.position.x * X.position.x + X.position.y * X.position.y + X.position.z * X.position.z);
    X.position.x /= length_x;
    X.position.y /= length_x;
    X.position.z /= length_x;

    geometry_msgs::Pose v3 = top_left;
    geometry_msgs::Pose v4 = top_right;

    geometry_msgs::Pose Y = ave(v4, v3);
    double length_y = sqrt(Y.position.x * Y.position.x + Y.position.y * Y.position.y + Y.position.z * Y.position.z);
    Y.position.x /= length_y;
    Y.position.y /= length_y;
    Y.position.z /= length_y;

    geometry_msgs::Pose Z = crossProdict(X, Y);

    double roll  = atan2(-Z.position.y, Z.position.z);
    double pitch = asin(Z.position.x);
    double yaw   = atan2(-Y.position.x, X.position.x);

    geometry_msgs::Pose pose;

    pose.position.x = p_centre.position.x;
    pose.position.y = p_centre.position.y;
    pose.position.z = p_centre.position.z;

    tf2::Quaternion quaternion;
    //<This does not work as it is from fixed frame, where as we calculated the intrinsic frame
    ///<* @param roll Angle around X
    ///<* @param pitch Angle around Y
    ///<* @param yaw Angle around Z*/
//      quaternion.setRPY(roll, pitch, yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
    ///<We calculated roll as angle around x axis, which is what setrpy uses, but seteuler defines roll as angle around z - <3 Ben
    ///<We also calculated the rpy as intrinsic rotation
    ///<* @param yaw Angle around Y
    ///<* @param pitch Angle around X
    ///<* @param roll Angle around Z
    quaternion.setEuler(pitch, roll, yaw);  // Create this quaternion from angle around X/angle around Y/angle around Z (in radians)

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    poses[marker_id] = pose;
  }
  return poses;
}

void DepthMarkerDetector::detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners) {
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary_, marker_corners, marker_ids, this->detector_params_, rejected_markers_);
}
