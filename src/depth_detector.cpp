
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

geometry_msgs::Pose crossProduct(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  geometry_msgs::Pose pose;
  pose.position.x = p1.position.y * p2.position.z - p1.position.z * p2.position.y;
  pose.position.y = p1.position.z * p2.position.x - p1.position.x * p2.position.z;
  pose.position.z = p1.position.x * p2.position.y - p1.position.y * p2.position.x;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose add(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
  geometry_msgs::Pose pose;
  pose.position.x = (p1.position.x + p2.position.x);
  pose.position.y = (p1.position.y + p2.position.y);
  pose.position.z = (p1.position.z + p2.position.z);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose divide(geometry_msgs::Pose p1, float p2){
  geometry_msgs::Pose pose;
  pose.position.x = p1.position.x/p2;
  pose.position.y = p1.position.y/p2;
  pose.position.z = p1.position.z/p2;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose norm(geometry_msgs::Pose p1){
  geometry_msgs::Pose pose;
    double length = sqrt(p1.position.x * p1.position.x + p1.position.y * p1.position.y + p1.position.z * p1.position.z);
    pose.position.x = p1.position.x / length;
    pose.position.y = p1.position.y / length;
    pose.position.z = p1.position.z / length;
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
  
    //find depth point for each corner + centre
    geometry_msgs::Pose p_centre  = findDepthPoint(depth_image, camera_info, centre.x, centre.y);
    geometry_msgs::Pose top_left  = findDepthPoint(depth_image, camera_info, c1.x, c1.y);
    geometry_msgs::Pose top_right = findDepthPoint(depth_image, camera_info, c2.x, c2.y);
    geometry_msgs::Pose bot_right = findDepthPoint(depth_image, camera_info, c3.x, c3.y);
    geometry_msgs::Pose bot_left  = findDepthPoint(depth_image, camera_info, c4.x, c4.y);

    if (!isValid(top_left) || !isValid(top_right) || !isValid(bot_right) || !isValid(bot_left))
      continue;

    //vectors that represent the edges of the square
    geometry_msgs::Pose top  = diff(top_right, top_left);
    geometry_msgs::Pose right = diff(top_right, bot_right);
    geometry_msgs::Pose left = diff(top_left, bot_left);
    geometry_msgs::Pose bottom  = diff(bot_right, bot_left);

    //calculate unit vector for X axis by averaging top and bottom edges and normalising 
    geometry_msgs::Pose X = ave(top, bottom);
    X = norm(X);

    //calculate unit vector for Y axis by averaging left and right edges and normalising
    geometry_msgs::Pose Y = ave(left, right);
    Y = norm(Y);
  
    //Z axis is the crossproduct of X and Y
    geometry_msgs::Pose Z = crossProduct(X, Y);

    geometry_msgs::Pose pose;

    //place origin at centre of marker
    pose.position.x = p_centre.position.x;
    pose.position.y = p_centre.position.y;
    pose.position.z = p_centre.position.z;

    //math to convert the rotation matrix to a quaternion
    //rotation matrix is
    //     [X.x Y.x Z.x ]
    // R = [X.y Y.y Z.y ]
    //     [X.z Y.z Z.z ]
    // Math stolen from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
//[row][col]

    //X Y Z should be rewritten into a 3x3 array 
    float trace = X.position.x + Y.position.y + Z.position.z;
      if( trace > 0 ) 
      {
        float s = 0.5f / sqrtf(trace+ 1.0f);
        pose.orientation.w = 0.25f / s;
        pose.orientation.x = (Y.position.z - Z.position.y ) * s;
        pose.orientation.y = (Z.position.x - X.position.z ) * s;
        pose.orientation.z = (X.position.y - Y.position.x ) * s;
      } 
      else 
      {
        if ( X.position.x > Y.position.y && X.position.x > Z.position.z ) 
        {
          float s = 2.0f * sqrtf( 1.0f + X.position.x - Y.position.y - Z.position.z);
          pose.orientation.w = (Y.position.z - Z.position.y ) / s;
          pose.orientation.x = 0.25f * s;
          pose.orientation.y = (Y.position.x + X.position.y ) / s;
          pose.orientation.z = (Z.position.x + X.position.z ) / s;
        } 
        else if (Y.position.y > Z.position.z) 
        {
          float s = 2.0f * sqrtf( 1.0f + Y.position.y - X.position.x - Z.position.z);
          pose.orientation.w = (Z.position.x - X.position.z ) / s;
          pose.orientation.x = (Y.position.x + X.position.y ) / s;
          pose.orientation.y = 0.25f * s;
          pose.orientation.z = (Y.position.z + Z.position.y ) / s;
        } 
        else 
        {
          float s = 2.0f * sqrtf( 1.0f + Z.position.z - X.position.x - Y.position.y );
          pose.orientation.w = (X.position.y - Y.position.x ) / s;
          pose.orientation.x = (X.position.z + Z.position.x ) / s;
          pose.orientation.y = (Y.position.z + Z.position.y ) / s;
          pose.orientation.z = 0.25f * s;
        }
    }
  
    poses[marker_id] = pose;
  }
  return poses;
}


void DepthMarkerDetector::detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners) {
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary_, marker_corners, marker_ids, this->detector_params_, rejected_markers_);
}
