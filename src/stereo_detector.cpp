//
// Created by anyone on 18/02/20.
//

#include "../include/aruco_detector/stereo_detector.h"
#include <tf2/LinearMath/Quaternion.h>

//Q: Disparity-to-depth mapping matrix in 4x4 row-major format
geometry_msgs::Pose calculatePose(Point left_point, Point right_point, Mat Q){
  //Note this is ripped directly from kiwibot_vision. Hard values are unknown as documentation is lacking
  //cout<< "left: x: " << lp.x <<" y: "<<lp.y<<endl;
  //cout<< "right: x: " << rp.x <<" y: "<<rp.y<<endl;
  cv::Mat coordmat = cv::Mat::zeros(4, 1, CV_64FC1);
  coordmat.at<double>(0, 0) = left_point.x;//*4
  coordmat.at<double>(0, 1) = left_point.y;// * 4
  coordmat.at<double>(0, 2) = (double) abs((left_point.x) - (right_point.x ));
  coordmat.at<double>(0, 3) = 1;
  //cout<<coordmat.at<double>(0, 0) << coordmat.at<double>(0, 1)<<
  //  coordmat.at<double>(0, 2)<< coordmat.at<double>(0, 3)<<endl;
  cv::Mat result = Q * coordmat;

  double x = (double) result.at<double>(0, 0) / result.at<double>(0, 3);
  double y = (double) result.at<double>(0, 1) / result.at<double>(0, 3);
  double z = (double) result.at<double>(0, 2) / result.at<double>(0, 3);

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  return pose;
}

Point getMarkerCenter(vector<Point2f> corners) {
  float x=0, y=0;
  for(int i=0;i<corners.size();i++)
  {
    x+=corners[i].x;
    y+=corners[i].y;
  }
  return Point(x/4,y/4);
}

cv::Mat getQMatrix(const maara_msgs::StereoCameraInfo stereo_info) {
  cv::Mat Q(4, 4, CV_64FC1, (void *) stereo_info.Q.data());
  return Q;
}

void displayResult(Mat image, vector<int> ids, vector<vector<cv::Point2f> > marker_corners, std::string name){
  cv::aruco::drawDetectedMarkers(image, marker_corners, ids);
  cv::imshow(name, image);
  cv::waitKey(10);
}

int getIndex(int id, std::vector<int> &point_ids){
  for(int i =0;i<point_ids.size();i++){
    if(point_ids[i]==id){
      return i;
    }
  }
  return -1;
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

std::map<int, geometry_msgs::Pose> StereoMarkerDetector::processImages(Mat left_image, Mat right_image, maara_msgs::StereoCameraInfo stereo_info, double marker_size, bool display) {
//  # Parse
//  camera_info_msg = CameraInfo()
//  camera_info_msg.width = calib_data["image_width"]
//  camera_info_msg.height = calib_data["image_height"]
//  camera_info_msg.K = calib_data["camera_matrix"]["data"]
//  camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
//  camera_info_msg.R = calib_data["rectification_matrix"]["data"]
//  camera_info_msg.P = calib_data["projection_matrix"]["data"]
//  camera_info_msg.distortion_model = calib_data["distortion_model"]
//  return camera_info_msg

  vector<int> left_ids;
  vector<vector<cv::Point2f> > left_corners;
  detect(left_image, left_ids, left_corners);

  vector<int> right_ids;
  vector<vector<cv::Point2f> > right_corners;
  detect(right_image, right_ids, right_corners);

  Mat Q = getQMatrix(stereo_info);

  std::map<int, geometry_msgs::Pose> poses;
  for(int l_i = 0; l_i < left_ids.size(); ++l_i){
    int marker_id = left_ids[l_i];

    int r_i = getIndex(l_i, right_ids);
    if(r_i > 0){
      vector<cv::Point2f> l_corners = left_corners[l_i];
      vector<cv::Point2f> r_corners = right_corners[r_i];

      Point left_centre  = getMarkerCenter(l_corners);
      Point right_centre = getMarkerCenter(r_corners);

      geometry_msgs::Pose p_centre  = calculatePose(left_centre, right_centre, Q);
      geometry_msgs::Pose top_left  = calculatePose(l_corners[0], r_corners[0], Q);
      geometry_msgs::Pose top_right = calculatePose(l_corners[1], r_corners[1], Q);
      geometry_msgs::Pose bot_right = calculatePose(l_corners[2], r_corners[2], Q);
      geometry_msgs::Pose bot_left  = calculatePose(l_corners[3], r_corners[3], Q);

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
  }

  if(display) {
    displayResult(left_image, left_ids, left_corners, "Left");
    displayResult(right_image, right_ids, right_corners, "Right");
  }

  return poses;
}

void StereoMarkerDetector::detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners) {
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary_, marker_corners, marker_ids, this->detector_params_, rejected_markers_);
}
