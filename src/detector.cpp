//
// Created by anyone on 12/02/20.
//

#include <tf2/LinearMath/Quaternion.h>
#include "../include/aruco_detector/detector.h"

MarkerDetector::MarkerDetector()
{
//  this->dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
  this->dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  this->detector_params_ = aruco::DetectorParameters::create();
}

void MarkerDetector::detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners)
{
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary_, marker_corners, marker_ids, this->detector_params_, rejected_markers_);
}

cv::Mat getQMatrix(const maara_msgs::StereoCameraInfo stereo_info) {
  cv::Mat Q(4, 4, CV_64FC1, (void *) stereo_info.Q.data());
  return Q;
}

cv::Mat getCameraMatrix(const sensor_msgs::CameraInfo camera_info) {
  cv::Mat camera_matrix(3, 3, CV_64FC1, (void *) camera_info.K.data());
  return camera_matrix;
}

cv::Mat getDistCoef(const sensor_msgs::CameraInfo camera_info) {
  cv::Mat camera_matrix(1, 5, CV_64FC1, (void *) camera_info.D.data());
  return camera_matrix;
}

Matx41d aa2quaternion(const Matx31d& aa)
{
  double angle = norm(aa);
  Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
  double angle_2 = angle / 2;
  //qx, qy, qz, qw
  Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
  return q;
}

void displayResult(Mat image, vector<int> ids, vector<vector<cv::Point2f> > marker_corners, std::string name){
  cv::aruco::drawDetectedMarkers(image, marker_corners, ids);
  cv::imshow(name, image);
  cv::waitKey(10);
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

std::map<int, geometry_msgs::Pose> MarkerDetector::processImages(Mat image, sensor_msgs::CameraInfo camera_info, double marker_size,  bool display) {
  vector<int> ids;
  vector<vector<cv::Point2f> > marker_corners;
  detect(image, ids, marker_corners);

  cv::Mat camera_matrix, dist_coeffs;
  //This process uses the K camera matrix and D distortion vector from CameraInfo
  camera_matrix = getCameraMatrix(camera_info);
  dist_coeffs   = getDistCoef(camera_info);

  std::vector<cv::Vec3d> rvecs, tvecs;//Output
  cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);

  if(display) {
    Mat image_copy;
    image.copyTo(image_copy);
    cv::aruco::drawDetectedMarkers(image_copy, marker_corners, ids);
    for (int i = 0; i < ids.size(); i++) {
      cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05);
    }
    cv::imshow("Output", image_copy);
    cv::waitKey(10);
  }

  std::map<int, geometry_msgs::Pose> poses;

  for (int i = 0; i < ids.size(); i++) {
    int id = ids[i];
    Vec<double, 3> rotation    = rvecs[i];
    Vec<double, 3> translation = tvecs[i];

    geometry_msgs::Pose pose;
    pose.position.x = translation[0];
    pose.position.y = translation[1];
    pose.position.z = translation[2];

    Matx41d quaternion = aa2quaternion(rotation);
    pose.orientation.x = quaternion.val[0];
    pose.orientation.y = quaternion.val[1];
    pose.orientation.z = quaternion.val[2];
    pose.orientation.w = quaternion.val[3];

    poses[id] = pose;
  }

  return poses;
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

double length(geometry_msgs::Pose pose){
  double v = pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z;
  v = sqrt(v);
  return v;
}

geometry_msgs::Pose norm(geometry_msgs::Pose p1){
  geometry_msgs::Pose pose;
  double length = sqrt(p1.position.x * p1.position.x + p1.position.y * p1.position.y + p1.position.z * p1.position.z);
  pose.position.x = p1.position.x / length;
  pose.position.y = p1.position.y / length;
  pose.position.z = p1.position.z / length;
  pose.orientation.w = 1.0;
//  double q_length = length(p1);

  return pose;
}

bool isValid(geometry_msgs::Pose pose){
  return pose.position.z > 0;
}

std::map<int, geometry_msgs::Pose> MarkerDetector::processImage(Mat image, Mat depth_image, sensor_msgs::CameraInfo camera_info,  bool display) {
  vector<int> ids;
  vector<vector<cv::Point2f> > marker_corners;
  detect(image, ids, marker_corners);

  if(display) {
    displayResult(image, ids, marker_corners, "Output");
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

    //vectors that represent the direction of the edges of the square
    geometry_msgs::Pose top     = diff(top_right, top_left);
    geometry_msgs::Pose right   = diff(top_right, bot_right);
    geometry_msgs::Pose left    = diff(top_left, bot_left);
    geometry_msgs::Pose bottom  = diff(bot_right, bot_left);

    //normalise
    top    = norm(top);
    right  = norm(right);
    left   = norm(left);
    bottom = norm(bottom);

    //calculate unit vector for X axis by averaging top and bottom edges and normalising
    geometry_msgs::Pose X = ave(top, bottom);
    X = norm(X);
    //calculate unit vector for Y axis by averaging left and right edges and normalising
    geometry_msgs::Pose Y = ave(left, right);
    Y = norm(Y);
    //Z axis is the crossproduct of X and Y
    geometry_msgs::Pose Z = crossProduct(X, Y);
//    Z = norm(Z);

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
    double trace = X.position.x + Y.position.y + Z.position.z;
    if( trace > 0 )
    {
      double s = 0.5f / sqrtf(trace+ 1.0f);
      pose.orientation.w = 0.25f / s;
      pose.orientation.x = (Y.position.z - Z.position.y ) * s;
      pose.orientation.y = (Z.position.x - X.position.z ) * s;
      pose.orientation.z = (X.position.y - Y.position.x ) * s;
    }
    else
    {
      if ( X.position.x > Y.position.y && X.position.x > Z.position.z )
      {
        double s = 2.0f * sqrtf( 1.0f + X.position.x - Y.position.y - Z.position.z);
        pose.orientation.w = (Y.position.z - Z.position.y ) / s;
        pose.orientation.x = 0.25f * s;
        pose.orientation.y = (Y.position.x + X.position.y ) / s;
        pose.orientation.z = (Z.position.x + X.position.z ) / s;
      }
      else if (Y.position.y > Z.position.z)
      {
        double s = 2.0f * sqrtf( 1.0f + Y.position.y - X.position.x - Z.position.z);
        pose.orientation.w = (Z.position.x - X.position.z ) / s;
        pose.orientation.x = (Y.position.x + X.position.y ) / s;
        pose.orientation.y = 0.25f * s;
        pose.orientation.z = (Y.position.z + Z.position.y ) / s;
      }
      else
      {
        double s = 2.0f * sqrtf( 1.0f + Z.position.z - X.position.x - Y.position.y );
        pose.orientation.w = (X.position.y - Y.position.x ) / s;
        pose.orientation.x = (X.position.z + Z.position.x ) / s;
        pose.orientation.y = (Y.position.z + Z.position.y ) / s;
        pose.orientation.z = 0.25f * s;
      }
    }
    double l = length(pose);
    pose.orientation.w = pose.orientation.w / l;
    pose.orientation.x = pose.orientation.x / l;
    pose.orientation.y = pose.orientation.y / l;
    pose.orientation.z = pose.orientation.z / l;

    poses[marker_id] = pose;
  }
  return poses;
}

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

int getIndex(int id, std::vector<int> &point_ids){
  for(int i =0;i<point_ids.size();i++){
    if(point_ids[i]==id){
      return i;
    }
  }
  return -1;
}

//std::vector<KeyPoint> undistortRight(const std::vector<KeyPoint>& p) {
//  if(p.empty()) return std::vector<KeyPoint>();
//  Mat points(p.size(),1, CV_32FC2);
//  for(int i = 0; i < p.size(); i++) {
//    points.at<Vec2f>(i)[0] = float(p[i].pt.x);
//    points.at<Vec2f>(i)[1] = float(p[i].pt.y);
//  }
//  Mat res(p.size(),1, CV_32FC2);
//  undistortPoints(points, res, M2_, D2_, R2_, P2_);
//  std::vector<KeyPoint> result;
//  result.reserve(p.size());
//  for(int i = 0; i < p.size(); i++) {
//    Vec2f undist = res.at<Vec2f>(i);
//    result.push_back(p[i]);
//    result[i].pt = Point(undist[0], undist[1]);
//  }
//  return result;
//}
//
//std::vector<KeyPoint> undistortLeft(const std::vector<KeyPoint >& p) {
//  if(p.empty()) return std::vector<KeyPoint >();
//  Mat points(p.size(),1, CV_32FC2);
//  for(int i = 0; i < p.size(); i++) {
//    points.at<Vec2f>(i)[0] = float(p[i].pt.x);
//    points.at<Vec2f>(i)[1] = float(p[i].pt.y);
//  }
//  Mat res(p.size(),1, CV_32FC2);
//  undistortPoints(points, res, M1_, D1_, R1_, P1_);
//  std::vector<KeyPoint > result;
//  result.reserve(p.size());
//  for(int i = 0; i < p.size(); i++) {
//    Vec2f undist = res.at<Vec2f>(i);
//    result.push_back(p[i]);
//    result[i].pt = Point(undist[0], undist[1]);
//  }
//  return result;
//}

std::vector<Point2f> undistortPoints(const std::vector<Point2f >& p, Mat M, Mat D, Mat R, Mat P) {
  if(p.empty())
    return std::vector<Point2f >();
  Mat points(p.size(),1, CV_32FC2);
  for(int i = 0; i < p.size(); i++) {
    points.at<Vec2f>(i)[0] = float(p[i].x);
    points.at<Vec2f>(i)[1] = float(p[i].y);
  }
  Mat res(p.size(),1, CV_32FC2);
  undistortPoints(points, res, M, D, R, P);
  std::vector<Point2f > result;
  result.reserve(p.size());
  for(int i = 0; i < p.size(); i++) {
    Vec2f undist = res.at<Vec2f>(i);
    result.push_back(p[i]);
    result[i] = Point(undist[0], undist[1]);
  }
  return result;
}

std::map<int, geometry_msgs::Pose> MarkerDetector::processImages(Mat left_image, Mat right_image, maara_msgs::StereoCameraInfo stereo_info, double marker_size, bool display) {
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
      Point2f left_centre  = getMarkerCenter(l_corners);

      vector<cv::Point2f> r_corners = right_corners[r_i];
      Point2f right_centre = getMarkerCenter(r_corners);

      /*
       * MAJOR STEP MISSING!
       *
       * MUST UNDISTORT THE POINTS BEFORE USING Q!!!
       *
       * Need StereoCameraInfo or undistorted image
       */
      //camera matrix, dist_coeffs, R1, P1
      Mat M1, D1, R1, P1;
      l_corners = undistortPoints(l_corners, M1, D1, R1, P1);
      left_centre = undistortPoints({left_centre}, M1, D1, R1, P1)[0];

      Mat M2, D2, R2, P2;
      r_corners = undistortPoints(r_corners, M2, D2, R2, P2);
      right_centre = undistortPoints({right_centre}, M2, D2, R2, P2)[0];


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

      geometry_msgs::Pose Z = crossProduct(X, Y);

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
      double trace = X.position.x + Y.position.y + Z.position.z;
      if( trace > 0 )
      {
        double s = 0.5f / sqrtf(trace+ 1.0f);
        pose.orientation.w = 0.25f / s;
        pose.orientation.x = (Y.position.z - Z.position.y ) * s;
        pose.orientation.y = (Z.position.x - X.position.z ) * s;
        pose.orientation.z = (X.position.y - Y.position.x ) * s;
      }
      else
      {
        if ( X.position.x > Y.position.y && X.position.x > Z.position.z )
        {
          double s = 2.0f * sqrtf( 1.0f + X.position.x - Y.position.y - Z.position.z);
          pose.orientation.w = (Y.position.z - Z.position.y ) / s;
          pose.orientation.x = 0.25f * s;
          pose.orientation.y = (Y.position.x + X.position.y ) / s;
          pose.orientation.z = (Z.position.x + X.position.z ) / s;
        }
        else if (Y.position.y > Z.position.z)
        {
          double s = 2.0f * sqrtf( 1.0f + Y.position.y - X.position.x - Z.position.z);
          pose.orientation.w = (Z.position.x - X.position.z ) / s;
          pose.orientation.x = (Y.position.x + X.position.y ) / s;
          pose.orientation.y = 0.25f * s;
          pose.orientation.z = (Y.position.z + Z.position.y ) / s;
        }
        else
        {
          double s = 2.0f * sqrtf( 1.0f + Z.position.z - X.position.x - Y.position.y );
          pose.orientation.w = (X.position.y - Y.position.x ) / s;
          pose.orientation.x = (X.position.z + Z.position.x ) / s;
          pose.orientation.y = (Y.position.z + Z.position.y ) / s;
          pose.orientation.z = 0.25f * s;
        }
      }
    }
  }

  if(display) {
    displayResult(left_image, left_ids, left_corners, "Left");
    displayResult(right_image, right_ids, right_corners, "Right");
  }

  return poses;
}


