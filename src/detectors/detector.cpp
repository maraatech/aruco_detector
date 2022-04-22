#include "../../include/aruco_detector/detector.h"
#include <tf2/LinearMath/Quaternion.h>

//TODO Move all these functions to common or cares_lib like package - generally tidy them up...

void MarkerDetector::displayMarkers(Mat image, vector<int> ids, vector<vector<cv::Point2f> > marker_corners, std::string name){
  cv::aruco::drawDetectedMarkers(image, marker_corners, ids);
//  Mat dst;
//  cv::resize(image, dst, cv::Size(640, 480), 0, 0, INTER_CUBIC);
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 640, 480);
  cv::imshow(name, image);
  cv::waitKey(10);
}

void MarkerDetector::detectMarkers(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners){
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary, marker_corners, marker_ids, this->detector_params, rejected_markers_);
}

//TODO shift function somewhere useful
geometry_msgs::Pose MarkerDetector::calculatePose(geometry_msgs::Pose p_centre, geometry_msgs::Pose top_left, geometry_msgs::Pose top_right, geometry_msgs::Pose bot_right, geometry_msgs::Pose  bot_left){
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
  return pose;
}

std::map<int, geometry_msgs::Pose> MarkerDetector::processImage(Mat image,
                                                                sensor_msgs::CameraInfo camera_info,
                                                                double marker_size,
                                                                bool display) {
  vector<int> ids;
  vector<vector<cv::Point2f> > marker_corners;
  detectMarkers(image, ids, marker_corners);

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
      cv::drawFrameAxes(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.15);
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

std::map<int, geometry_msgs::Pose> MarkerDetector::processImage(Mat image,
                                                                Mat depth_image,
                                                                sensor_msgs::CameraInfo camera_info,
                                                                bool display, bool is_depth_in_meters) {
  vector<int> ids;
  vector<vector<cv::Point2f> > marker_corners;
  detectMarkers(image, ids, marker_corners);

  if(display) {
    displayMarkers(image, ids, marker_corners, "Output");
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
    geometry_msgs::Pose p_centre  = findDepthPoint(depth_image, camera_info, centre.x, centre.y, is_depth_in_meters);
    geometry_msgs::Pose top_left  = findDepthPoint(depth_image, camera_info, c1.x, c1.y, is_depth_in_meters);
    geometry_msgs::Pose top_right = findDepthPoint(depth_image, camera_info, c2.x, c2.y, is_depth_in_meters);
    geometry_msgs::Pose bot_right = findDepthPoint(depth_image, camera_info, c3.x, c3.y, is_depth_in_meters);
    geometry_msgs::Pose bot_left  = findDepthPoint(depth_image, camera_info, c4.x, c4.y, is_depth_in_meters);

    if (!isValid(top_left) || !isValid(top_right) || !isValid(bot_right) || !isValid(bot_left))
      continue;

    geometry_msgs::Pose pose = calculatePose(p_centre, top_left, top_right, bot_right, bot_left);
    poses[marker_id] = pose;
  }
  return poses;
}

std::map<int, geometry_msgs::Pose> MarkerDetector::processImages(Mat left_image,
                                                                 Mat right_image,
                                                                 cares_msgs::StereoCameraInfo stereo_info,
                                                                 bool display) {
  vector<int> left_ids;
  vector<vector<cv::Point2f> > left_corners;
  detectMarkers(left_image, left_ids, left_corners);

  vector<int> right_ids;
  vector<vector<cv::Point2f> > right_corners;
  detectMarkers(right_image, right_ids, right_corners);

  if(display) {
    displayMarkers(left_image, left_ids, left_corners, "Left Output");
    displayMarkers(right_image, right_ids, right_corners, "Right Output");
  }

  Mat Q = getQMatrix(stereo_info);

  std::map<int, geometry_msgs::Pose> poses;
  for(int l_i = 0; l_i < left_ids.size(); ++l_i){
    int marker_id = left_ids[l_i];

    int r_i = getIndex(marker_id, right_ids);
    if(r_i >= 0){
      vector<cv::Point2f> l_corners = left_corners[l_i];
      Point2f left_centre  = getMarkerCenter(l_corners);

      vector<cv::Point2f> r_corners = right_corners[r_i];
      Point2f right_centre = getMarkerCenter(r_corners);

      geometry_msgs::Pose p_centre  = calculateStereoPose(left_centre, right_centre, Q);
      geometry_msgs::Pose top_left  = calculateStereoPose(l_corners[0], r_corners[0], Q);
      geometry_msgs::Pose top_right = calculateStereoPose(l_corners[1], r_corners[1], Q);
      geometry_msgs::Pose bot_right = calculateStereoPose(l_corners[2], r_corners[2], Q);
      geometry_msgs::Pose bot_left  = calculateStereoPose(l_corners[3], r_corners[3], Q);

      if (!isValid(top_left) || !isValid(top_right) || !isValid(bot_right) || !isValid(bot_left)){
        continue;
      }

      geometry_msgs::Pose pose = calculatePose(p_centre, top_left, top_right, bot_right, bot_left);
      poses[marker_id] = pose;
    }
  }
  return poses;
}

