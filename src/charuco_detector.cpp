//
// Created by anyone on 12/04/22.
//
#include "../include/aruco_detector/charuco_detector.h"

Matx41d aa2quaternion2(const Matx31d& aa)
{
  double angle = cv::norm(aa);
  Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
  double angle_2 = angle / 2;
  //qx, qy, qz, qw
  Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
  return q;
}

void displayResult(Mat image, Mat camera_matrix, Mat dist_coeffs, Vec3d rvec, Vec3d tvec, std::string name){
  cv::drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1f);
//  Mat dst;
//  cv::resize(image, dst, cv::Size(640, 480), 0, 0, INTER_CUBIC);
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 640, 480);
  cv::imshow(name, image);
  cv::waitKey(100);
}

void displayResult2(Mat image, vector<int> ids, vector<vector<cv::Point2f> > marker_corners, std::string name){
  cv::aruco::drawDetectedMarkers(image, marker_corners, ids);
//  Mat dst;
//  cv::resize(image, dst, cv::Size(640, 480), 0, 0, INTER_CUBIC);
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 640, 480);
  cv::imshow(name, image);
  cv::waitKey(10);
}

Mat vectors2Pose(Vec3d & rvec, Vec3d & tvec)
{
  Mat result = Mat::eye(4, 4, CV_64FC1); auto data = (double*)result.data;
  Mat rotation; Rodrigues(rvec, rotation); auto rData = (double*)rotation.data;
  for (auto row = 0; row < 3; row++)
  {
    for (auto column = 0; column < 3; column++)
    {
      data[column + row * 4] = rData[column + row * 3];
    }
  } data[3] = tvec[0]; data[7] = tvec[1]; data[11] = tvec[2]; return result;
}

std::map<int, geometry_msgs::Pose> CharcuoDetector::processImage(Mat image, sensor_msgs::CameraInfo camera_info, double marker_size, bool display) {
  std::map<int, geometry_msgs::Pose> poses;

   std::cout << CV_VERSION << std::endl;

   vector<int> marker_ids;
   vector<vector<cv::Point2f> > marker_corners;
   vector<vector<cv::Point2f> > marker_rejected;//Markers to ingore

//  cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
   this->detect(image, marker_ids, marker_corners);

    // refine strategy to detect more markers
    aruco::refineDetectedMarkers(image, this->board, marker_corners, marker_ids, marker_rejected);

   std::vector<cv::Point2f> charuco_corners;
   std::vector<int> charuco_ids;

//   This process uses the K camera matrix and D distortion vector from CameraInfo
//   cv::Mat dist_coeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64FC1);
   cv::Mat camera_matrix = this->getCameraMatrix(camera_info);
   cv::Mat dist_coeffs   = this->getDistCoef(camera_info);

//   cv::Mat camera_matrix = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);


//   std::cout << camera_info   << std::endl;
   std::cout << camera_matrix << std::endl;
   std::cout << dist_coeffs   << std::endl;

   cv::Mat image_copy = image.clone();
   if(marker_ids.size() > 0) {
     ROS_INFO("Here %i %i", marker_corners.size(), marker_ids.size());
     displayResult2(image_copy, marker_ids, marker_corners, "markers");
     cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, this->board, charuco_corners, charuco_ids, camera_matrix, dist_coeffs);
     ROS_INFO("Here again... %i", charuco_ids.size());
   }

   if (charuco_ids.size() > 0) {
     ROS_INFO("Found");
     cv::Scalar color = cv::Scalar(255, 0, 0);
     cv::aruco::drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids, color);
     cv::Vec3d rvec, tvec;
     ROS_INFO("Pose");
     bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, this->board, camera_matrix, dist_coeffs, rvec,tvec);
     ROS_INFO("Pose: %i", valid);
     // if charuco pose is valid

     if (valid) {
       ROS_INFO("Valid");
       std::cout << tvec << std::endl;
       std::cout << rvec << std::endl;
       displayResult(image_copy, camera_matrix, dist_coeffs, rvec, tvec, "Results");

      Vec<double, 3> rotation    = rvec;
      Vec<double, 3> translation = tvec;

      Mat transform = vectors2Pose(rvec, tvec);
      Vec<double, 4> corner_pose(0,0,0,1);

      Mat board_pose = transform * corner_pose;

      geometry_msgs::Pose pose;
      pose.position.x = board_pose.at<double>(0);
      pose.position.y = board_pose.at<double>(1);
      pose.position.z = board_pose.at<double>(2);

      Matx41d quaternion = aa2quaternion2(rotation);
      pose.orientation.x = quaternion.val[0];
      pose.orientation.y = quaternion.val[1];
      pose.orientation.z = quaternion.val[2];
      pose.orientation.w = quaternion.val[3];
      poses[0] = pose;

       pose.position.x = translation[0];
       pose.position.y = translation[1];
       pose.position.z = translation[2];
       poses[1] = pose;
     }
   }
   else
     ROS_INFO("Not Found");
  
  return poses;
}
