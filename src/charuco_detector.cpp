//
// Created by anyone on 12/04/22.
//
#include "../include/aruco_detector/charuco_detector.h"

void displayResult(Mat image, Mat camera_matrix, Mat dist_coeffs, Vec3d rvec, Vec3d tvec, std::string name){
  cv::drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 1.0f);
//  Mat dst;
//  cv::resize(image, dst, cv::Size(640, 480), 0, 0, INTER_CUBIC);
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 640, 480);
  cv::imshow(name, image);
  cv::waitKey(100);
}

std::map<int, geometry_msgs::Pose> CharcuoDetector::processImage(Mat image, sensor_msgs::CameraInfo camera_info, double marker_size, bool display) {

   vector<int> marker_ids;
   vector<vector<cv::Point2f> > marker_corners;
   this->detect(image, marker_ids, marker_corners);

   std::vector<cv::Point2f> charuco_corners;
   std::vector<int> charuco_ids;

   cv::Mat camera_matrix, dist_coeffs;
   //This process uses the K camera matrix and D distortion vector from CameraInfo
   camera_matrix = this->getCameraMatrix(camera_info);
   dist_coeffs   = this->getDistCoef(camera_info);

   cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, this->board, charuco_corners, charuco_ids, camera_matrix, dist_coeffs);

   cv::Mat image_copy = image.clone();
   if (charuco_ids.size() > 0) {
     ROS_INFO("Found");
     cv::Scalar color = cv::Scalar(255, 0, 0);
     cv::aruco::drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids, color);
     cv::Vec3d rvec, tvec;
     bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, this->board, camera_matrix, dist_coeffs, rvec,tvec);
     // if charuco pose is valid
     if (valid) {
       ROS_INFO("Valid");
       std::cout << tvec << std::endl;
       std::cout << rvec << std::endl;
       displayResult(image_copy, camera_matrix, dist_coeffs, rvec, tvec, "Results");
     }
   }
   else
     ROS_INFO("Not Found");

  std::map<int, geometry_msgs::Pose> pose;
  return pose;
}
