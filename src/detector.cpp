//
// Created by anyone on 12/02/20.
//

#include <tf2/LinearMath/Quaternion.h>
#include "detector.h"

MarkerDetector::MarkerDetector()
{
//  this->dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
  this->dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  this->detector_params_ = aruco::DetectorParameters::create();
}

/**
 * Attempt to detect Aruco markers/diamond in image
 * @param image
 * @param marker_ids
 * @param marker_corners
 */
void MarkerDetector::detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners)
{
  vector<vector<cv::Point2f> > rejected_markers_;//Markers to ingore
  aruco::detectMarkers(image, this->dictionary_, marker_corners, marker_ids, this->detector_params_, rejected_markers_);
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

/**
 * Process the given image and return detected markers
 * @param image
 * @param display
 * @return detected markers
 */
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

//D: [0.0, 0.0, 0.0, 0.0, 0.0]
//K: [613.3427734375, 0.0, 319.651123046875, 0.0, 613.8692626953125, 247.9708709716797, 0.0, 0.0, 1.0]
//R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
//P: [613.3427734375, 0.0, 319.651123046875, 0.0, 0.0, 613.8692626953125, 247.9708709716797, 0.0, 0.0, 0.0, 1.0, 0.0]

