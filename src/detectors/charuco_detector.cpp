//
// Created by anyone on 12/04/22.
//
#include "../../include/aruco_detector/charuco_detector.h"

void CharcuoDetector::displayFrameAxes(Mat image, Mat camera_matrix, Mat dist_coeffs, Vec3d rvec, Vec3d tvec, std::string name){
  cv::drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1f);
//  Mat dst;
//  cv::resize(image, dst, cv::Size(640, 480), 0, 0, INTER_CUBIC);
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 640, 480);
  cv::imshow(name, image);
  cv::waitKey(100);
}

std::map<int, geometry_msgs::Pose> CharcuoDetector::processImage(Mat image, sensor_msgs::CameraInfo camera_info, double marker_size, bool display) {
  std::map<int, geometry_msgs::Pose> poses;

  vector<int> marker_ids;
  vector<vector<cv::Point2f> > marker_corners;
  vector<vector<cv::Point2f> > marker_rejected;//Markers to ingore

  this->detectMarkers(image, marker_ids, marker_corners);

  // refine strategy to detectMarkers more markers
  aruco::refineDetectedMarkers(image, this->board, marker_corners, marker_ids, marker_rejected);

  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;

  cv::Mat camera_matrix = getCameraMatrix(camera_info);
  cv::Mat dist_coeffs   = getDistCoef(camera_info);

  cv::Mat image_copy = image.clone();
  if(marker_ids.size() > 0) {
    if(display) {
      displayMarkers(image_copy, marker_ids, marker_corners, "markers");
    }
    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, this->board, charuco_corners, charuco_ids);//, camera_matrix, dist_coeffs);
  }

  if (charuco_ids.size() > 0) {
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::aruco::drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids, color);
    cv::Vec3d rotation_vec, translation_vec;
    bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, this->board, camera_matrix, dist_coeffs, rotation_vec, translation_vec);

    if (valid) {
      if(display) {
        displayFrameAxes(image_copy, camera_matrix, dist_coeffs, rotation_vec, translation_vec, "Results");
      }

      geometry_msgs::Pose pose;
      pose.position.x = translation_vec[0];
      pose.position.y = translation_vec[1];
      pose.position.z = translation_vec[2];

      Matx41d quaternion = aa2quaternion(rotation_vec);
      pose.orientation.x = quaternion.val[0];
      pose.orientation.y = quaternion.val[1];
      pose.orientation.z = quaternion.val[2];
      pose.orientation.w = quaternion.val[3];

      poses[0] = pose;
    }
  }
  return poses;
}

//TODO Experimental Funciton using homography over 2D points - results poor due to bugs
//std::map<int, geometry_msgs::Pose> CharcuoDetector::processImages(Mat left_image, Mat right_image, cares_msgs::StereoCameraInfo stereo_info, bool display) {
////  return this->stereoProcess(left_image, right_image, stereo_info, display);
//  std::map<int, geometry_msgs::Pose> board_poses;
//
//  std::map<int, geometry_msgs::Pose> marker_poses = MarkerDetector::processImages(left_image, right_image, stereo_info, display);
//
//  int board_width       = this->board->getChessboardSize().width / 2;//half the number of aruco IDs vs Squares
//  int board_height      = this->board->getChessboardSize().height / 2;//half the number of aruco IDs vs Squares
//  double square_length  = this->board->getSquareLength();
//
//  vector<Point2f> board_points_planar;
//  vector<Point2f> marker_points_planar;
//  for(auto marker_pose : marker_poses) {
//    int id = marker_pose.first;
//
//    int index_x = id % board_width;
//    int index_y = id / board_width;
//
//    double board_x = square_length * index_x * 2;
//    if (index_y % 2 != 0)//if odd account for blank square at start
//      board_x += square_length;
//    double board_y = square_length * index_y;//twice as many squares as aruco Ids
//
//    board_points_planar.push_back(cv::Point2f(board_x, board_y));
//
//    geometry_msgs::Pose pose = marker_pose.second;
//    marker_points_planar.push_back(cv::Point2f(pose.position.x, pose.position.y));
//  }
//
//  cv::Mat H = cv::findHomography(marker_points_planar, board_points_planar);
//
//  cv::Vec3d point(0, 0, 1);
//  cv::Mat result = H * point;
//
//  geometry_msgs::Pose pose;
//  pose.position.x = result.at<double>(0);
//  pose.position.y = result.at<double>(1);
//  pose.position.z = result.at<double>(2);
//  pose.orientation.w = 1.0;
//  board_poses[0] = pose;
//
//  return board_poses;
//}

//TODO Experimental Function for stereo pose estimation - results are poor
//std::map<int, geometry_msgs::Pose> CharcuoDetector::processImages(Mat left_image, Mat right_image, cares_msgs::StereoCameraInfo stereo_info, bool display) {
//  std::map<int, geometry_msgs::Pose> marker_poses = MarkerDetector::processImages(left_image, right_image, stereo_info, display);
//
//  std::map<int, geometry_msgs::Pose> board_poses;
//
//  int board_width       = this->board->getChessboardSize().width / 2;//half the number of aruco IDs vs Squares
//  int board_height      = this->board->getChessboardSize().height / 2;//half the number of aruco IDs vs Squares
//  double square_length  = this->board->getSquareLength();
//
//  double roll_avg, pitch_avg, yaw_avg;
//  for(auto marker_pose : marker_poses){
//    geometry_msgs::Pose pose = marker_pose.second;
//    tf::Quaternion q(
//            pose.orientation.x,
//            pose.orientation.y,
//            pose.orientation.z,
//            pose.orientation.w);
//    tf::Matrix3x3 matrix(q);
//    double roll, pitch, yaw;
//    matrix.getRPY(roll, pitch, yaw);
//    roll_avg  += normaliseAngle(roll);
//    pitch_avg += normaliseAngle(pitch);
//    yaw_avg   += normaliseAngle(yaw);
//  }
//  roll_avg  = roll_avg / marker_poses.size();
//  pitch_avg = pitch_avg / marker_poses.size();
//  yaw_avg   = yaw_avg / marker_poses.size();
//
//  double x_avg = 0;
//  double y_avg = 0;
//  double z_avg = 0;
//  for (auto marker_pose : marker_poses) {
//    int id = marker_pose.first;
//
//    int index_x = id % board_width;
//    int index_y = id / board_width;
//
//    double board_x = square_length * index_x * 2;
//    if (index_y % 2 != 0)//if odd account for blank square at start
//      board_x += square_length;
//    double board_y = square_length * index_y;//twice as many squares as aruco Ids
//
//    //Pose of the marker relative to the camera optical space
//    geometry_msgs::Pose pose = marker_pose.second;
//
//    tf::Quaternion q(
//            pose.orientation.x,
//            pose.orientation.y,
//            pose.orientation.z,
//            pose.orientation.w);
////    tf::Quaternion q;
////    q.setRPY(roll_avg, pitch_avg, yaw_avg);
//    tf::Matrix3x3 matrix(q);
//    cv::Mat rotation(3,3,CV_64F);
//    for(int i = 0; i < 3; i++)
//      for(int j = 0; j < 3; j++)
//        rotation.at<double>(i, j) = matrix[i][j];
//
//    cv::Vec3d point(-board_x, board_y, 0);
//    cv::Mat result = rotation * point;
//
//    x_avg += pose.position.x + result.at<double>(0);
//    y_avg += pose.position.y + result.at<double>(1);
//    z_avg += pose.position.z + result.at<double>(2);
//  }
//  geometry_msgs::Pose estimated_pose;
//  estimated_pose.position.x = x_avg / marker_poses.size();
//  estimated_pose.position.y = y_avg / marker_poses.size();
//  estimated_pose.position.z = z_avg / marker_poses.size();
//  estimated_pose.orientation.w = 1;
//  board_poses[0] = estimated_pose;
//
//  this->once = false;
//  return board_poses;
//}
//  std::vector<geometry_msgs::Pose> estimatd_poses;
