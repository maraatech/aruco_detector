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

double normaliseAngle(double angle){
  //180 == M_PI
  //180 to -180
  //185 -> -175
  if(angle >= M_PI){
    return -M_PI + (angle - M_PI);
  }
  // -185 -> 175
  if(angle <= -M_PI){
    return M_PI + (angle + M_PI);
  }
  return angle;
}

std::map<int, geometry_msgs::Pose> CharcuoDetector::processImages(Mat left_image, Mat right_image, cares_msgs::StereoCameraInfo stereo_info, bool display) {
  std::map<int, geometry_msgs::Pose> marker_poses = MarkerDetector::processImages(left_image, right_image, stereo_info, display);

  std::map<int, geometry_msgs::Pose> board_poses;

  int board_width       = this->board->getChessboardSize().width / 2;//half the number of aruco IDs vs Squares
  int board_height      = this->board->getChessboardSize().height / 2;//half the number of aruco IDs vs Squares
  double square_length  = this->board->getSquareLength();

  double roll_avg, pitch_avg, yaw_avg;
  for(auto marker_pose : marker_poses){
    geometry_msgs::Pose pose = marker_pose.second;
    tf::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
    tf::Matrix3x3 matrix(q);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    roll_avg  += normaliseAngle(roll);
    pitch_avg += normaliseAngle(pitch);
    yaw_avg   += normaliseAngle(yaw);
  }
  roll_avg  = roll_avg / marker_poses.size();
  pitch_avg = pitch_avg / marker_poses.size();
  yaw_avg   = yaw_avg / marker_poses.size();

  double x_avg = 0;
  double y_avg = 0;
  double z_avg = 0;
  for (auto marker_pose : marker_poses) {
    int id = marker_pose.first;

    int index_x = id % board_width;
    int index_y = id / board_width;

    double board_x = square_length * index_x * 2;
    if (index_y % 2 != 0)//if odd account for blank square at start
      board_x += square_length;
    double board_y = square_length * index_y;//twice as many squares as aruco Ids

    //Pose of the marker relative to the camera optical space
    geometry_msgs::Pose pose = marker_pose.second;

    tf::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
//    tf::Quaternion q;
//    q.setRPY(roll_avg, pitch_avg, yaw_avg);
    tf::Matrix3x3 matrix(q);
    cv::Mat rotation(3,3,CV_64F);
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        rotation.at<double>(i, j) = matrix[i][j];

    cv::Vec3d point(-board_x, board_y, 0);
    cv::Mat result = rotation * point;

    x_avg += pose.position.x + result.at<double>(0);
    y_avg += pose.position.y + result.at<double>(1);
    z_avg += pose.position.z + result.at<double>(2);

//    geometry_msgs::Pose estimated_pose;
//    estimated_pose.position.x = pose.position.x + result.at<double>(0);
//    estimated_pose.position.y = pose.position.y + result.at<double>(1);
//    estimated_pose.position.z = pose.position.z + result.at<double>(2);
//    estimated_pose.orientation.w = 1;

//    board_poses[id] = estimated_pose;
  }
  geometry_msgs::Pose estimated_pose;
  estimated_pose.position.x = x_avg / marker_poses.size();
  estimated_pose.position.y = y_avg / marker_poses.size();
  estimated_pose.position.z = z_avg / marker_poses.size();
  estimated_pose.orientation.w = 1;
  board_poses[0] = estimated_pose;

  this->once = false;
  return board_poses;
}
//  std::vector<geometry_msgs::Pose> estimatd_poses;
