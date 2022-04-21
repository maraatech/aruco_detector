//
// Created by anyone on 22/04/22.
//
#include "../include/aruco_detector/common.h"

namespace cares {
    cv::Mat getQMatrix(const cares_msgs::StereoCameraInfo stereo_info) {
      cv::Mat Q(4, 4, CV_64FC1, (void *) stereo_info.Q.data());
      return Q.clone();
    }

    cv::Mat getCameraMatrix(const sensor_msgs::CameraInfo camera_info) {
      cv::Mat camera_matrix(3, 3, CV_64FC1, (void *) camera_info.K.data());
      return camera_matrix.clone();
    }

    cv::Mat getDistCoef(const sensor_msgs::CameraInfo camera_info) {
      cv::Mat dist_coeffs(1, camera_info.D.size(), CV_64FC1, (void *) camera_info.D.data());
      return dist_coeffs.clone();
    }

    cv::Mat vectors2Pose(cv::Vec3d &rvec, cv::Vec3d &tvec) {
      cv::Mat result = cv::Mat::eye(4, 4, CV_64FC1);
      auto data = (double *) result.data;
      cv::Mat rotation;
      Rodrigues(rvec, rotation);
      auto rData = (double *) rotation.data;
      for (auto row = 0; row < 3; row++) {
        for (auto column = 0; column < 3; column++) {
          data[column + row * 4] = rData[column + row * 3];
        }
      }
      data[3] = tvec[0];
      data[7] = tvec[1];
      data[11] = tvec[2];
      return result;
    }

    cv::Matx41d aa2quaternion(const cv::Matx31d &aa) {
      double angle = cv::norm(aa);
      cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
      double angle_2 = angle / 2;
      //qx, qy, qz, qw
      cv::Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
      return q;
    }

    cv::Point getMarkerCenter(std::vector<cv::Point2f> corners) {
      float x = 0, y = 0;
      for (int i = 0; i < corners.size(); i++) {
        x += corners[i].x;
        y += corners[i].y;
      }
      return cv::Point(x / 4, y / 4);
    }

    geometry_msgs::Pose
    findDepthPoint(cv::Mat depth_image, sensor_msgs::CameraInfo camera_info, int pixel_x, int pixel_y,
                   double is_depth_in_meters) {
      //#     [fx  0 cx]
      //# K = [ 0 fy cy]
      //#     [ 0  0  1]

      double z = 0.0;
      if (is_depth_in_meters) {
        z = (double) depth_image.at<float>(pixel_y, pixel_x);
      } else {
        z = (double) depth_image.at<unsigned short>(pixel_y, pixel_x);
      }

      double x = (pixel_x - camera_info.K.at(2)) / camera_info.K.at(0);
      double y = (pixel_y - camera_info.K.at(5)) / camera_info.K.at(4);

      x = x * z;
      y = y * z;
      z = z;

      if (!is_depth_in_meters) {
        x /= 1000.0;
        y /= 1000.0;
        z /= 1000.0;
      }

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

    geometry_msgs::Pose diff(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
      geometry_msgs::Pose pose;
      pose.position.x = p1.position.x - p2.position.x;
      pose.position.y = p1.position.y - p2.position.y;
      pose.position.z = p1.position.z - p2.position.z;
      pose.orientation.w = 1.0;
      return pose;
    }

    geometry_msgs::Pose ave(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
      geometry_msgs::Pose pose;
      pose.position.x = (p1.position.x + p2.position.x) / 2.0;
      pose.position.y = (p1.position.y + p2.position.y) / 2.0;
      pose.position.z = (p1.position.z + p2.position.z) / 2.0;
      pose.orientation.w = 1.0;
      return pose;
    }

    geometry_msgs::Pose crossProduct(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
      geometry_msgs::Pose pose;
      pose.position.x = p1.position.y * p2.position.z - p1.position.z * p2.position.y;
      pose.position.y = p1.position.z * p2.position.x - p1.position.x * p2.position.z;
      pose.position.z = p1.position.x * p2.position.y - p1.position.y * p2.position.x;
      pose.orientation.w = 1.0;
      return pose;
    }

    double length(geometry_msgs::Pose pose) {
      double v = pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x +
                 pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z;
      v = sqrt(v);
      return v;
    }

    geometry_msgs::Pose norm(geometry_msgs::Pose p1) {
      geometry_msgs::Pose pose;
      double length = sqrt(
              p1.position.x * p1.position.x + p1.position.y * p1.position.y + p1.position.z * p1.position.z);
      pose.position.x = p1.position.x / length;
      pose.position.y = p1.position.y / length;
      pose.position.z = p1.position.z / length;
      pose.orientation.w = 1.0;
      return pose;
    }

    bool isValid(geometry_msgs::Pose pose) {
      return pose.position.z > 0;
    }

    geometry_msgs::Pose calculateStereoPose(cv::Point left_point, cv::Point right_point, cv::Mat Q) {
      //Note this is ripped directly from kiwibot_vision. Hard values are unknown as documentation is lacking
      cv::Mat coordmat = cv::Mat::zeros(4, 1, CV_64FC1);
      double disparity = (double) abs((left_point.x) - (right_point.x));
      coordmat.at<double>(0, 0) = left_point.x;//*4
      coordmat.at<double>(0, 1) = left_point.y;// * 4
      coordmat.at<double>(0, 2) = disparity;
      coordmat.at<double>(0, 3) = 1;
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

    int getIndex(int id, std::vector<int> &point_ids) {
      for (int i = 0; i < point_ids.size(); i++) {
        if (point_ids[i] == id) {
          return i;
        }
      }
      return -1;
    }
}