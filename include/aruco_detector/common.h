//
// Created by anyone on 17/05/21.
//

#ifndef SRC_COMMON_H
#define SRC_COMMON_H

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <cares_msgs/StereoCameraInfo.h>

namespace cares{
    cv::Mat getQMatrix(const cares_msgs::StereoCameraInfo stereo_info);
    cv::Mat getCameraMatrix(const sensor_msgs::CameraInfo camera_info);
    cv::Mat getDistCoef(const sensor_msgs::CameraInfo camera_info);

    cv::Mat vectors2Pose(cv::Vec3d & rvec, cv::Vec3d & tvec);

    cv::Matx41d aa2quaternion(const cv::Matx31d& aa);

    cv::Point getMarkerCenter(std::vector<cv::Point2f> corners);

    geometry_msgs::Pose findDepthPoint(cv::Mat depth_image, sensor_msgs::CameraInfo camera_info, int pixel_x, int pixel_y, double is_depth_in_meters);

    geometry_msgs::Pose diff(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

    geometry_msgs::Pose ave(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

    geometry_msgs::Pose crossProduct(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

    double length(geometry_msgs::Pose pose);

    geometry_msgs::Pose norm(geometry_msgs::Pose p1);

    bool isValid(geometry_msgs::Pose pose);

    geometry_msgs::Pose calculateStereoPose(cv::Point left_point, cv::Point right_point, cv::Mat Q);

    int getIndex(int id, std::vector<int> &point_ids);
}

#endif //SRC_COMMON_H
