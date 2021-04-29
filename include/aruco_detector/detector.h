//
// Created by anyone on 12/02/20.
//

#ifndef ARUCO_DETECTOR_DETECTOR_H
#define ARUCO_DETECTOR_DETECTOR_H

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/aruco/charuco.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <cares_msgs/StereoCameraInfo.h>

using namespace cv;
using namespace std;

class MarkerDetector {
public:
    MarkerDetector();

    //Stereo Detection
    std::map<int, geometry_msgs::Pose> processImages(Mat left_image, Mat right_image, cares_msgs::StereoCameraInfo stereo_info, bool display);
    //Single Image
    std::map<int, geometry_msgs::Pose> processImage(Mat image, sensor_msgs::CameraInfo camera_info, double marker_size, bool display);
    //Depth Detection
    std::map<int, geometry_msgs::Pose> processImage(Mat image, Mat depth_image, sensor_msgs::CameraInfo camera_info, bool display, bool is_depth_in_meters=false);

private:

    Ptr<aruco::DetectorParameters> detector_params_;
    Ptr<aruco::Dictionary> dictionary_;

    void detect(Mat image, vector<int> &point_ids, vector<vector<cv::Point2f> > &marker_corners);
};


#endif //ARUCO_DETECTOR_DETECTOR_H
