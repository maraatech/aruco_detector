//
// Created by tqia574 on 13/01/20.
//

#ifndef MAARA_CALIBRATION_DEPTH_MARKER_DETECTOR_H
#define MAARA_CALIBRATION_DEPTH_MARKER_DETECTOR_H

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/aruco/charuco.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>

using namespace cv;
using namespace std;

class DepthMarkerDetector {
public:
  DepthMarkerDetector() {
    this->dictionary_      = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    this->detector_params_ = aruco::DetectorParameters::create();
  }

  std::map<int, geometry_msgs::Pose> processImage(Mat image, Mat depth_image, sensor_msgs::CameraInfo camera_info,  bool display);

private:
    Ptr<aruco::DetectorParameters> detector_params_;
    Ptr<aruco::Dictionary> dictionary_;

    Point getMarkerCenter(vector<Point2f> corners);
    void detect(Mat image, vector<int> &point_ids, vector<vector<cv::Point2f> > &marker_corners);
};


#endif //MAARA_CALIBRATION_DEPTH_MARKER_DETECTOR_H
