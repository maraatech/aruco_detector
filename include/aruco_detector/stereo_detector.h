//
// Created by anyone on 24/02/20.
//

#ifndef ARUCO_DETECTOR_STEREO_DETECTOR_H
#define ARUCO_DETECTOR_STEREO_DETECTOR_H

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/aruco/charuco.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>
#include <maara_msgs/StereoCameraInfo.h>

using namespace cv;
using namespace std;

class StereoMarkerDetector {
public:
    StereoMarkerDetector() {
      this->dictionary_      = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
      this->detector_params_ = aruco::DetectorParameters::create();
    }

    std::map<int, geometry_msgs::Pose> processImages(Mat left_image, Mat right_image, maara_msgs::StereoCameraInfo stereo_info, double marker_size, bool display);

private:
    Ptr<aruco::DetectorParameters> detector_params_;
    Ptr<aruco::Dictionary> dictionary_;

    void detect(Mat image, vector<int> &point_ids, vector<vector<cv::Point2f> > &marker_corners);
};

#endif //ARUCO_DETECTOR_STEREO_DETECTOR_H
