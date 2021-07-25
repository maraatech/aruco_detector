//
// Created by anyone on 18/05/21.
//

#ifndef SRC_DIAMOND_DETECTOR_H
#define SRC_DIAMOND_DETECTOR_H

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/aruco/charuco.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <cares_msgs/StereoCameraInfo.h>
#include "detector.h"

using namespace cv;
using namespace std;

class DiamondDetector : public MarkerDetector {
private:
    int centre_id, top_left_id, top_right_id, bottom_right_id, bottom_left_id;
public:
    DiamondDetector(int dictionary_id, int centre_id, int top_left_id, int top_right_id, int bottom_right_id, int bottom_left_id) : MarkerDetector(dictionary_id){
      this->centre_id       = centre_id;
      this->top_left_id     = top_left_id;
      this->top_right_id    = top_right_id;
      this->bottom_right_id = bottom_right_id;
      this->bottom_left_id  = bottom_left_id;
    }

    //Stereo Detection
    virtual std::map<int, geometry_msgs::Pose> processImages(Mat left_image,
                                                     Mat right_image,
                                                     cares_msgs::StereoCameraInfo stereo_info,
                                                     bool display) override;
//    //Single Image
//    std::map<int, geometry_msgs::Pose> processImage(Mat image,
//                                                    sensor_msgs::CameraInfo camera_info,
//                                                    double marker_size,
//                                                    bool display);
//    //Depth Detection
    virtual std::map<int, geometry_msgs::Pose> processImage(Mat image,
                                                    Mat depth_image,
                                                    sensor_msgs::CameraInfo camera_info,
                                                    bool display,
                                                    bool is_depth_in_meters=false);
};


#endif //SRC_DIAMOND_DETECTOR_H
