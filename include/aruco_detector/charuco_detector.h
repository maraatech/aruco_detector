//
// Created by anyone on 12/04/22.
//

#ifndef ARUCO_DETECTOR_CHARUCO_DETECTOR_H
#define ARUCO_DETECTOR_CHARUCO_DETECTOR_H

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
#include "math.h"
#include <tf/tf.h>

using namespace cv;
using namespace std;

class CharcuoDetector : public MarkerDetector {
private:
    cv::Ptr<cv::aruco::CharucoBoard> board;
    bool once;
public:
    CharcuoDetector(int dictionary_id, int board_width, int board_height, double square_length, double marker_length) : MarkerDetector(dictionary_id){
      this->board = cv::aruco::CharucoBoard::create(board_width, board_height, square_length, marker_length, this->dictionary);
      this->once = true;
    }

    //Stereo Detection
    virtual std::map<int, geometry_msgs::Pose> processImages(Mat left_image,
                                                             Mat right_image,
                                                             cares_msgs::StereoCameraInfo stereo_info,
                                                             bool display) override;
    //Single Image
    std::map<int, geometry_msgs::Pose> processImage(Mat image,
                                                    sensor_msgs::CameraInfo camera_info,
                                                    double marker_size,
                                                    bool display) override;
//    //Depth Detection
//    virtual std::map<int, geometry_msgs::Pose> processImage(Mat image,
//                                                            Mat depth_image,
//                                                            sensor_msgs::CameraInfo camera_info,
//                                                            bool display,
//                                                            bool is_depth_in_meters=false) override;
};

#endif //ARUCO_DETECTOR_CHARUCO_DETECTOR_H
