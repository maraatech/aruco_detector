//
// Created by anyone on 17/05/21.
//

#ifndef SRC_DETECTOR_H
#define SRC_DETECTOR_H

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
private:
    Ptr<aruco::DetectorParameters> detector_params;
    Ptr<aruco::Dictionary> dictionary;

    void detect(Mat image, vector<int> &marker_ids, vector<vector<cv::Point2f> > &marker_corners);
public:
    MarkerDetector(int dictionary_id){
      this->dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
      this->detector_params = aruco::DetectorParameters::create();
    }

    //Stereo Detection
    virtual std::map<int, geometry_msgs::Pose> processImages(Mat left_image,
                                                     Mat right_image,
                                                     cares_msgs::StereoCameraInfo stereo_info,
                                                     bool display);
    //Single Image
    std::map<int, geometry_msgs::Pose> processImage(Mat image,
                                                    sensor_msgs::CameraInfo camera_info,
                                                    double marker_size,
                                                    bool display);
    //Depth Detection
    std::map<int, geometry_msgs::Pose> processImage(Mat image,
                                                    Mat depth_image,
                                                    sensor_msgs::CameraInfo camera_info,
                                                    bool display,
                                                    bool is_depth_in_meters=false);

    geometry_msgs::Pose calculatePose(geometry_msgs::Pose p_centre,
                                      geometry_msgs::Pose top_left,
                                      geometry_msgs::Pose top_right,
                                      geometry_msgs::Pose bot_right,
                                      geometry_msgs::Pose  bot_left);
};



#endif //SRC_DETECTOR_H
