#ifndef FISHEYE_RECTIFICATION_NODE_HPP
#define FISHEYE_RECTIFICATION_NODE_HPP


#include <ros/ros.h>


#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/**
 * @brief The FishEyeRectifier class rectifies the image of a fisheye camera with the camera parameters from the camera_info topic and provides the rectified image.
 */

class FishEyeRectifier{
  public:
  FishEyeRectifier();

  void cam_info_callback(const sensor_msgs::CameraInfoConstPtr &msg);
  void image_callback(const sensor_msgs::ImageConstPtr &msg);

  private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Subscriber cam_info_sub;

    bool cam_info_received;

    // camera parameters
    cv::Size image_size;
    cv::Mat camera_matrix;
    cv::Mat distortion_coeff;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;

    // maps for rectification
    cv::Mat map1;
    cv::Mat map2;


};


#endif // FISHEYE_RECTIFICATION_NODE_HPP
