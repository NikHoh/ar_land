#include "ar_land/fisheye_rectification_node.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>



FishEyeRectifier::FishEyeRectifier() : cam_info_received(false), it(nh){

  image_sub = it.subscribe("/image",1, &FishEyeRectifier::image_callback,this);
  cam_info_sub = nh.subscribe("/camera_info", 1, &FishEyeRectifier::cam_info_callback,this);

  image_pub = it.advertise("fisheye_rectified",1);

}

void FishEyeRectifier::cam_info_callback(const sensor_msgs::CameraInfoConstPtr& msg){

  // save cam parameters somewhere somehow
  camera_matrix = cv::Mat(3,3,CV_64FC1, (void *) msg->K.data());
  //distortion wtf
  distortion_coeff = cv::Mat(1,4,CV_64FC1, (void *) msg->D.data()); // only for fisheye, check this before
  rectification_matrix = cv::Mat(3,3,CV_64FC1, (void *) msg->R.data());
  projection_matrix = cv::Mat(3,4,CV_64FC1, (void *) msg->P.data());
  image_size = cv::Size(msg->width,msg->height);

  cv::fisheye::initUndistortRectifyMap(camera_matrix, distortion_coeff, rectification_matrix, projection_matrix,image_size, CV_32FC1,map1,map2);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void FishEyeRectifier::image_callback(const sensor_msgs::ImageConstPtr &msg){

  if(!cam_info_received) return;

  cv_bridge::CvImagePtr cv_ptr;


  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //alternative toCvShare
    cv::Mat input_image;
    input_image = cv_ptr->image;
    cv::Mat result_image;                 // reconsider
    //result_image = cv_ptr->image.clone();

    cv::remap(input_image,result_image,map1,map2,cv::INTER_LINEAR,cv::BORDER_CONSTANT);

    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = msg->header.frame_id;
    out_msg.header.stamp = msg->header.stamp;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = result_image;
    image_pub.publish(out_msg.toImageMsg());

  } catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}

/*
void FishEyeRectifier::detectArucoMarker(cv::Mat &inputImage){
  //cv::Mat markerImage;
  int markerLength = 20;  // 16 cm for ours
  cv::cvtColor(inputImage,outpuImage,cv::COLOR_BGR2GRAY);
  cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); // example dictionary with 250 6x6 markers
  //cv::aruco::drawMarker(dictionary,23,200,markerImage,1);  //only for marker creatioin
  std::vector<int> markerIds;
  std::vector<Point2f> markerCorners, rejectedCandidates;
  cv::aruco::DetectorParameters parameters;
  cv::aruco::detectMarkers(inputImage,dictionary,markerCorners,markerIds,parameters,rejectedCandidates);
  cv::Mat outpuImage;  //same as input image
  cv::aruco::drawDetectedMarkers(outputImage,markerCorners,markerIds);
  std::vector<Vec3d> rvecs,tvecs;
  //cv::aruco::estimatePoseBoard(markerCorners,markerIds,) use this
  cv::aruco::estimatePoseSingleMarkers(markerCorners,markerLength,camera_matrix,distortion_coeff,rvecs,tvecs);
  cv::aruco::drawAxis(outpuImage,camera_matrix,distortion_coeff,rvec,tvec,0.1); // only for single frame

}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fisheye_rectification_node");

  FishEyeRectifier rectifier;

  ros::spin();

}
