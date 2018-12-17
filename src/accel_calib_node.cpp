#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>


/**
 * This node can be used to gather data for IMU-Accelerometer calibration.
 * Output data will be written to ~/calib-data.csv.
 *
 * (OpenCV is used to capture keyboard events ...)
 * 
 * parameters:
 * - imu_topic_name (string): the name of the IMU topic
 * - num_values (int): the number of measurements to use for averaging acceleration in each orientation
 */


class accel_calib_node {
  ros::Subscriber sub;
  ros::NodeHandle n;

  public:
    accel_calib_node();
    void keyLoop();
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
  
  private:
    int num_values;
    bool recording;
    std::vector<geometry_msgs::Vector3> accel_buffer;
    std::vector<std::vector<double> > out_buffer;
    void save_all();
    void write_average();
};

accel_calib_node::accel_calib_node () : n("~") {
  n.param("num_values", num_values, 25);
  std::string imu_topic_name;
  n.getParam("imu_topic_name", imu_topic_name);
  recording = false;

  sub = n.subscribe(imu_topic_name, 20, &accel_calib_node::imu_cb, this);
}

void accel_calib_node::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  geometry_msgs::Vector3 curr_acc = msg->linear_acceleration;

  if (recording && accel_buffer.size() < num_values){
    accel_buffer.push_back(curr_acc);
  }
  else if (accel_buffer.size() == num_values) {
    ROS_INFO("Recording Done ...");
    write_average();
    recording = false;
    accel_buffer.clear();
    keyLoop();
  }
}

void accel_calib_node::save_all() {
  std::ofstream file;
  file.open ("/home/johannes/calib-data.csv", std::ofstream::out | std::ofstream::app);
  file << "x,y,z\n";

  for (int i=0; i<out_buffer.size(); i++) {
    std::vector<double> vals = out_buffer[i];
    for (int i=0; i<3; i++) {
      file << vals[i];
      if (i != 2) {
        file << ",";
      }
    }
    file << "\n";
  }

  file.close();
  out_buffer.clear();
  ROS_INFO("Calibration data saved.");
}

void accel_calib_node::write_average() {
  double avg_x, avg_y, avg_z;
  int num_values = accel_buffer.size();

  for (int i=0; i<num_values; i++) {
    geometry_msgs::Vector3 vec = accel_buffer[i];
    avg_x += vec.x;
    avg_y += vec.y;
    avg_z += vec.z;
  }
  
  std::vector<double> measurement;
  measurement.push_back(avg_x/num_values);
  measurement.push_back(avg_y/num_values);
  measurement.push_back(avg_z/num_values);
  out_buffer.push_back(measurement);
}


#define KEYCODE_R 114
#define KEYCODE_S 115

void accel_calib_node::keyLoop()
{
  cv::Mat no_img = cv::Mat(10,10, CV_64F, cvScalar(0.));
  cv::imshow("key", no_img);
  
  while (ros::ok() && !recording) {
    int key = cv::waitKey();
    switch(key)
    {
      case KEYCODE_R:
        ROS_INFO("Recording ...");
        recording = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("Saving values ...");
        try {
            save_all();
        } catch (const std::exception &exc) {
            ROS_INFO("%s", exc.what());
        }
        ros::shutdown();
        break;
    }
  }

  cv::destroyAllWindows();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "accel_calib_node");
  accel_calib_node node;
  node.keyLoop();
  ros::spin();
  return 0;
}
