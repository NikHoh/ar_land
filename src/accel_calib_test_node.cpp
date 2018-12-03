#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <deque>
#include <string>
#include <cmath>


/**
 * This node computes the absolute value of the linear acceleration vector taken from an imu topic.
 * The output value is filtered using a simple moving average filter.
 * 
 * publishes:
 * - /ar_land/imu_absolute_val (std_msgs/Float64)
 * 
 * parameters:
 * - buffer_length (int): number of values to use for the moving average filter
 * - imu_topic_name (string): the name of the IMU topic 
 */


class accel_calib_test_node {
  ros::Publisher imu_absolute_val_pub;
  ros::Subscriber sub;
  ros::NodeHandle n;

  public:
    accel_calib_test_node();
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
  
  private:
    int accel_buffer_len;
    std::deque<geometry_msgs::Vector3> accel_buffer;
    void publish_abs();
};

accel_calib_test_node::accel_calib_test_node () : n("~") {
  n.param("buffer_length", accel_buffer_len, 25);
  std::string imu_topic_name;
  n.getParam("imu_topic_name", imu_topic_name);
  sub = n.subscribe(imu_topic_name, 20, &accel_calib_test_node::imu_cb, this);            
  imu_absolute_val_pub = n.advertise<std_msgs::Float64>("/ar_land/imu_absolute_val", 20);
}

void accel_calib_test_node::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  geometry_msgs::Vector3 curr_acc = msg->linear_acceleration;

  // remove oldest element if the buffer is already full
  // and add the current value
  if (accel_buffer.size() >= accel_buffer_len){
    accel_buffer.pop_back();
  }
  accel_buffer.push_front(curr_acc);

  publish_abs();
}

void accel_calib_test_node::publish_abs(){
  float current_abs = 0;
  
  // compute the average value of the length of the vectors currently in the buffer
  for (int i=0; i<accel_buffer.size(); i++) {
    geometry_msgs::Vector3 vec = accel_buffer[i];
    double xp = pow(vec.x, 2);
    double yp = pow(vec.y, 2);
    double zp = pow(vec.z, 2);    
    current_abs += sqrt(xp + yp + zp);
  }
  current_abs = current_abs / accel_buffer.size();

  std_msgs::Float64 msg;
  msg.data = current_abs;
  imu_absolute_val_pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "accel_calib_test_node"); 
  accel_calib_test_node node;
  ros::spin();
  return 0;
}