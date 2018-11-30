#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

/**
 * Simple ROS node that does a very basic integration of IMU linear acceleration values.
 * Does not take rotation into account. Only intended for debugging.
 */


class accel_calib_test_node {
  ros::Publisher vel_pub;
  ros::Publisher pose_pub;
  ros::Subscriber sub;
  ros::NodeHandle n;
  public:
    accel_calib_test_node();
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
};

accel_calib_test_node::accel_calib_test_node () : n("~") {
  this->sub = n.subscribe("/crazyflie/imu", 20, &accel_calib_test_node::imu_cb, this);            
  this->vel_pub = n.advertise<geometry_msgs::Vector3>("/ar_land/imu_lin_velocity", 20);
  this->pose_pub = n.advertise<geometry_msgs::Vector3>("/ar_land/imu_lin_position", 20);
}

void accel_calib_test_node::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    static ros::Time last_cb_time = ros::Time::now();
    static geometry_msgs::Vector3 velocity;
    static geometry_msgs::Vector3 position;

    geometry_msgs::Vector3 curr_acc = msg->linear_acceleration;
    ros::Time curr_time = ros::Time::now();
    float dt = (curr_time - last_cb_time).toSec();

    velocity.x += curr_acc.x*dt;
    velocity.y += curr_acc.y*dt;
    velocity.z += curr_acc.z*dt;

    position.x += 0.5*curr_acc.x*dt*dt;
    position.y += 0.5*curr_acc.y*dt*dt;
    position.z += 0.5*curr_acc.z*dt*dt;

    this->vel_pub.publish(velocity);
    this->pose_pub.publish(position);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "accel_calib_test_node"); 
  accel_calib_test_node node;
  ros::spin();
  return 0;
}