#ifndef PID_CONTROLLER_NODE_H
#define PID_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "ar_land/pid.hpp"

class pid_controller_node
{
public:

  ros::NodeHandle nh;
  pid_controller_node(const std::string& worldFrame, const std::string& frame, const ros::NodeHandle& n);

  //Functions
  void run(double frequency);
  void iteration(const ros::TimerEvent& e);


private:


  // Functions
  double get(const ros::NodeHandle& n, const std::string& name);
  void goalChanged(const geometry_msgs::PoseStamped::ConstPtr& msg);  
  void pidReset();
  void pidStart();



  // Subscribers
  ros::Subscriber pose_goal_in_world_sub;

  // Publisher
  ros::Publisher control_out_pub;


  // Variables
  bool controller_enabled;
  bool controller_started;
  bool resetPID;

  float z_integral;

  std::string world_frame_id;
  std::string drone_frame_id;

  tf::TransformListener tf_lis;
  PID pid_x;
  PID pid_y;
  PID pid_z;
  PID pid_yaw;
  geometry_msgs::PoseStamped pose_goal_in_world_msg;

};

#endif // PID_CONTROLLER_NODE_H
