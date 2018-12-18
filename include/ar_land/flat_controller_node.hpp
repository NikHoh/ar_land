#ifndef FLAT_CONTROLLER_NODE_H
#define FLAT_CONTROLLER_NODE_H

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
#include "ar_land/tools.hpp"
#include <dynamic_reconfigure/server.h>
#include <ar_land/dynamic_param_configConfig.h>
#include <ar_land/PosVelAcc.h>
#include <sensor_msgs/Imu.h>
#include <ar_land/PosVelAcc.h>
#include <crazyflie_driver/GenericLogData.h>

class flat_controller_node
{
public:

  ros::NodeHandle nh;
  flat_controller_node(const std::string& world_frame_id, const std::string& drone_frame_id, const std::string& imu_frame_id, const ros::NodeHandle& n);

  //Functions
  void run(double frequency);
  void iteration(const ros::TimerEvent& e);
  void dynamic_reconfigure_callback(ar_land::dynamic_param_configConfig& config, uint32_t level);


private:


  // Functions
  double get(const ros::NodeHandle& n, const std::string& name);
  void receiveTrajectory(const ar_land::PosVelAcc::ConstPtr& msg);
  void receiveIMUdata(const sensor_msgs::Imu::ConstPtr& msg);
  void receiveIMURot_Quat(const crazyflie_driver::GenericLogDataConstPtr& msg);
  //void receiveIMURot_rpy(const crazyflie_driver::GenericLogDataConstPtr& msg); // one or the other
  void pidReset();
  void pidStart();
  void getActualPosVel(const ros::TimerEvent& e);



  // Subscribers
  //ros::Subscriber pose_goal_in_world_sub;
  ros::Subscriber goal_posVelAcc_sub;
  ros::Subscriber imuData_sub;
  ros::Subscriber imuRotation_quat_sub;
  ros::Subscriber imuRotation_rpy_sub;


  // Publisher
  ros::Publisher control_out_pub;
  ros::Publisher control_error_pub;
  ros::Publisher obs_posVelAcc_pub;


  // Variables
  bool controller_enabled;
  bool controller_started;
  bool resetPID;
  bool observer_init;

  float z_integral;

  std::string world_frame_id;
  std::string drone_frame_id;
  std::string imu_frame_id;

  tf::TransformListener tf_lis;
  //PID pid_x;
  //PID pid_y;
  //PID pid_z;
  PID pid_yaw;
  ar_land::PosVelAcc posVelAcc_goal_in_world_msg;
  sensor_msgs::Imu imuData_msg;

  tf::Quaternion imuRotation;



  dynamic_reconfigure::Server<ar_land::dynamic_param_configConfig> m_server;

  tf::Vector3 a_ref;
  tf::Vector3 v_ref;
  tf::Vector3 x_ref;

  tf::Matrix3x3 K_x;
  tf::Matrix3x3 K_v;

  tf::Vector3 v_actual;
  tf::Vector3 x_actual;
  tf::Vector3 x_obs;
  tf::Vector3 v_obs;
  tf::Vector3 v_obs_prev;
  tf::Vector3 x_obs_prev;
  tf::Vector3 x_actual_prev;

  ros::Time prev_time;



};

tf::Matrix3x3 fuseRotation(tf::Transform tf_by_imu, tf::Transform tf_by_tracking_room);

#endif // FLAT_CONTROLLER_NODE_H
