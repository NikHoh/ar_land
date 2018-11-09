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

class pid_controller_node
{
public:
  pid_controller_node();
  ros::NodeHandle nh;
  pid_controller_node(const std::string& worldFrame, const std::string& frame, const ros::NodeHandle& n);

  //Functions
  void run(double frequency);
  void iteration(const ros::TimerEvent& e);

private:



  enum State
  {
    Idle = 0,
    Automatic = 1,
    TakingOff = 2,
    Landing = 3,
  };
  // Functions
  double get(const ros::NodeHandle& n, const std::string& name);
  bool takeoff( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void goalChanged(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void getTransform(const std::string& sourceFrame, const std::string& targetFrame, tf::StampedTransform& result);
  void pidReset();



  // Subscribers
  ros::Subscriber T_cam_board_sub;

  // Publisher
  ros::Publisher T_world_goal_pub;
  ros::Publisher debug_pose_pub;

  // Variables
  std::string m_world_frame_id;
  std::string m_frame;
  ros::Publisher m_pubNav;
  tf::TransformListener m_listener;
  PID m_pidX;
  PID m_pidY;
  PID m_pidZ;
  PID m_pidYaw;
  State m_state;
  geometry_msgs::PoseStamped m_goal;
  ros::Subscriber m_subscribeGoal;
  ros::Subscriber imu_subscriber; // @Niko
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  float m_thrust;
  float m_startZ;
  float imu_pos_x;
  float imu_pos_y;
  float imu_pos_z;
  float imu_ang_yaw;
  float imu_a_x;
  float imu_a_y;
  float imu_a_z;
  float imu_w_z;
  float imu_v_x;
  float imu_v_y;
  float imu_v_z;
  double previous_time;
  geometry_msgs::Vector3 a_vec;

};

#endif // PID_CONTROLLER_NODE_H
