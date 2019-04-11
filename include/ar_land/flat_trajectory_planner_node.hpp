#ifndef FLAT_TRAJECTORY_PLANNER_NODE_H
#define FLAT_TRAJECTORY_PLANNER_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ar_land/flight_state_change.h>
#include <ar_land/goal_change.h>
#include <math.h>
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>
#include "ar_land/tools.hpp"
#include <geometry_msgs/Accel.h>
#include <ar_land/PosVelAcc.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>




/**
 * @brief The flat_trajectory_planner_node class provides a trajectory planner that publishes trajectory points with desired velocity and acceleration.
 *
 * adaptable parameters in code are a mean velocity and start and endpoint of the trajectory.
 */

class flat_trajectory_planner_node
{
public:
  flat_trajectory_planner_node();
  ros::NodeHandle nh;
  void run(double frequency);

private:
  // Functions
  void updateBoardinWorld();
  bool state_change(ar_land::flight_state_changeRequest &req,
                    ar_land::flight_state_changeResponse  &res);
  void getImuAccelZ(const sensor_msgs::Imu::ConstPtr& msg);
  bool goal_change(ar_land::goal_change::Request& req, ar_land::goal_change::Response& res);
  void setTrajPoint(const ros::TimerEvent& e);
  void updateGoalPos();
  void receiveObserverData(const ar_land::PosVelAcc &msg);



  enum State
  {
    Idle = 0,
    Automatic = 1,
    TakingOff = 2,
    Landing = 3,
    Emergency = 4,
  };

  // Subscribers
  ros::Subscriber T_cam_board_sub;
  ros::Subscriber obs_posVelAcc_sub;

  // Publisher
  ros::Publisher pose_goal_in_world_pub;
  ros::Publisher goal_posVelAcc_pub;
  // extra goal publisher to be able to use standard msg and read data in matlab
  ros::Publisher goal_pos_pub;
  ros::Publisher goal_vel_pub;
  ros::Publisher goal_acc_pub;
  ros::Publisher path_pub, drone_path_pub;

  // Services
  ros::ServiceServer flight_state_change_srv;
  ros::ServiceServer goal_change_srv_serv;

  // Variables
  tf::TransformListener tf_lis;
  tf::TransformBroadcaster tf_br;


  std::string drone_frame_id;
  std::string world_frame_id;
  std::string board_frame_id;
  std::string goal_frame_id;
  std::string cam_frame_id;

  std::string T_cam_board_topic;
  std::string pose_goal_in_world_topic;
  std::string goal_posVelAcc_topic;
  std::string goal_pos_topic;
  std::string goal_vel_topic;
  std::string goal_acc_topic;
  ros::Publisher control_out_pub;
  ros::Subscriber control_out_sub;
  State flight_state;

  double frequency;
  float dt;
  float t_prev;
  bool run_traj;
  bool replan_traj;
  bool traj_started;
  bool traj_finished;
  bool land_straight;

  bool calc_traj_with_real_values;

  tf::Vector3 start_position_in_board;
  ros::Time start_time;
  ros::Time rost_prev;

  ros::Timer timer;


  tf::Vector3 goal_position_in_board;
  tf::Vector3 board_position_in_world;
  tf::Vector3 twist_goal_in_board;
  tf::Vector3 accel_goal_in_board;

  float last_accel_z;
  float accel_z;
  float mean_velocity; // [m/s]  travel velocity

  // observed velocity and acceleration
  double xp_obs, yp_obs, zp_obs;
  double xpp_obs, ypp_obs, zpp_obs;

  // actual position, velocity and acceleration
  double x_0, y_0, z_0;
  double xp_0, yp_0, zp_0;
  double xpp_0, ypp_0, zpp_0;

  // final position, velocity and acceleration
  double x_f_old, y_f_old, z_f_old;
  double x_f, y_f, z_f;
  double xp_f, yp_f, zp_f;
  double xpp_f, ypp_f, zpp_f;
  double  x_f_corr, y_f_corr, z_f_corr;

  float T;
  ros::Time latenz;

  bool board_moving;
  double x_f_prev,y_f_prev;
  nav_msgs::Path goal_path, drone_path;


};

#endif // FLAT_TRAJECTORY_PLANNER_NODE_H
