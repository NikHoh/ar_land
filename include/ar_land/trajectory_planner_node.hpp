#ifndef TRAJECTORY_PLANNER_NODE_H
#define TRAJECTORY_PLANNER_NODE_H

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





class trajectory_planner_node
{
public:
  trajectory_planner_node();
  ros::NodeHandle nh;
  void iteration(const ros::TimerEvent& e);

private:
  // Functions
  void setGoalinWorld(const geometry_msgs::TransformStamped &msg);
  bool state_change(ar_land::flight_state_changeRequest &req,
                    ar_land::flight_state_changeResponse  &res);
  void getValue(const geometry_msgs::Twist &msg);
  bool goal_change(ar_land::goal_change::Request& req, ar_land::goal_change::Response& res);
  void setTrajPoint(const ros::TimerEvent& e);



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

  // Publisher
  ros::Publisher pose_goal_in_world_pub;

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
  ros::Publisher control_out_pub;
  ros::Subscriber control_out_sub;
  State flight_state;

  double frequency;
  float dt;
  bool traj_started;

  tf::Vector3 start_position_in_board;
  ros::Time start_time;

  ros::Timer timer;


  tf::Vector3 goal_position_in_board;

  float last_thrust;
  float thrust;

};

#endif // TRAJECTORY_PLANNER_NODE_H
