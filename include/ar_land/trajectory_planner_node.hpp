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

class trajectory_planner_node
{
public:
  trajectory_planner_node();
  ros::NodeHandle nh;

private:
  // Functions
  void markerPoseCallback(const geometry_msgs::TransformStamped &msg);

  // Subscribers
  ros::Subscriber T_cam_board_sub;

  // Publisher
  ros::Publisher T_world_goal_pub;
  ros::Publisher debug_pose_pub;

  // Variables
  tf::TransformListener m_tf_listener;
  tf::Transform m_transform;
  tf::TransformBroadcaster tf_br;
  geometry_msgs::PoseStamped m_cf_pose;

  std::string drone_frame_id;
  std::string world_frame_id;
  std::string board_frame_id;
  std::string goal_frame_id;
  std::string cam_frame_id;

  std::string T_cam_board_topic;
  std::string T_world_goal_topic;

};

#endif // TRAJECTORY_PLANNER_NODE_H
