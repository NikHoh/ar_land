#ifndef MARKER_OBSERVER_NODE_H
#define MARKER_OBSERVER_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class MarkerObserver
{
public:
  MarkerObserver();
  void MarkerObserver::updateMarkerPose(const geometry_msgs::TransformStamped &T_cam_board_msg);
  void run();

private:
  ros::NodeHandle nh;

  // Transforms
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  std::string drone_frame_id;
  std::string world_frame_id;
  std::string board_frame_id;
  std::string goal_frame_id;
  std::string cam_frame_id;

  std::string T_cam_board_topic;

  // Subscribers
  ros::Subscriber T_cam_board_sub;

};



#endif // MARKER_OBSERVER_NODE_H
