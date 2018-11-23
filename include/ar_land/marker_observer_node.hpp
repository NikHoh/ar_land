#ifndef MARKER_OBSERVER_NODE_H
#define MARKER_OBSERVER_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class MarkerObserver
{
public:
  MarkerObserver();
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

};



#endif // MARKER_OBSERVER_NODE_H
