#ifndef STATE_ADMIN_NODE_H
#define STATE_ADMIN_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


class state_admin_node
{
public:

  ros::NodeHandle nh;
  state_admin_node();



private:


  // Functions
  bool takeoff( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


  // Services
  ros::ServiceServer takeoff_srv_serv;
  ros::ServiceServer land_srv_serv;
  ros::ServiceClient flight_state_change_client;



};

#endif // STATE_ADMIN_NODE_H
