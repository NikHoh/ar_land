#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "ar_land/pid.hpp"
#include "ar_land/state_admin_node.hpp"
#include <ar_land/flight_state_change.h>

state_admin_node::state_admin_node()
{
  ros::NodeHandle nh;
  //tf_lis.waitForTransform(world_frame_id, drone_frame_id, ros::Time(0), ros::Duration(10.0));

  takeoff_srv_serv = nh.advertiseService("/crazyflie/takeoff", &state_admin_node::takeoff, this);
  land_srv_serv = nh.advertiseService("/crazyflie/land", &state_admin_node::land, this);
  flight_state_change_client = nh.serviceClient<ar_land::flight_state_change>("flight_state_change");
}

bool state_admin_node::takeoff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Takeoff requested!");

  ar_land::flight_state_change srv;
  srv.request.flight_state = 2;

  if(flight_state_change_client.call(srv))
  {
    ROS_INFO("Flight status change done: %d", (int)srv.response.changed);
  }

  return true;
}

bool state_admin_node::land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("Landing requested!");

  ar_land::flight_state_change srv;
  srv.request.flight_state = 3;

  if(flight_state_change_client.call(srv))
  {
    ROS_INFO("Flight status change done: %d", (int)srv.response.changed);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_admin_node");

  //ros::NodeHandle n("~"); unbenutzt

  state_admin_node state_admin();

  ros::spin();

  return 0;
}
