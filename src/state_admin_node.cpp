#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "ar_land/state_admin_node.hpp"
#include <ar_land/flight_state_change.h>
#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Land.h>


state_admin_node::state_admin_node()
{
  ROS_INFO("Im Konstruktor der State Admin Node");
  //ros::NodeHandle n;
  //tf_lis.waitForTransform(world_frame_id, drone_frame_id, ros::Time(0), ros::Duration(10.0));
  takeoff_srv_serv = nh.advertiseService("/ar_land/takeoff", &state_admin_node::takeoff, this);
  land_srv_serv = nh.advertiseService("/ar_land/land", &state_admin_node::land, this);
  flight_state_change_client = nh.serviceClient<ar_land::flight_state_change>("flight_state_change");
}

bool state_admin_node::takeoff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("State admin node: Takeoff requested!");

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
  ROS_INFO("State admin node: Landing requested!");

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
ROS_INFO("In der main der stade_admin_node");
  state_admin_node admin_node;

  ros::spin();

  return 0;
}
