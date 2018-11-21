#ifndef STATE_ADMIN_NODE_H
#define STATE_ADMIN_NODE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ar_land/flight_state_change.h>


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
