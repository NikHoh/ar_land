#include "ar_land/state_admin_node.hpp"

state_admin_node::state_admin_node()
{
  takeoff_srv_serv = nh.advertiseService("/ar_land/takeoff", &state_admin_node::takeoff, this);
  land_srv_serv = nh.advertiseService("/ar_land/land", &state_admin_node::land, this);
  emergency_srv_serv = nh.advertiseService("/ar_land/emergency", &state_admin_node::emergency, this);
  flight_state_change_client = nh.serviceClient<ar_land::flight_state_change>("flight_state_change");
  nh.param<std::string>("world_frame_id", world_frame_id, "/world");
  nh.param<std::string>("board_frame_id", board_frame_id, "board_c3po");
}

bool state_admin_node::takeoff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

  ar_land::flight_state_change srv;
  srv.request.flight_state = 2;

  flight_state_change_client.call(srv);

  return true;
}

bool state_admin_node::land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{

  ar_land::flight_state_change srv;
  srv.request.flight_state = 3;

  tf::StampedTransform world_to_board_tf;

  try{
    tf_lis.lookupTransform(world_frame_id, board_frame_id, ros::Time(0), world_to_board_tf); // tf which comes from the camera
  }
  catch (tf::TransformException &ex) {
      ROS_ERROR("No Transformation to Markerboard!");
          return false;
  }
  flight_state_change_client.call(srv);

  return true;

}

bool state_admin_node::emergency(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ar_land::flight_state_change srv;
  srv.request.flight_state = 4;

  flight_state_change_client.call(srv);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_admin_node");
  state_admin_node admin_node;
  ros::spin();

  return 0;
}
