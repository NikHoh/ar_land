#include "ar_land/marker_observer_node.hpp"


MarkerObserver::MarkerObserver(){

  ros::NodeHandle n("~");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "board_c3po");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");
}

void MarkerObserver::run(){
  ros::Rate rate(10.0); // in Hertz
   while (nh.ok()){
     tf::StampedTransform transform;
     try{
       tf_listener.lookupTransform(world_frame_id, board_frame_id,
                                ros::Time(0), transform); // pick latest availible transform
       tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame_id, board_frame_id));
     }
     catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
     }
     rate.sleep();


   }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_observer_node");

  MarkerObserver marker_observer;
  marker_observer.run();

}
