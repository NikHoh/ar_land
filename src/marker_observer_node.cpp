#include "ar_land/marker_observer_node.hpp"


MarkerObserver::MarkerObserver(){

  ros::NodeHandle n("~");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "board_c3po");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");
  n.param<std::string>("T_cam_board_topic", T_cam_board_topic, "/ar_single_board/transform");

  T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &MarkerObserver::getCamBoardTf, this);
  cam_to_board_tf.setIdentity();
  world_to_board_tf.setIdentity();

}

void MarkerObserver::getCamBoardTf(const geometry_msgs::TransformStamped &T_cam_board_msg)
{
  tf::transformStampedMsgToTF(T_cam_board_msg, cam_to_board_tf);

  tf::StampedTransform cam_to_drone_tf;
  tf::StampedTransform world_to_drone_tf;

  try{
    tf_listener.lookupTransform(cam_frame_id, drone_frame_id, ros::Time(0), cam_to_drone_tf);     // tf which is set up in parameter server
    tf_listener.lookupTransform(world_frame_id,drone_frame_id,ros::Time(0),world_to_drone_tf);  // tf found by odometry/tracking room
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::StampedTransform drone_to_cam_tf;
  drone_to_cam_tf.setData(cam_to_drone_tf.inverse());

  tf::Transform world_to_board;
  world_to_board = world_to_drone_tf*drone_to_cam_tf*cam_to_board_tf; // calculate world_to_board transformation

  world_to_board_tf.setData(world_to_board);
  world_to_board_tf.child_frame_id_ = board_frame_id;
  world_to_board_tf.frame_id_ = world_frame_id;
  world_to_board_tf.stamp_ = cam_to_board_tf.stamp_;


  float acc_distance = 0.5;
  float measure_confidence = 0.5;

  if(!observed_board){
    observed_board = true;
  }else{  //if board was observed at least once a previous transformation must exist

    if(cam_to_board_tf.getOrigin().length() > acc_distance){ //assume some distance for accurate measurement
      world_to_board_tf.setOrigin((measure_confidence*world_to_board_tf.getOrigin())+((1-measure_confidence)*previous_world_to_board_tf.getOrigin())); // take the average of the 2 vectors, for now.... change to some confidence level depending on measurement
      world_to_board_tf.setRotation(world_to_board_tf.getRotation().slerp(previous_world_to_board_tf.getRotation(),measure_confidence));  // computes the spherical linear interpolation, which is some rotation between the given two, change scalar to same confidence as above
    }
  }
  previous_world_to_board_tf = world_to_board_tf;
  tf_broadcaster.sendTransform(world_to_board_tf);

}


void MarkerObserver::run(double frequency)
{
  ros::NodeHandle node;
  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_observer_node");

  MarkerObserver marker_observer;
  double frequency = 30;
  marker_observer.run(frequency);
}
