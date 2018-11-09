#include "ar_land/trajectory_planner_node.hpp"
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>


namespace tf
{
  inline
  void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
  {
      pose.orientation = trans.rotation;
      pose.position.x = trans.translation.x;
      pose.position.y = trans.translation.y;
      pose.position.z = trans.translation.z;
  }

  inline
  void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
    {
      trans.rotation = pose.orientation;
      trans.translation.x = pose.position.x;
      trans.translation.y = pose.position.y;
      trans.translation.z = pose.position.z;
  }

  inline
  void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
  {
      convert(trans.transform, pose.pose);
      pose.header = trans.header;
  }

  inline
  void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
  {
      convert(pose.pose, trans.transform);
      trans.header = pose.header;
  }
}

trajectory_planner_node::trajectory_planner_node() {
  // initialize topics
  ros::NodeHandle n("~");
  std::string s;
  // liest Parameter mit dem Namen (1) aus dem Parameterserver aus und speichert diese unter dem Namen (2), falls nicht gefunden nimmt default (3) an
  n.param<std::string>("T_cam_board_topic", T_cam_board_topic, "/ar_single_board/transform");
  n.param<std::string>("T_world_goal_topic", T_world_goal_topic, "/ar_land/T_world_goal_topic");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "/aruco_board");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");

  // Initialisierung von im Header deklarierten Subscribern und Publishern
  T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &trajectory_planner_node::markerPoseCallback, this); // subscribed zu (1) und führt bei empfangener Nachricht (3) damit aus

  // TODO: if there is a new controller node created, try to change topic type to StampedTransform
  T_world_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(T_world_goal_topic, 1); // legt fest, dass m_cf_pose_pub auf dem Topic (1) veröffentlicht

  //debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1);



  // We publish some identity transforms and goal poses for the crazyflie controller
  // Since we are not using an external localization system, we get a
  // valid transformation tree only when the aruco_board is in the field of view
  // of the camera.

  tf::StampedTransform init_tf;
  init_tf.setIdentity(); // setzt init_tf als Einheitstransformation
  init_tf.frame_id_ = world_frame_id; // diese Transformation ist im World KSY definiert
  init_tf.child_frame_id_ = drone_frame_id; // diese Transformation definiert den Crazyflie Frame
  init_tf.stamp_ = ros::Time::now();
  tf_br.sendTransform(init_tf); // übergibt Transfomration an den transformations broadcaster

  geometry_msgs::TransformStamped T_world_goal_msg; // Definition einer TransformStamped Message aus dem geometry_msgs Paket
  geometry_msgs::PoseStamped pose_goal_msg;
  tf::transformStampedTFToMsg(init_tf, T_world_goal_msg);
  tf::convert(T_world_goal_msg, pose_goal_msg);
  T_world_goal_pub.publish(pose_goal_msg);



  // wait until tf_world_goal_pub has a Subscriber (until a picture is detected)
  // TODO: replace with subscriber callback
  ros::Rate rate(10);
  while (T_world_goal_pub.getNumSubscribers() < 1)
  {
    rate.sleep();
    tf_br.sendTransform(init_tf);
    T_world_goal_pub.publish(pose_goal_msg);
  }
}

void trajectory_planner_node::markerPoseCallback(const geometry_msgs::TransformStamped &T_cam_board_msg) {
  try {   

      tf::StampedTransform world_to_board_tf;
      tf::StampedTransform board_to_cam_tf;
      tf::StampedTransform cam_to_drone_tf;
      tf::StampedTransform cam_to_board_tf;
      //tf::StampedTransform world_to_drone_tf;
      tf::StampedTransform world_to_goal_tf;

      //try{
        m_tf_listener.lookupTransform(world_frame_id, board_frame_id, ros::Time(0), world_to_board_tf);
        m_tf_listener.lookupTransform(cam_frame_id, drone_frame_id, ros::Time(0), cam_to_drone_tf);
      //}
      //catch (tf::TransformException &ex) {
      //ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep();
      //}

      tf::transformStampedMsgToTF(T_cam_board_msg, cam_to_board_tf);
      board_to_cam_tf.setData(cam_to_board_tf.inverse());


      // Actually task of a state estimator node
      /*
      world_to_drone_tf.setData(world_to_board_tf*board_to_cam_tf*cam_to_drone_tf);
      world_to_drone_tf.child_frame_id_ = drone_frame_id; // crazyflie/base_link
      world_to_drone_tf.frame_id_ = world_frame_id;  // world
      world_to_drone_tf.stamp_ = ros::Time::now();
      tf_br.sendTransform(world_to_drone_tf);
      */


      // The Goal follows ROS conventions (Z axis up, X to the righ and Y to the forwar direction of movement)
      // We Position the Goal above the world coordinate frame (our marker)
      tf::Vector3 goal_position(0,0,0.5);
      world_to_goal_tf.setIdentity();
      world_to_goal_tf.setOrigin(goal_position);
      world_to_goal_tf.child_frame_id_ = goal_frame_id;
      world_to_goal_tf.frame_id_ = world_frame_id;
      world_to_goal_tf.stamp_ = ros::Time::now();

      tf_br.sendTransform(world_to_goal_tf);

      geometry_msgs::TransformStamped T_world_goal_msg;
      geometry_msgs::PoseStamped pose_goal_msg;
      tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);
      tf::convert(T_world_goal_msg, pose_goal_msg);


      ROS_DEBUG("Publishing transform and goal pose:%f, transform:%f", pose_goal_msg.pose.position.z, T_world_goal_msg.transform.translation.z);
      T_world_goal_pub.publish(pose_goal_msg);


  }
  catch (...) {
    ROS_ERROR("Failed to publish crazyflie pose");
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_planner_node"); // initialisiert Node mit dem Namen trajectory_planner_node
  trajectory_planner_node node;                // erzeugt trajectory_planner_node Node mit dem Namen "node"
  ros::spin();                     // erhält node am Leben
  return 0;
}
