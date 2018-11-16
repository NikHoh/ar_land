#include "ar_land/blind_trajectory_planner_node.hpp"
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

blind_trajectory_planner_node::blind_trajectory_planner_node()
  : flight_state(Idle)
  , thrust(0)
  , start_pos_z(0)

{
  // initialize topics
  ros::NodeHandle n("~");
  std::string s;
  // liest Parameter mit dem Namen (1) aus dem Parameterserver aus und speichert diese unter dem Namen (2), falls nicht gefunden nimmt default (3) an
  n.param<std::string>("T_cam_board_topic", T_cam_board_topic, "/ar_single_board/transform");
  n.param<std::string>("pose_goal_in_world_topic", pose_goal_in_world_topic, "/ar_land/T_world_goal_topic");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "/aruco_board");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");
  //n.param<int>("flight_state", flight_state, 0);


  // Initialisierung von im Header deklarierten Subscribern und Publishern
  T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &blind_trajectory_planner_node::setGoalinWorld, this); // subscribed zu (1) und führt bei empfangener Nachricht (3) damit aus

  // TODO: if there is a new controller node created, try to change topic type to StampedTransform
  pose_goal_in_world_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_goal_in_world_topic, 1); // legt fest, dass m_cf_pose_pub auf dem Topic (1) veröffentlicht

  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // TODO cmd_vel umbenennen
  //debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug_pose", 1);

  flight_state_change_srv = nh.advertiseService("flight_state_change", &blind_trajectory_planner_node::state_change, this);





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
  geometry_msgs::PoseStamped pose_goal_in_world_msg;
  tf::transformStampedTFToMsg(init_tf, T_world_goal_msg);
  tf::convert(T_world_goal_msg, pose_goal_in_world_msg);
  pose_goal_in_world_pub.publish(pose_goal_in_world_msg);



  // wait until tf_world_goal_pub has a Subscriber (until a picture is detected)
  // TODO: replace with subscriber callback
  ros::Rate rate(10);
  while (pose_goal_in_world_pub.getNumSubscribers() < 1)
  {
    rate.sleep();
    tf_br.sendTransform(init_tf);
    pose_goal_in_world_pub.publish(pose_goal_in_world_msg);
  }
}



bool blind_trajectory_planner_node::state_change(ar_land::flight_state_changeRequest &req,
                                                 ar_land::flight_state_changeResponse  &res)
{
  ROS_INFO("State change requested");
  flight_state = State(req.flight_state);
  res.changed = 1;

  switch(flight_state)
  {
  case Idle:
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    control_out_pub.publish(msg);
    ROS_INFO("State change to Idle");
  }
    break;
  case Automatic:
  {
    nh.setParam("controller_enabled", true);
    ROS_INFO("State change to Automatic");
  }
    break;
  case TakingOff:
  {
    //tf::StampedTransform transform;
    //tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), transform);
    static double startTime = ros::Time::now().toSec();
    while(flight_state != Automatic)
    {
      // verbessertes Startmanöver (roter Knopf drücken und dann zügig Marker unterhalten)
      if (ros::Time::now().toSec() - startTime > 0.25) // Quadrocopter ist abgehoben
      {
        //pidReset();
        nh.setParam("z_integral", 35260);
        nh.setParam("controller_enabled", true);

        flight_state = Automatic;
        ROS_INFO("TakingOff done");
        //thrust = 0;
      }
      else // Quadrocopter ist noch nicht abgehoben
      {
        thrust = 45000;
        geometry_msgs::Twist msg;
        msg.linear.z = thrust;
        control_out_pub.publish(msg);

      }
    }

  }
    break;
  case Landing:
  {
    boost::shared_ptr<const geometry_msgs::Twist> msg = ros::topic::waitForMessage<geometry_msgs::Twist>("cmd_vel");
    float thrust = (*msg).linear.z;
    ROS_INFO("Letzte Stellgröße ausgelesen: %f", thrust);
    ros::Rate rate(10);
    geometry_msgs::Twist control_out;
    while(thrust > 0)
    {

      thrust -= 100;
      if(thrust < 0)
      {
        thrust = 0;
      }
      control_out.linear.z = thrust;
      control_out_pub.publish(control_out);
      rate.sleep();
    }

    flight_state = Idle;


  }
    break; // wasn't here in the original version
  default:
  {
    ROS_ERROR("Flight state not set");
  }
    break;
  }
  return true;
}





void blind_trajectory_planner_node::setGoalinWorld(const geometry_msgs::TransformStamped &T_cam_board_msg) {

  tf_br.sendTransform(T_cam_board_msg); // 'publishes' the cam_to_board_tf coming from the marker detection into the tf tree
  ROS_INFO("tf from cam to board should now be in the tf tree");
  if(flight_state == Automatic)
  {
    tf::StampedTransform world_to_board_tf;
    tf::StampedTransform board_to_cam_tf;
    tf::StampedTransform cam_to_drone_tf;
    tf::StampedTransform cam_to_board_tf;
    //tf::StampedTransform world_to_drone_tf;
    tf::StampedTransform world_to_goal_tf;

    //try{
    tf_lis.lookupTransform(world_frame_id, board_frame_id, ros::Time(0), world_to_board_tf); // tf which is set up in parameter server
    tf_lis.lookupTransform(cam_frame_id, drone_frame_id, ros::Time(0), cam_to_drone_tf);     // tf which is set up in parameter server
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
    tf::Vector3 goal_position(0,0,0.7);
    world_to_goal_tf.setIdentity();
    world_to_goal_tf.setOrigin(goal_position);
    world_to_goal_tf.child_frame_id_ = goal_frame_id;
    world_to_goal_tf.frame_id_ = world_frame_id;
    world_to_goal_tf.stamp_ = ros::Time::now();

    tf_br.sendTransform(world_to_goal_tf);
    ROS_INFO("tf from world to goal should now be in the tf tree");

    geometry_msgs::TransformStamped T_world_goal_msg;
    geometry_msgs::PoseStamped pose_goal_in_world_msg;
    tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);
    tf::convert(T_world_goal_msg, pose_goal_in_world_msg);


    ROS_INFO("Publishing transform and goal pose:%f, transform:%f", pose_goal_in_world_msg.pose.position.z, T_world_goal_msg.transform.translation.z);
    pose_goal_in_world_pub.publish(pose_goal_in_world_msg);
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "blind_trajectory_planner_node"); // initialisiert Node mit dem Namen trajectory_planner_node

  ros::NodeHandle n("~");

  blind_trajectory_planner_node node();                // erzeugt trajectory_planner_node Node mit dem Namen "node"

  ros::spin();

  return 0;
}
