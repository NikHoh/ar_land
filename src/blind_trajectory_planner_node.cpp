#include "ar_land/blind_trajectory_planner_node.hpp"
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>
#include "ar_land/pid_controller_node.hpp"
#include "ar_land/tools.hpp"

blind_trajectory_planner_node::blind_trajectory_planner_node()
  : flight_state(Idle)
  , thrust(0)

{
  ROS_INFO("Im Konstruktor des blind_trjactory_planners");
  // initialize topics
  ros::NodeHandle n("~");
  // reads parameter with name (1) from parameter server and saves it in name (2), if not found default is (3)
  n.param<std::string>("T_cam_board_topic", T_cam_board_topic, "/ar_single_board/transform");
  n.param<std::string>("pose_goal_in_world_topic", pose_goal_in_world_topic, "/ar_land/T_world_goal_topic");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "board_c3po");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");

  // Subscribers
  T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &blind_trajectory_planner_node::setGoalinWorld, this); // subscribed zu (1) und führt bei empfangener Nachricht (3) damit aus
  control_out_sub = nh.subscribe("cmd_vel",1, &blind_trajectory_planner_node::getValue, this);

  // Publishers
  pose_goal_in_world_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_goal_in_world_topic, 1); // states that pose_goal_in_world_pub publishes to topic (1)
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Services
  flight_state_change_srv = nh.advertiseService("flight_state_change", &blind_trajectory_planner_node::state_change, this);

  // We publish some identity transforms and goal poses for the crazyflie controller
  // Since we are not using an external localization system, we get a
  // valid transformation tree only when the aruco_board is in the field of view
  // of the camera.

  tf::StampedTransform init_tf;
  /*
  init_tf.setIdentity(); // sets init_tf to identity transform
  init_tf.frame_id_ = world_frame_id; // this transformation is defined in the World coordinate system
  init_tf.child_frame_id_ = drone_frame_id; // diese Transformation definiert den Crazyflie Frame
  init_tf.stamp_ = ros::Time::now(); */
  //tf_br.sendTransform(init_tf); // übergibt Transfomration an den transformations broadcaster
/*
  geometry_msgs::TransformStamped T_world_goal_msg; // Definition einer TransformStamped Message aus dem geometry_msgs Paket
  geometry_msgs::PoseStamped pose_goal_in_world_msg;
  tf::transformStampedTFToMsg(init_tf, T_world_goal_msg);
  tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);
  pose_goal_in_world_pub.publish(pose_goal_in_world_msg);
*/


  // wait until tf_world_goal_pub has a Subscriber (until a picture is detected)
  // TODO: replace with subscriber callback
  ros::Rate rate(10);
  while (pose_goal_in_world_pub.getNumSubscribers() < 1)
  {
    rate.sleep();
    //tf_br.sendTransform(init_tf);
    //pose_goal_in_world_pub.publish(pose_goal_in_world_msg);
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
    nh.setParam("/ar_land/pid_controller_node/controller_enabled", true);
    ROS_INFO("State change to Automatic");
  }
    break;
  case TakingOff:
  {
    //tf::StampedTransform transform;
    //tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), transform);
    double startTime = ros::Time::now().toSec();
    while(flight_state != Automatic)
    {
      // verbessertes Startmanöver (roter Knopf drücken und dann zügig Marker unterhalten)
      if (ros::Time::now().toSec() - startTime > 1.5) // Quadrocopter ist abgehoben
      {
        ROS_INFO("1.5s vorbei. Controller wird aktiviert.");
        //pidReset();
        nh.setParam("/ar_land/pid_controller_node/z_integral", 40000);
        nh.setParam("/ar_land/pid_controller_node/controller_enabled", true);

        flight_state = Automatic;
        ROS_INFO("TakingOff done");
        //thrust = 0;
      }
      else // Quadrocopter ist noch nicht abgehoben
      {
        thrust = 40000;
        geometry_msgs::Twist msg;
        msg.linear.z = thrust;
        control_out_pub.publish(msg);

      }
    }

  }
    break;
  case Landing:
  {
    ROS_INFO("Drone is going to be landed.");
    //boost::shared_ptr<const geometry_msgs::Twist> msg = ros::topic::waitForMessage<geometry_msgs::Twist>("/crazyflie/cmd_vel_controller");    
    nh.setParam("/ar_land/pid_controller_node/resetPID", true);
    //float thrust = (*msg).linear.z;
    float thrust = last_thrust;
    ROS_INFO("Letzte Stellgroesse ausgelesen: %f", thrust);
    ros::Rate rate(10);
    geometry_msgs::Twist control_out;
    while(thrust > 0)
    {

      thrust -= 100;
      if(thrust < 36500)
      {
        thrust = 0;
      }
      control_out.linear.z = thrust;
      control_out.linear.x = 0;
      control_out.linear.y = 0;
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

void blind_trajectory_planner_node::getValue(const geometry_msgs::Twist &msg){
  last_thrust = msg.linear.z;
}


void blind_trajectory_planner_node::setGoalinWorld(const geometry_msgs::TransformStamped &T_cam_board_msg) {

  //ROS_INFO("trans: x %f y %f z %f rot w %f x %f y %f z %f", T_cam_board_msg.transform.translation.x, T_cam_board_msg.transform.translation.y,T_cam_board_msg.transform.translation.z, T_cam_board_msg.transform.rotation.w, T_cam_board_msg.transform.rotation.x, T_cam_board_msg.transform.rotation.y, T_cam_board_msg.transform.rotation.z );

  // Publishing of T_board_cam (coming from the single_board_node)
  tf::StampedTransform T_cam_board;
  tf::transformStampedMsgToTF(T_cam_board_msg, T_cam_board);
  tf::StampedTransform T_board_cam;
  T_board_cam.setData(T_cam_board.inverse());
  T_board_cam.stamp_ = T_cam_board.stamp_;
  T_board_cam.frame_id_ = board_frame_id;
  T_board_cam.child_frame_id_ = cam_frame_id;

  tf_br.sendTransform(T_board_cam); // 'publishes' the board_to_cam_tf coming from the marker detection into the tf tree, but tf can be older (lost track of marker)

  if(flight_state == Automatic)
  {
    tf::StampedTransform world_to_board_tf;
    //tf::StampedTransform board_to_cam_tf;
    tf::StampedTransform cam_to_drone_tf;
    //tf::StampedTransform cam_to_board_tf;
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
    //ROS_INFO("tf from world to goal should now be in the tf tree");

    geometry_msgs::TransformStamped T_world_goal_msg;
    geometry_msgs::PoseStamped pose_goal_in_world_msg;
    tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);
    tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);


    //ROS_INFO("Publishing transform and goal pose:%f, transform:%f", pose_goal_in_world_msg.pose.position.z, T_world_goal_msg.transform.translation.z);
    pose_goal_in_world_pub.publish(pose_goal_in_world_msg); // neccessary? it is also send by tf_broadcaster...
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "blind_trajectory_planner_node"); // initializes node named blind_trajectory_planner_node

  ros::NodeHandle n("~");
  blind_trajectory_planner_node node;                // erzeugt trajectory_planner_node Node mit dem Namen "node"
  ros::spin();

  return 0;
}
