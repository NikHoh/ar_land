#include "ar_land/blind_trajectory_planner_node.hpp"
#include <tf2/transform_datatypes.h>
#include <angles/angles.h>
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
    double startTime = ros::Time::now().toSec();
    while(flight_state != Automatic)
    {
      // press red button (Logitech controller) / triangle (PS4 controller)
      if (ros::Time::now().toSec() - startTime < 1.5) // drone has not finished takeoff
      {
        thrust = 40000;
        geometry_msgs::Twist msg;
        msg.linear.z = thrust;
        control_out_pub.publish(msg);
      }
      else // drone has completed takeoff --> switch to automatic mode
      {
        nh.setParam("/ar_land/pid_controller_node/z_integral", 40000);
        nh.setParam("/ar_land/pid_controller_node/controller_enabled", true);

        flight_state = Automatic;
        ROS_INFO("TakingOff done");
      }
    }
  }
    break;
  case Landing:
  {
    nh.setParam("/ar_land/pid_controller_node/resetPID", true);
    float thrust = last_thrust;
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
    break;
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

  // Broadcasting T_board_cam as transform to achieve valid tf-tree (message comes from the single_board_node)
  tf::StampedTransform cam_to_board_tf;
  tf::transformStampedMsgToTF(T_cam_board_msg, cam_to_board_tf);

  tf::StampedTransform world_to_board_tf;
  tf::StampedTransform cam_to_drone_tf;

  try{
    tf_lis.lookupTransform(world_frame_id, board_frame_id, ros::Time(0), world_to_board_tf); // tf which is set up in parameter server
    tf_lis.lookupTransform(cam_frame_id, drone_frame_id, ros::Time(0), cam_to_drone_tf);     // tf which is set up in parameter server
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::StampedTransform drone_to_cam_tf;
  drone_to_cam_tf.setData(cam_to_drone_tf.inverse());

  tf::StampedTransform board_to_world_tf;
  board_to_world_tf.setData(world_to_board_tf.inverse());

  tf::Transform drone_to_world;
  drone_to_world = drone_to_cam_tf*cam_to_board_tf*board_to_world_tf;  //calculate drone_to_world transformation


tf::StampedTransform world_to_drone_tf;
world_to_drone_tf.setData(drone_to_world.inverse());
world_to_drone_tf.child_frame_id_ = drone_frame_id;
world_to_drone_tf.frame_id_ = world_frame_id;
world_to_drone_tf.stamp_ = cam_to_board_tf.stamp_;

  tf_br.sendTransform(world_to_drone_tf); // broadcasts the world_to_drone_tf coming from the marker detection into the tf tree, but tf can be older (if it lost track of marker)

  if(flight_state == Automatic)
  {

    tf::StampedTransform world_to_goal_tf;



    // The Goal follows ROS conventions (Z axis up, X to the right and Y to the front)
    // We set the goal above the world coordinate frame (our marker)
    tf::Vector3 goal_position(0,0,0.7);
    world_to_goal_tf.setIdentity();
    world_to_goal_tf.setOrigin(goal_position);
    world_to_goal_tf.child_frame_id_ = goal_frame_id;
    world_to_goal_tf.frame_id_ = world_frame_id;
    world_to_goal_tf.stamp_ = ros::Time::now();

    tf_br.sendTransform(world_to_goal_tf);

    geometry_msgs::TransformStamped T_world_goal_msg;
    geometry_msgs::PoseStamped pose_goal_in_world_msg;
    tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);
    tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);

    pose_goal_in_world_pub.publish(pose_goal_in_world_msg); // neccessary for pid_controller_node
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "blind_trajectory_planner_node"); // initializes node named blind_trajectory_planner_node

  ros::NodeHandle n("~");
  blind_trajectory_planner_node node;                // Creates trajectory_planner_node
  ros::spin();

  return 0;
}
