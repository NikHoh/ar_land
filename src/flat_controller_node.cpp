#include "ar_land/flat_controller_node.hpp"

namespace func {
double get(
    const ros::NodeHandle& n,
    const std::string& name)
{
  double value;
  n.getParam(name, value);
  return value;
}
}
flat_controller_node::flat_controller_node( const std::string& world_frame_id,
                                          const std::string& drone_frame_id,
                                          const ros::NodeHandle& n)
  : world_frame_id(world_frame_id)
  , drone_frame_id(drone_frame_id)
  , control_out_pub()
  , tf_lis()
  , pid_yaw(
      func::get(n, "PIDs/Yaw/kp"),
      func::get(n, "PIDs/Yaw/kd"),
      func::get(n, "PIDs/Yaw/ki"),
      func::get(n, "PIDs/Yaw/minOutput"),
      func::get(n, "PIDs/Yaw/maxOutput"),
      func::get(n, "PIDs/Yaw/integratorMin"),
      func::get(n, "PIDs/Yaw/integratorMax"),
      "yaw")
  , pose_goal_in_world_msg()
  , pose_goal_in_world_sub()
  , PosVelAcc_sub()
  , controller_started(false)
  , x_actual_prev(null)
  , resetPID(false)

  // Body of class definition
{

  ros::NodeHandle nh;

  // Publishers
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  control_error_pub = nh.advertise<geometry_msgs::Vector3>("control_error_topic", 1);

  // Subscribers
  //pose_goal_in_world_sub = nh.subscribe("/ar_land/pose_goal_in_world_topic", 1, &flat_controller_node::goalChanged, this);
  PosVelAcc_sub = nh.subscribe("/ar_land/PosVelAcc_topic", 1, &flat_controller_node::receiveTrajectory, this);


  //dynamic reconfigure
  //dynamic reconfigure overwrites PID parameters immediately from cfg file
  dynamic_reconfigure::Server<ar_land::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&flat_controller_node::dynamic_reconfigure_callback, this, _1, _2);
  m_server.setCallback(f);
}

void flat_controller_node::run(double frequency)
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &flat_controller_node::iteration, this);
  ros::spin();

}

void flat_controller_node::receiveTrajectory(const ar_land::PosVelAcc::ConstPtr& msg)
{
  posVelAcc_goal_in_world_msg = *msg;
}

void flat_controller_node::pidReset()
{
  //pid_x.reset();
  //pid_y.reset();
  //pid_z.reset();
  pid_yaw.reset();
  controller_started = false;
  controller_enabled = false;
  nh.setParam("/ar_land/flat_controller_node/controller_enabled", false);
  nh.setParam("/ar_land/flat_controller_node/resetPID", false);
}

void flat_controller_node::pidStart()
{
  if(!controller_started && controller_enabled)
  {
    nh.getParam("/ar_land/flat_controller_node/z_integral", z_integral);
    //pid_x.set(0);
    //pid_y.set(0);
    //pid_z.set(z_integral/ pid_z.ki());
    pid_yaw.set(0);
  }
  controller_started = true;
}

void flat_controller_node::iteration(const ros::TimerEvent& e)
{
  nh.param<bool>("/ar_land/flat_controller_node/controller_enabled", controller_enabled, false);

  if(controller_enabled){

    if(!controller_started)
    {
      flat_controller_node::pidStart();
    }

    flat_controller_node::getActualPosVel();



      /*
      geometry_msgs::PoseStamped pose_goal_in_world;
      pose_goal_in_world.header.stamp = tf_world_to_drone.stamp_;
      pose_goal_in_world.header.frame_id = world_frame_id;
      pose_goal_in_world.pose = pose_goal_in_world_msg.pose;

      geometry_msgs::PoseStamped pose_goal_in_drone;
      tf_lis.transformPose(drone_frame_id, pose_goal_in_world, pose_goal_in_drone);

      tfScalar roll, pitch, yaw;
      tf::Matrix3x3(
            tf::Quaternion(
              pose_goal_in_drone.pose.orientation.x,
              pose_goal_in_drone.pose.orientation.y,
              pose_goal_in_drone.pose.orientation.z,
              pose_goal_in_drone.pose.orientation.w
              )).getRPY(roll, pitch, yaw);

      geometry_msgs::Twist control_out;
      //control_out.linear.x = pid_x.update(0.0, pose_goal_in_drone.pose.position.x); // update(float value, float target_value)
      //control_out.linear.y = pid_y.update(0.0, pose_goal_in_drone.pose.position.y);
      //control_out.linear.z = pid_z.update(0.0, pose_goal_in_drone.pose.position.z);
      control_out.angular.z = pid_yaw.update(0.0, yaw);

      geometry_msgs::Vector3 control_error_msg;
      //control_error_msg.x = pid_x.getError();
      //control_error_msg.y = pid_y.getError();
      //control_error_msg.z = pid_z.getError();

      control_error_pub.publish(control_error_msg);
      ROS_INFO("E: %f P: %f I: %f D: %f Out: %f ", pid_yaw.getError(), pid_yaw.getP(), pid_yaw.getI(), pid_yaw.getD(), pid_yaw.getOutput() );

      control_out_pub.publish(control_out);
      */

  }

  nh.getParam("/ar_land/flat_controller_node/resetPID", resetPID);
  if(resetPID)
  {
    flatS_controller_node::pidReset();
    resetPID = false;
  }



}

void flat_controller_node::getActualPosVel(){



  tf::StampedTransform tf_world_to_drone;
  try{
    tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
  }
  catch(tf::TransformException &ex)
  {
    ROS_INFO("No Transformation from World(Board) to Drone found");

  }

  x_actual = tf_world_to_drone.getOrigin();
  if(x_actual_prev == null) {                           // also sufficient if one lands several times ? maybe needs to be reseted better
    x_actual_prev = x_actual;
  }
  v_actual = (x_actual - x_actual_prev)*30;


  x_actual_prev = x_actual;
}

void flat_controller_node::dynamic_reconfigure_callback(
      ar_land::dynamic_param_configConfig& config, uint32_t level) {

  ROS_INFO("Reconfigure Request: %f %f %f", config.Kp_x, config.Ki_x, config.Kd_x);

  // Coefficients for the PID controller
  //pid_x.setKP(config.Kp_x);
  //pid_x.setKI(config.Ki_x);
  //pid_x.setKD(config.Kd_x);

  //pid_y.setKP(config.Kp_y);
  //pid_y.setKI(config.Ki_y);
  //pid_y.setKD(config.Kd_y);

  //pid_z.setKP(config.Kp_z);
  //pid_z.setKI(config.Ki_z);
  //pid_z.setKD(config.Kd_z);

  pid_yaw.setKP(config.Kp_yaw);
  pid_yaw.setKI(config.Ki_yaw);
  pid_yaw.setKD(config.Kd_yaw);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flat_controller_node");

  // Read parameters
  ros::NodeHandle n("~");
  std::string world_frame_id;
  std::string drone_frame_id;
  double frequency;

  n.param<std::string>("world_frame_id", world_frame_id, "/world"); // liest Parameter (1) vom Parameter Server aus und speichert in (2)

  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");

  n.param<double>("frequency", frequency, 30.0);

  flat_controller_node controller(world_frame_id, drone_frame_id, n);
  controller.run(frequency);

  return 0;
}
