#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


#include "ar_land/pid.hpp"
#include "ar_land/pid_controller_node.hpp"

namespace func {
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
  double value;
  n.getParam(name, value);
  return value;
}
}



pid_controller_node::pid_controller_node( // deklariert und initialisiert Controller Classe mit Attributen m_world_frame_id, m_frame, m_pubNav ...
                                          const std::string& world_frame_id,
                                          const std::string& drone_frame_id,
                                          const ros::NodeHandle& n)
// Attribute der Klasse Controller zuweisen (Deklaration der Attribute weiter unten
  : world_frame_id(world_frame_id)
  , drone_frame_id(drone_frame_id)
  , control_out_pub()
  , tf_lis()
  , pid_x(
      func::get(n, "PIDs/X/kp"),
      func::get(n, "PIDs/X/kd"),
      func::get(n, "PIDs/X/ki"),
      func::get(n, "PIDs/X/minOutput"),
      func::get(n, "PIDs/X/maxOutput"),
      func::get(n, "PIDs/X/integratorMin"),
      func::get(n, "PIDs/X/integratorMax"),
      "x")
  , pid_y(
      func::get(n, "PIDs/Y/kp"),
      func::get(n, "PIDs/Y/kd"),
      func::get(n, "PIDs/Y/ki"),
      func::get(n, "PIDs/Y/minOutput"),
      func::get(n, "PIDs/Y/maxOutput"),
      func::get(n, "PIDs/Y/integratorMin"),
      func::get(n, "PIDs/Y/integratorMax"),
      "y")
  , pid_z(
      func::get(n, "PIDs/Z/kp"),
      func::get(n, "PIDs/Z/kd"),
      func::get(n, "PIDs/Z/ki"),
      func::get(n, "PIDs/Z/minOutput"),
      func::get(n, "PIDs/Z/maxOutput"),
      func::get(n, "PIDs/Z/integratorMin"),
      func::get(n, "PIDs/Z/integratorMax"),
      "z")
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
  , controller_started(false)


  // Körper der Klassendefinition
{

  ros::NodeHandle nh;
  //tf_lis.waitForTransform(world_frame_id, drone_frame_id, ros::Time(0), ros::Duration(10.0)); // this transformation is a identity tf at this moment
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pose_goal_in_world_sub = nh.subscribe("/ar_land/pose_goal_in_world_topic", 1, &pid_controller_node::goalChanged, this); // subscribed die Sollwerte (Ziel KSY Pose von World aus gesehen)

  //dynamic reconfigure overwrites PID parameters immediately from cfg file
  dynamic_reconfigure::Server<
        ar_land::dynamic_param_configConfig>::CallbackType f;
    f = boost::bind(&pid_controller_node::dynamic_reconfigure_callback, this, _1, _2);
    m_server.setCallback(f);
}

void pid_controller_node::run(double frequency) // not in main() possible?
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &pid_controller_node::iteration, this);
  ros::spin();
}

void pid_controller_node::goalChanged(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_goal_in_world_msg = *msg;
}



void pid_controller_node::pidReset()
{
  pid_x.reset();
  pid_y.reset();
  pid_z.reset();
  pid_yaw.reset();
  controller_started = false;
  ROS_INFO("PID Controller reset");
}

void pid_controller_node::pidStart()
{
  if(!controller_started)
  {
    nh.getParam("z_integral", z_integral);

    pid_z.setIntegral(z_integral/ pid_z.ki());
    ROS_INFO("PID is getting started");
  }
  controller_started = true;

}

void pid_controller_node::iteration(const ros::TimerEvent& e)
{
  nh.param<bool>("controller_enabled", controller_enabled, false);
ROS_DEBUG("Controller enabled? %d", controller_enabled);
  if(controller_enabled){


    //float dt = e.current_real.toSec() - e.last_real.toSec();

    tf::StampedTransform tf_world_to_drone;
    tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone); // TODO try catch einfügen

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
    control_out.linear.x = pid_x.update(0.0, pose_goal_in_drone.pose.position.x);
    control_out.linear.y = pid_y.update(0.0, pose_goal_in_drone.pose.position.y);
    control_out.linear.z = pid_z.update(0.0, pose_goal_in_drone.pose.position.z);
    control_out.angular.z = pid_yaw.update(0.0, yaw);
    ROS_INFO("P: %d I: %d D: %d Out: %d ", pid_x.getP(), pid_x.getI(), pid_x.getD(), pid_x.getOutput() );

    control_out_pub.publish(control_out);
    ROS_INFO("Controller published control values");
  }

}

void pid_controller_node::dynamic_reconfigure_callback(
    ar_land::dynamic_param_configConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f", config.Kp_x, config.Ki_x, config.Kd_x);
  // Coefficients for the PID controller
  pid_x.setKP(config.Kp_x);
  pid_x.setKI(config.Ki_x);
  pid_x.setKD(config.Kd_x);

  pid_y.setKP(config.Kp_y);
  pid_y.setKI(config.Ki_y);
  pid_y.setKD(config.Kd_y);

  pid_z.setKP(config.Kp_z);
  pid_z.setKI(config.Ki_z);
  pid_z.setKD(config.Kd_z);

  pid_yaw.setKP(config.Kp_yaw);
  pid_yaw.setKI(config.Ki_yaw);
  pid_yaw.setKD(config.Kd_yaw);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller_node");

  // Read parameters
  ros::NodeHandle n("~");
  std::string world_frame_id;
  std::string drone_frame_id;
  double frequency;

  n.param<std::string>("world_frame_id", world_frame_id, "/world"); // liest Parameter (1) vom Parameter Server aus und speichert in (2)

  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");

  n.param<double>("frequency", frequency, 50.0);

  pid_controller_node controller(world_frame_id, drone_frame_id, n);
  controller.run(frequency);

  return 0;
}
