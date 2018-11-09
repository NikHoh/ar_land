#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

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



 pid_controller_node::pid_controller_node(                             // deklariert und initialisiert Controller Classe mit Attributen m_world_frame_id, m_frame, m_pubNav ...
        const std::string& world_frame_id,
        const std::string& drone_frame_id,
        const ros::NodeHandle& n)
    // Attribute der Klasse Controller zuweisen (Deklaration der Attribute weiter unten
        : m_world_frame_id(world_frame_id)
        , m_frame(drone_frame_id)
        , m_pubNav()
        , m_listener()
        , m_pidX(
            func::get(n, "PIDs/X/kp"),
            func::get(n, "PIDs/X/kd"),
            func::get(n, "PIDs/X/ki"),
            func::get(n, "PIDs/X/minOutput"),
            func::get(n, "PIDs/X/maxOutput"),
            func::get(n, "PIDs/X/integratorMin"),
            func::get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            func::get(n, "PIDs/Y/kp"),
            func::get(n, "PIDs/Y/kd"),
            func::get(n, "PIDs/Y/ki"),
            func::get(n, "PIDs/Y/minOutput"),
            func::get(n, "PIDs/Y/maxOutput"),
            func::get(n, "PIDs/Y/integratorMin"),
            func::get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            func::get(n, "PIDs/Z/kp"),
            func::get(n, "PIDs/Z/kd"),
            func::get(n, "PIDs/Z/ki"),
            func::get(n, "PIDs/Z/minOutput"),
            func::get(n, "PIDs/Z/maxOutput"),
            func::get(n, "PIDs/Z/integratorMin"),
            func::get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            func::get(n, "PIDs/Yaw/kp"),
            func::get(n, "PIDs/Yaw/kd"),
            func::get(n, "PIDs/Yaw/ki"),
            func::get(n, "PIDs/Yaw/minOutput"),
            func::get(n, "PIDs/Yaw/maxOutput"),
            func::get(n, "PIDs/Yaw/integratorMin"),
            func::get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
        , imu_v_x(0)
        , imu_v_y(0)
        , imu_v_z(0)
        , imu_pos_x(0)
        , imu_pos_y(0)
        , imu_pos_z(0)
        , imu_ang_yaw(0)
        , previous_time(0)
        // Körper der Klassendefinition
    {

        ros::NodeHandle nh;
        m_listener.waitForTransform(m_world_frame_id, m_frame, ros::Time(0), ros::Duration(10.0));
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &pid_controller_node::goalChanged, this); // subscribed die Sollwerte (Ziel KSY Pose von World aus gesehen)

        m_serviceTakeoff = nh.advertiseService("takeoff", &pid_controller_node::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &pid_controller_node::land, this);
    }



    void pid_controller_node::run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &pid_controller_node::iteration, this);
        ros::spin();
    }


    void pid_controller_node::goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }


    bool pid_controller_node::takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_world_frame_id, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool pid_controller_node::land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void pid_controller_node::getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pid_controller_node::pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void pid_controller_node::iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_world_frame_id, m_frame, ros::Time(0), transform);
                static float startTime = ros::Time::now().toSec();

                // verbessertes Startmanöver (roter Knopf drücken und dann zügig Marker unterhalten)
                if (ros::Time::now().toSec() - startTime > 0.25) // Quadrocopter ist abgehoben
                {
                    pidReset();
                    m_pidZ.setIntegral(35260/ m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else // Quadrocopter ist noch nicht abgehoben
                {
                    m_thrust = 45000;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }
                /*if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000) // Quadrocopter ist abgehoben
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else // Quadrocopter ist noch nicht abgehoben
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }
                */

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_world_frame_id, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_world_frame_id, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_world_frame_id;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
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
