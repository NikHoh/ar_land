#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>


class JoyController
{
private:
  ros::NodeHandle nh;
  ros::Subscriber joy_subscriber_;

  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist velocity_;

  ros::ServiceClient land_client;
  ros::ServiceClient takeoff_client;
  ros::ServiceClient emergency_client;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  double frequency_;

public:

  JoyController()
  {
    ros::NodeHandle params("~");

    params.param<int>("x_axis", axes_.x.axis, 4);
    params.param<int>("y_axis", axes_.y.axis, 3);
    params.param<int>("z_axis", axes_.z.axis, 2);
    params.param<int>("yaw_axis", axes_.yaw.axis, 1);

    params.param<double>("yaw_velocity_max", axes_.yaw.max, 90.0 * M_PI / 180.0);

    params.param<double>("x_velocity_max", axes_.x.max, 2.0);
    params.param<double>("y_velocity_max", axes_.y.max, 2.0);
    params.param<double>("z_velocity_max", axes_.z.max, 2.0);

    params.param<double>("frequency", frequency_, 100);

    joy_subscriber_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&JoyController::joyCallback, this, _1));
    velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ROS_INFO("joy control hello");
    if(land_client = nh.serviceClient<std_srvs::Empty>("/crazyflie/land"))
      ROS_INFO("Found Service \" land \" ");
    if(takeoff_client = nh.serviceClient<std_srvs::Empty>("/crazyflie/takeoff"))
          ROS_DEBUG("Found Service \" takeoff \" ");
    emergency_client = nh.serviceClient<std_srvs::Empty>("/crazyflie/emergeny");
  }


  ~JoyController()
  {
    stop();
  }

  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    velocity_.linear.x = getAxis(joy, axes_.x);
    velocity_.linear.y = getAxis(joy, axes_.y);
    velocity_.linear.z = getAxis(joy, axes_.z);
    velocity_.angular.z = getAxis(joy, axes_.yaw);

    //Define buttons and actions
    bool buttonLand = (bool) getButton(joy,1);
    bool buttonEmergency = (bool) getButton(joy,2);
    bool buttonTakeoff = (bool) getButton(joy,3);

    std_srvs::Empty srv;
    if(buttonLand){
      land_client.call(srv);
    }else if(buttonEmergency){
      emergency_client.call(srv);
    }
    else if(buttonTakeoff){
      takeoff_client.call(srv);
    }
  }

  void execute()
  {
    ros::Rate loop_rate(frequency_);
    while (ros::ok()) {
      velocity_publisher_.publish(velocity_);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0) {
      return 0;
    }
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0) {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size()) {
      return 0;
    }
    return sign * joy->axes[axis.axis - 1] * axis.max;
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0) {
      return 0;
    }
    if ((size_t) button > joy->buttons.size()) {
      return 0;
    }
    return joy->buttons[button - 1];
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0) {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_);
    }
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_control");
  JoyController jcont;
  jcont.execute();

  ROS_INFO("Joy Control started");
}
