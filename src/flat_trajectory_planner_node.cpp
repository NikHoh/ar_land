#include "ar_land/flat_trajectory_planner_node.hpp"


flat_trajectory_planner_node::flat_trajectory_planner_node()
  : flight_state(Idle)
  , thrust(0)

{
  ROS_INFO("Im Konstruktor des trjactory_planners");
  // initialize topics
  ros::NodeHandle n("~");
  // reads parameter with name (1) from parameter server and saves it in name (2), if not found default is (3)
  n.param<std::string>("T_cam_board_topic", T_cam_board_topic, "/ar_single_board/transform");
  n.param<std::string>("goal_posVelAcc_topic", goal_posVelAcc_topic, "/ar_land/goal_posVelAcc_topic");
  n.param<std::string>("world_frame_id", world_frame_id, "/world");
  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");
  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");
  n.param<std::string>("board_frame_id", board_frame_id, "board_c3po");
  n.param<std::string>("cam_frame_id", cam_frame_id, "/cam");

  // Subscribers
  //T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &flat_trajectory_planner_node::setGoalinWorld, this); // subscribed zu (1) und führt bei empfangener Nachricht (3) damit aus
  control_out_sub = nh.subscribe("cmd_vel",1, &flat_trajectory_planner_node::getValue, this);
  obs_posVelAcc_sub = nh.subscribe("obs_posVelAcc_topic", 1, &flat_trajectory_planner_node::receiveObserverData, this);
  // Publishers
  goal_posVelAcc_pub = nh.advertise<ar_land::PosVelAcc>(goal_posVelAcc_topic, 1);
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Services
  flight_state_change_srv = nh.advertiseService("flight_state_change", &flat_trajectory_planner_node::state_change, this);
  goal_change_srv_serv = nh.advertiseService("/ar_land/goal_change", &flat_trajectory_planner_node::goal_change, this);

  goal_position_in_board.setValue(0,0,0.7);
  frequency = 30;
  run_traj = false;
  traj_started = false;
  traj_finished = false;
  calc_traj_with_real_values = false;
  landing = false;
  dt = 0;

  xp_0 = 0;
  yp_0 = 0;
  zp_0 = 0;
  xpp_0 = 0;
  ypp_0 = 0;
  zpp_0 = 0;

  xp_f = 0;
  yp_f = 0;
  zp_f = 0;
  xpp_f = 0;
  ypp_f = 0;
  zpp_f = 0;
  T = 0;


}

bool flat_trajectory_planner_node::state_change(ar_land::flight_state_changeRequest &req,
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

    ROS_INFO("State change to Automatic");
  }
    break;
  case TakingOff:
  {

    // set 0.5m above world frame as takeoff goal
    x_f = 0.0;
    y_f = 0.0;
    z_f = 0.5;

    nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
    nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
    nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);
    traj_started = false;
    traj_finished = false;
    run_traj = true;

    /*
    double startTime = ros::Time::now().toSec();
    while(flight_state != Automatic)
    {
      // press red button (Logitech controller) / triangle (PS4 controller)
      if (ros::Time::now().toSec() - startTime < 1.5) // drone has not finished takeoff
      {
        thrust = 45500;
        geometry_msgs::Twist msg;
        msg.linear.z = thrust;
        control_out_pub.publish(msg);
      }
      else // drone has completed takeoff --> switch to automatic mode
      {
        //nh.setParam("/ar_land/flat_controller_node/z_integral", 44500);
        nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);

        flight_state = Automatic;
        ROS_INFO("TakingOff done");
      }
    }*/
  }
    break;
  case Landing:
  {

    x_f = board_position_in_world.x();
    y_f = board_position_in_world.y();
    z_f = board_position_in_world.z();

    // timer for updating current board position while landing


    landing = true;

    nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
    nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
    nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);
    traj_started = false;
    traj_finished = false;
    run_traj = true;

    // detect case landed and set run_traj = false;
    // TODO... Wenn LandeTrajektorie fertig tue etwas


    /*if(t_ratio > 1) {
      goal_position_in_board = P_d;
      timer.stop();
      timer.setPeriod(ros::Duration(0),true);
      // the following is actually not very nice and should be done in the Landing Case
      geometry_msgs::Twist control_out;
      nh.setParam("/ar_land/flat_controller_node/resetPID", true);
      nh.setParam("/ar_land/flat_controller_node/controller_enabled", false);
      control_out.linear.z = 0;
      control_out.linear.x = 0;
      control_out.linear.y = 0;
      control_out_pub.publish(control_out);
      flight_state = Idle;
      traj_started = false;
      ROS_INFO("Landing accomplished");
}*/
  }
    break;
  case Emergency:
  {
    nh.setParam("/ar_land/flat_controller_node/resetPID", true);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", false);
    traj_started = false;
    traj_finished = false;
    run_traj = false;
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    control_out_pub.publish(msg);
    ROS_INFO("State change to Emergency");
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

void flat_trajectory_planner_node::updateGoalPos()
{

  x_f = board_position_in_world.x();
  y_f = board_position_in_world.y();
  z_f = board_position_in_world.z();
  nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
  nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
  nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
}

void flat_trajectory_planner_node::setTrajPoint(const ros::TimerEvent& e)
{
  updateBoardinWorld(); // updates current board position all the time

  //double latency_time = ros::Time::now().toSec(); // for debugging purposes
  if(landing)
  {
    updateGoalPos(); // sets the current board position as goal position
    ROS_INFO("Set new goal to (%0.2f, %0.2f, %0.2f)", x_f, y_f, z_f);
  }  

  if(run_traj)
  {
    float vel = 0.2; // [m/s]
    if(!traj_started)
    {
      // initializes start of completely new commanded trajectory with zero velocities and accelerations and actual position of drone
      start_time = ros::Time::now();
      traj_started = true;
      xp_0 = 0.0;
      xpp_0 = 0.0;
      yp_0 = 0.0;
      ypp_0 = 0.0;
      zp_0 = 0.0;
      zpp_0 = 0.0;

      // actual position of drone

      tf::StampedTransform tf_world_to_drone;
      try{
        tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
      }
      catch(tf::TransformException &ex)
      {
        ROS_INFO("No Transformation from World to Drone found");
      }
      x_0 = tf_world_to_drone.getOrigin().x();
      y_0 = tf_world_to_drone.getOrigin().y();
      z_0 = tf_world_to_drone.getOrigin().z();
    }

    if(!traj_finished)
    {
      double t = 1.0/30;

      T = tf::Vector3(x_0-x_f, y_0-y_f, z_0-z_f).length()/vel;

      if(t>T)
      {
        t = T; // ensures that last point of trajectory is calculated properly if calc_traj_with_real_values == true
      }

tf::Vector3 T_matrix_1 = tf::Vector3(720, -360*T, 60*pow(T,2));
tf::Vector3 T_matrix_2 = tf::Vector3(-360*T,       168*pow(T,2),  -24*pow(T,3));
tf::Vector3 T_matrix_3 = tf::Vector3(60*pow(T,2),  -24*pow(T,3),   3*pow(T,4));

      // x trajectory

      tf::Vector3 delta_pvax;
      delta_pvax.setValue(x_f-x_0-xp_0*T-0.5*xpp_0*pow(T,2),
                          xp_f-xp_0-xpp_0*T,
                          xpp_f-xpp_0);

      double alpx =  (1.0/pow(T,5))*T_matrix_1.dot(delta_pvax);
      double betx =  (1.0/pow(T,5))*T_matrix_2.dot(delta_pvax);
      double gamx =  (1.0/pow(T,5))*T_matrix_3.dot(delta_pvax);

      double x_out = alpx*pow(t,5)/120+betx*pow(t,4)/24+gamx*pow(t,3)/6+xpp_0*pow(t,2)/2+xp_0*t+x_0;
      double xp_out =  alpx*pow(t,4)/24+betx*pow(t,3)/6+gamx*pow(t,2)/2+xpp_0*t/2+xp_0;
      double xpp_out = alpx*pow(t,3)/6+betx*pow(t,2)/2+gamx*t+xpp_0;

      // y trajectory

      tf::Vector3 delta_pvay;
      delta_pvay.setValue(y_f-y_0-yp_0*T-0.5*ypp_0*pow(T,2),
                          yp_f-yp_0-ypp_0*T,
                          ypp_f-ypp_0);

      double alpy =  (1.0/pow(T,5))*T_matrix_1.dot(delta_pvay);
      double bety =  (1.0/pow(T,5))*T_matrix_2.dot(delta_pvay);
      double gamy =  (1.0/pow(T,5))*T_matrix_3.dot(delta_pvay);

      double y_out = alpy*pow(t,5)/120+bety*pow(t,4)/24+gamy*pow(t,3)/6+ypp_0*pow(t,2)/2+yp_0*t+y_0;
      double yp_out = alpy*pow(t,4)/24+bety*pow(t,3)/6+gamy*pow(t,2)/2+ypp_0*t/2+yp_0;
      double ypp_out = alpy*pow(t,3)/6+bety*pow(t,2)/2+gamy*t+ypp_0;

      // z trajectory

      tf::Vector3 delta_pvaz;
      delta_pvaz.setValue(z_f-z_0-zp_0*T-0.5*zpp_0*pow(T,2),
                          zp_f-zp_0-zpp_0*T,
                          zpp_f-zpp_0);

      double alpz = (1.0/pow(T,5))*T_matrix_1.dot(delta_pvaz);
      double betz =  (1.0/pow(T,5))*T_matrix_2.dot(delta_pvaz);
      double gamz =  (1.0/pow(T,5))*T_matrix_3.dot(delta_pvaz);

      double z_out = alpz*pow(t,5)/120+betz*pow(t,4)/24+gamz*pow(t,3)/6+zpp_0*pow(t,2)/2+zp_0*t+z_0;
      double zp_out = alpz*pow(t,4)/24+betz*pow(t,3)/6+gamz*pow(t,2)/2+zpp_0*t/2+zp_0;
      double zpp_out = alpz*pow(t,3)/6+betz*pow(t,2)/2+gamz*t+zpp_0;

      tf::Vector3 goal_position_in_world = tf::Vector3(x_out, y_out, z_out);
      tf::Vector3 twist_goal_in_world = tf::Vector3(xp_out, yp_out, zp_out);
      tf::Vector3 accel_goal_in_world = tf::Vector3(xpp_out, ypp_out, zpp_out);

      ar_land::PosVelAcc posVelAcc_in_world;

      tf::vector3TFToMsg(goal_position_in_world,posVelAcc_in_world.position);
      tf::vector3TFToMsg(twist_goal_in_world,posVelAcc_in_world.twist);
      tf::vector3TFToMsg(accel_goal_in_world,posVelAcc_in_world.acc);

      goal_posVelAcc_pub.publish(posVelAcc_in_world); // needed for flat_controller_node

      // for debugging purposes: sendTransform from world to set trajectory point
      // -----------------------------------------------------------------------------
      tf::StampedTransform traj_debug;
      traj_debug.setIdentity();
      traj_debug.setOrigin(goal_position_in_world);
      traj_debug.frame_id_ = world_frame_id;
      traj_debug.child_frame_id_ = goal_frame_id;
      traj_debug.stamp_ = ros::Time::now();

      tf_br.sendTransform(traj_debug);

      // ----------------------------------------------------------------------------


      if(calc_traj_with_real_values) // sets position, velocity and acceleration for new trajectory calculation step with the real measured and observed values
      {
        xp_0 = xp_obs;
        xpp_0 = xpp_obs;
        yp_0 = yp_obs;
        ypp_0 = ypp_obs;
        zp_0 = zp_obs;
        zpp_0 = zpp_obs;

        // actual position of drone

        tf::StampedTransform tf_world_to_drone;
        try{
          tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
        }
        catch(tf::TransformException &ex)
        {
          ROS_INFO("No Transformation from World to Drone found");
        }

        x_0 = tf_world_to_drone.getOrigin().x();
        y_0 = tf_world_to_drone.getOrigin().y();
        z_0 = tf_world_to_drone.getOrigin().z();
      }
      else // sets position, velocity and acceleration for new trajectory calculation step with the previous calculated values
      {
        x_0 = x_out;
        y_0 = y_out;
        z_0 = z_out;
        xp_0 = xp_out;
        yp_0 = yp_out;
        zp_0 = zp_out;
        xpp_0 = xpp_out;
        ypp_0 = ypp_out;
        zpp_0 = zpp_out;
      }

      if(t == T)
      {
        traj_finished = true;
      }
    } // !traj_finished

    if(traj_finished) // published desired position vel and acc as long as another traj is demanded to let the drone hover (maybe not really neccessary cause once the final data are published once everything is okay
    {                 // could also be placed in the flight state: automatic mode
      tf::Vector3 goal_position_in_world = tf::Vector3(x_f, y_f, z_f);
      tf::Vector3 twist_goal_in_world = tf::Vector3(0, 0, 0);
      tf::Vector3 accel_goal_in_world = tf::Vector3(0, 0, 0);

      ar_land::PosVelAcc posVelAcc_in_world;

      tf::vector3TFToMsg(goal_position_in_world,posVelAcc_in_world.position);
      tf::vector3TFToMsg(twist_goal_in_world,posVelAcc_in_world.twist);
      tf::vector3TFToMsg(accel_goal_in_world,posVelAcc_in_world.acc);

      goal_posVelAcc_pub.publish(posVelAcc_in_world);

    }
  }

  //ROS_INFO("Latenz: %f", ros::Time::now().toSec()-start_zeit);

}

void flat_trajectory_planner_node::getValue(const geometry_msgs::Twist &msg){
  last_thrust = msg.linear.z;
}

void flat_trajectory_planner_node::receiveObserverData(const ar_land::PosVelAcc &msg){
  xp_obs = msg.twist.x;
  yp_obs = msg.twist.y;
  zp_obs = msg.twist.z;

  xpp_obs = msg.acc.x;
  ypp_obs = msg.acc.y;
  zpp_obs = msg.acc.z;

}


bool flat_trajectory_planner_node::goal_change(ar_land::goal_change::Request& req, ar_land::goal_change::Response& res)
{
  ROS_INFO("Goal change requested. %d: ", (int) req.button_code);

  tf::StampedTransform tf_world_to_drone;
  try{
    tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
  }
  catch(tf::TransformException &ex)
  {
    ROS_INFO("No Transformation from World to Drone found");
  }

  x_f = tf_world_to_drone.getOrigin().x();
  y_f = tf_world_to_drone.getOrigin().y();
  z_f = tf_world_to_drone.getOrigin().z();

  switch(req.button_code)
  {
  case 1:
  {
    x_f = x_f-0.1;
  }
    break;
  case 2:
  {
    x_f = x_f+0.1;
  }
    break;
  case 3:
  {
    y_f = y_f-0.1;
  }
    break;
  case 4:
  {
    y_f = y_f+0.1;
  }
    break;
  case 5:
  {
    z_f = z_f-0.1;
  }
    break;
  case 6:
  {
    z_f = z_f+0.1;
  }
    break;
  default:
  {
    ROS_ERROR("Error in Button Code");
  }
    break;
  }

  ROS_INFO("Set new goal to (%0.2f, %0.2f, %0.2f)", x_f, y_f, z_f);
  nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
  nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
  nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
  nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);
  traj_started = false;
  traj_finished = false;
  run_traj = true;
}

void flat_trajectory_planner_node::run(double frequency)
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &flat_trajectory_planner_node::setTrajPoint, this); // start at last goal_position
  ros::spin();
}

void flat_trajectory_planner_node::updateBoardinWorld() {

  tf::StampedTransform world_to_board_tf;

  try{
    tf_lis.lookupTransform(world_frame_id, board_frame_id, ros::Time(0), world_to_board_tf); // tf which comes from the camera
  }
  catch (tf::TransformException &ex) {
      // empty
  }

  if(!world_to_board_tf.child_frame_id_.empty())
  {
    board_position_in_world = world_to_board_tf.getOrigin(); // information about pose here or in controller_node?
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "flat_trajectory_planner_node"); // initializes node named flat_trajectory_planner_node

  ros::NodeHandle n("~");
  flat_trajectory_planner_node node;                // Creates flat_trajectory_planner_node
  double frequency = 30; // TODO frequency okay?
  node.run(frequency);
  return 0;
}
