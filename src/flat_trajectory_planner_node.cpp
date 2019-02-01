#include "ar_land/flat_trajectory_planner_node.hpp"


flat_trajectory_planner_node::flat_trajectory_planner_node()
  : flight_state(Idle),
    board_moving(false)


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
  n.param<std::string>("pose_goal_in_world_topic", pose_goal_in_world_topic, "/ar_land/pose_goal_in_world_topic");
  n.param<std::string>("goal_pos_topic", goal_pos_topic, "/ar_land/goal_pos_topic");
  n.param<std::string>("goal_vel_topic", goal_vel_topic, "/ar_land/goal_vel_topic");
  n.param<std::string>("goal_acc_topic", goal_acc_topic, "/ar_land/goal_acc_topic");
  // Subscribers
  //T_cam_board_sub = nh.subscribe(T_cam_board_topic, 1, &flat_trajectory_planner_node::setGoalinWorld, this); // subscribed zu (1) und führt bei empfangener Nachricht (3) damit aus
  control_out_sub = nh.subscribe("/crazyflie/imu", 1, &flat_trajectory_planner_node::getImuAccelZ, this);
  obs_posVelAcc_sub = nh.subscribe("obs_posVelAcc_topic", 1, &flat_trajectory_planner_node::receiveObserverData, this);
  // Publishers
  goal_posVelAcc_pub = nh.advertise<ar_land::PosVelAcc>(goal_posVelAcc_topic, 1);
  goal_pos_pub = nh.advertise<geometry_msgs::Vector3>(goal_pos_topic, 1);
  goal_vel_pub = nh.advertise<geometry_msgs::Vector3>(goal_vel_topic, 1);
  goal_acc_pub = nh.advertise<geometry_msgs::Vector3>(goal_acc_topic, 1);
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pose_goal_in_world_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_goal_in_world_topic, 1); // states that pose_goal_in_world_pub publishes to topic (1)

  path_pub = nh.advertise<nav_msgs::Path>("/goal_path",1);
  drone_path_pub = nh.advertise<nav_msgs::Path>("/drone_path",1);
  //Services
  flight_state_change_srv = nh.advertiseService("flight_state_change", &flat_trajectory_planner_node::state_change, this);
  goal_change_srv_serv = nh.advertiseService("/ar_land/goal_change", &flat_trajectory_planner_node::goal_change, this);

  goal_position_in_board.setValue(0.3,0.3,0.7);
  frequency = 100;
  run_traj = false;
  replan_traj = false;
  traj_started = false;
  traj_finished = false;
  calc_traj_with_real_values = false;
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
  last_accel_z = 0;
  accel_z = 0;
  x_f_corr = 0;
  y_f_corr = 0;
  z_f_corr = 0;
  land_straight = false;
  latenz = ros::Time::now();


}

bool flat_trajectory_planner_node::state_change(ar_land::flight_state_changeRequest &req,
                                                ar_land::flight_state_changeResponse  &res)
{
  flight_state = State(req.flight_state);
  res.changed = 1;

  switch(flight_state)
  {
  case Idle:
  {
    run_traj = false;
    traj_started = false;
    traj_finished = false;

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
control_out_pub.publish(msg);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", false);
    nh.setParam("/ar_land/pid_controller_node/controller_enabled", false);
    nh.setParam("/ar_land/flat_controller_node/resetPID", true);
control_out_pub.publish(msg);
    ROS_INFO("State change to Idle");
    ROS_INFO("Latenz seit Bump: %f: ", ros::Time::now().toSec()-latenz.toSec());
  }
    break;
  case Automatic:
  {
    ROS_INFO("State change to Automatic");
    run_traj = false;
    traj_started = false;
    traj_finished = false;
    nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
    nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
    nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
    tf::Vector3 goal_position_in_world = tf::Vector3(x_f, y_f, z_f);
    tf::Vector3 twist_goal_in_world = tf::Vector3(0, 0, 0);
    tf::Vector3 accel_goal_in_world = tf::Vector3(0, 0, 0);

    ar_land::PosVelAcc posVelAcc_in_world;

    tf::vector3TFToMsg(goal_position_in_world,posVelAcc_in_world.position);
    tf::vector3TFToMsg(twist_goal_in_world,posVelAcc_in_world.twist);
    tf::vector3TFToMsg(accel_goal_in_world,posVelAcc_in_world.acc);

    goal_posVelAcc_pub.publish(posVelAcc_in_world);


    if(replan_traj)
    {
      ROS_INFO("Will replan trajectory");
      traj_started = false;
      traj_finished = false;
      run_traj = true;
    }

  goal_path = nav_msgs::Path();
  goal_path.header.stamp = ros::Time::now();
  goal_path.header.frame_id = world_frame_id;
  path_pub.publish(goal_path);

  drone_path = nav_msgs::Path();
  drone_path.header.stamp = ros::Time::now();
  drone_path.header.frame_id = world_frame_id;
  drone_path_pub.publish(drone_path);
    // test for using PID Controllers
    /*
nh.setParam("/ar_land/pid_controller_node/controller_enabled", true);
    geometry_msgs::TransformStamped T_world_goal_msg;
    geometry_msgs::PoseStamped pose_goal_in_world_msg;
    tf::StampedTransform world_to_goal_tf;
    world_to_goal_tf.setOrigin(goal_position_in_world);
    world_to_goal_tf.setRotation(tf::Quaternion::getIdentity());
    world_to_goal_tf.frame_id_ = world_frame_id;
    world_to_goal_tf.child_frame_id_ = goal_frame_id;    
    world_to_goal_tf.stamp_ = ros::Time::now();
    tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);

    tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);

    pose_goal_in_world_pub.publish(pose_goal_in_world_msg); // neccessary for pid_controller_node
*/

    // -----------------------------------------------

  }
    break;
  case TakingOff:
  {
    ROS_INFO("State change to TakeOff");

    // test for using PID Controllers
    /*
    tf::StampedTransform tf_world_to_drone;
    try{
      tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
    }
    catch(tf::TransformException &ex)
    {
      ROS_INFO("No Transformation from World to Drone found");
    }

    geometry_msgs::TransformStamped T_world_goal_msg;
    geometry_msgs::PoseStamped pose_goal_in_world_msg;
    tf::StampedTransform world_to_goal_tf;
    world_to_goal_tf.setOrigin(tf_world_to_drone.getOrigin());
    world_to_goal_tf.setRotation(tf::Quaternion::getIdentity());
    world_to_goal_tf.frame_id_ = world_frame_id;
    world_to_goal_tf.child_frame_id_ = goal_frame_id;
    world_to_goal_tf.stamp_ = ros::Time::now();
    tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);

    tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);

    pose_goal_in_world_pub.publish(pose_goal_in_world_msg); // neccessary for pid_controller_node
    */

    // -----------------------------------------------

    // set 0.5m above world frame as takeoff goal
    x_f = 0.5;
    y_f = -0.5;
    z_f = 2.0;

    nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
    nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
    nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);
    nh.setParam("/ar_land/pid_controller_node/controller_enabled", true);
    traj_started = false;
    traj_finished = false;
    run_traj = true;
  }
    break;
  case Landing:
  {
    ROS_INFO("State change to Landing");
    x_f = board_position_in_world.x();
    y_f = board_position_in_world.y();
    z_f = board_position_in_world.z();
    nh.setParam("/ar_land/flat_controller_node/x_final_in_world", x_f);
    nh.setParam("/ar_land/flat_controller_node/y_final_in_world", y_f);
    nh.setParam("/ar_land/flat_controller_node/z_final_in_world", z_f);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", true);
    traj_started = false;
    traj_finished = false;
    run_traj = true;

  }
    break;
  case Emergency:
  {
    nh.setParam("/ar_land/flat_controller_node/resetPID", true);
    nh.setParam("/ar_land/flat_controller_node/controller_enabled", false);
    nh.setParam("/ar_land/pid_controller_node/controller_enabled", false);
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
  if(flight_state == Landing)
  {

    updateGoalPos(); // sets the current board position as goal position

    if(land_straight)
    {
    z_f = z_f + z_f_corr;
    x_f = x_f + x_f_corr;
    y_f = y_f + y_f_corr;
    zp_f = 0;
    }
    else
    {
      zp_f = - 0.6;
      z_f = z_f - 0.04;
    }
    //ROS_INFO("Set new goal to (%0.2f, %0.2f, %0.2f)", x_f, y_f, z_f);
  }  
  else
  {
    zp_f = 0;
  }

  if(run_traj)
  {
    tf::StampedTransform tf_world_to_drone;
    try{
      tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
    }
    catch(tf::TransformException &ex)
    {
      ROS_INFO("No Transformation from World to Drone found");
    }
    float vel = 0.5;//325; // [m/s]  travel velocity
    if(!traj_started)
    {          
      //t_prev = 0;
      //rost_prev = ros::Time::now();
      // initializes start of completely new commanded trajectory with zero velocities and accelerations and actual position of drone
      start_time = ros::Time::now();
      traj_started = true;
      xp_0 = 0.0;
      xpp_0 = 0.0;
      yp_0 = 0.0;
      ypp_0 = 0.0;
      zp_0 = 0.0;
      zpp_0 = 0.0;
      x_f_prev = x_f;
      y_f_prev = y_f;


      // actual position of drone as start point for trajectory when landing
      if(flight_state == TakingOff)
      {

      x_0 = tf_world_to_drone.getOrigin().x();
      y_0 = tf_world_to_drone.getOrigin().y();
      z_0 = tf_world_to_drone.getOrigin().z();
}
      else // if not TakingOff
      {
      x_0 = x_f_old;
      y_0 = y_f_old;
      z_0 = z_f_old;
      // ----------------------------------------------------------------
      // try to assign a end position that is not on the ground but beneath

      double delta_x = -tf_world_to_drone.getOrigin().x() + x_f;
      double delta_y = -tf_world_to_drone.getOrigin().y() + y_f;
      double delta_z = -tf_world_to_drone.getOrigin().z() + z_f;

      tf::Vector3 vector = tf::Vector3(delta_x, delta_y, 0).normalized()*0.1;

      x_f_corr = vector.x();
      y_f_corr = vector.y();
      z_f_corr = vector.z();
      // ----------------------------------------------------------------
      }
      float distance =  tf::Vector3(x_0-x_f, y_0-y_f, z_0-z_f).length();

      if(distance > 2.5)
        vel = vel*2;
      else if (distance > 1.5)
        vel = vel*1.5;

      T = distance/vel;


      T = 0.5*2*vel*distance + 1.25; // new approachh


      ROS_INFO("Start Traj: \t %f, %f, %f", x_0, y_0, z_0);
      ROS_INFO("End Traj: \t %f, %f, %f", x_f, y_f, z_f);
    }

    if(!traj_finished)
    {
      //double t = 1.0/frequency + t_prev;
double t = 1.0/frequency;


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
      double xp_out =  alpx*pow(t,4)/24+betx*pow(t,3)/6+gamx*pow(t,2)/2+xpp_0*t+xp_0;
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
      double yp_out = alpy*pow(t,4)/24+bety*pow(t,3)/6+gamy*pow(t,2)/2+ypp_0*t+yp_0;
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
      double zp_out = alpz*pow(t,4)/24+betz*pow(t,3)/6+gamz*pow(t,2)/2+zpp_0*t+zp_0;
      double zpp_out = alpz*pow(t,3)/6+betz*pow(t,2)/2+gamz*t+zpp_0;

      tf::Vector3 goal_position_in_world = tf::Vector3(x_out, y_out, z_out);
      tf::Vector3 twist_goal_in_world = tf::Vector3(xp_out, yp_out, zp_out);
      tf::Vector3 accel_goal_in_world = tf::Vector3(xpp_out, ypp_out, zpp_out);

      ar_land::PosVelAcc posVelAcc_in_world;

      tf::vector3TFToMsg(goal_position_in_world,posVelAcc_in_world.position);
      tf::vector3TFToMsg(twist_goal_in_world,posVelAcc_in_world.twist);
      tf::vector3TFToMsg(accel_goal_in_world,posVelAcc_in_world.acc);

      goal_posVelAcc_pub.publish(posVelAcc_in_world); // needed for flat_controller_node
      goal_pos_pub.publish(posVelAcc_in_world.position); // only neccessary for matlab plots
      goal_vel_pub.publish(posVelAcc_in_world.twist); // only neccessary for matlab plots
      goal_acc_pub.publish(posVelAcc_in_world.acc); // only neccessary for matlab plots

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

      // test for using PID Controllers

      geometry_msgs::TransformStamped T_world_goal_msg;
      geometry_msgs::PoseStamped pose_goal_in_world_msg;
      tf::StampedTransform world_to_goal_tf;
      world_to_goal_tf.setOrigin(goal_position_in_world);
      world_to_goal_tf.setRotation(tf::Quaternion::getIdentity());
      world_to_goal_tf.child_frame_id_ = goal_frame_id;
      world_to_goal_tf.frame_id_ = world_frame_id;
      world_to_goal_tf.stamp_ = ros::Time::now();
      tf::transformStampedTFToMsg(world_to_goal_tf, T_world_goal_msg);
      tools_func::convert(T_world_goal_msg, pose_goal_in_world_msg);

      pose_goal_in_world_pub.publish(pose_goal_in_world_msg); // neccessary for pid_controller_node


      if(flight_state == Landing){
      goal_path.poses.push_back(pose_goal_in_world_msg);
      goal_path.header.stamp = ros::Time::now();
      goal_path.header.frame_id = world_frame_id;

      //-----------------------------------------

      tf::StampedTransform tf_world_to_drone;
      try{
        tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
      }
      catch(tf::TransformException &ex)
      {
        ROS_INFO("No Transformation from World to Drone found");
      }
      geometry_msgs::PoseStamped pose_drone_in_world_msg;
      geometry_msgs::TransformStamped T_world_drone_msg;
      //T_world_drone_msg.header.frame_id = world_frame_id;
      //T_world_drone_msg.child_frame_id = drone_frame_id;
      //T_world_drone_msg.transform = tf_world_to_drone;
      tf::transformStampedTFToMsg(tf_world_to_drone, T_world_drone_msg);

      tools_func::convert(T_world_drone_msg, pose_drone_in_world_msg);
      drone_path.poses.push_back(pose_drone_in_world_msg);
      drone_path.header.stamp = ros::Time::now();
      drone_path.header.frame_id = world_frame_id;
      //----------------------------------------


      path_pub.publish(goal_path);
      drone_path_pub.publish(drone_path);
      }

      // -----------------------------------------------


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
       // ROS_INFO("Traj point: %0.2f, %0.2f, %0.2f", z_out, zp_out, zpp_out);

        //t_prev = t;
      }
if(flight_state == Landing)
  ROS_INFO("diff_z: %f:   accel_z: %f:", accel_z-last_accel_z, accel_z);

      if(flight_state == Landing && (tf_world_to_drone.getOrigin().z()-board_position_in_world.getZ()) < 0.1) // drone is near (less than 10cm) the marker while landing
      {
        //ROS_INFO("%f",std::abs(last_accel_z-accel_z));
        // "hear" for the bump

        if(accel_z-last_accel_z>3&&accel_z>0.2)
        {
          ROS_INFO("Bump detected");

          traj_finished = true;
        }
      }

      if(t == T)
      {
        traj_finished = true;
      }
      if(!board_moving){
       T = T - t;
      }else{
        float alpha = -0.9;
        if(pow(x_out-x_f,2)+pow(y_out-y_f,2)>pow(x_out-x_f_prev,2) + pow(y_out-y_f_prev,2)){
          alpha = 1.0;
        }
        T = T- t + alpha*10*(pow(x_f-x_f_prev,2) +  pow(y_f-y_f_prev,2));
      }
      x_f_prev = x_f;
      y_f_prev = y_f;
    } // !traj_finished

    if(traj_finished) // published desired position vel and acc as long as another traj is demanded to let the drone hover (maybe not really neccessary cause once the final data are published once everything is okay
    {
      x_f_old = x_f;
      y_f_old = y_f;
      z_f_old = z_f;
      ar_land::flight_state_changeRequest req;
      ar_land::flight_state_changeResponse res;
      if(flight_state == Landing)
      {
        ROS_INFO("Landing done. --> State change to Idle");
latenz = ros::Time::now();
        req.flight_state = 0; // Idle
      }
      else
      {
        req.flight_state = 1; // Automatic
      }
    state_change(req, res);
    }
  }

  //ROS_INFO("Latenz: %f", ros::Time::now().toSec()-start_zeit);

}

void flat_trajectory_planner_node::getImuAccelZ(const sensor_msgs::Imu::ConstPtr& msg){
  last_accel_z = accel_z;
  sensor_msgs::Imu imu_msg;
  imu_msg = (*msg);
  accel_z = imu_msg.linear_acceleration.z;
  //double test = std::abs(last_accel_z-accel_z);
  //ROS_INFO("delta_accel_z: %f", test);
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

  //x_f = tf_world_to_drone.getOrigin().x();
  //y_f = tf_world_to_drone.getOrigin().y();
  //z_f = tf_world_to_drone.getOrigin().z();

  switch(req.button_code)
  {
  case 1:
  {
    x_f = x_f-0.4;
  }
    break;
  case 2:
  {
    x_f = x_f+0.4;
  }
    break;
  case 3:
  {
    y_f = y_f-0.4;
  }
    break;
  case 4:
  {
    y_f = y_f+0.4;
  }
    break;
  case 5:
  {
    z_f = z_f-0.4;
  }
    break;
  case 6:
  {
    z_f = z_f+0.4;
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
  double frequency = 100; // TODO frequency okay?
  node.run(frequency);
  return 0;
}
