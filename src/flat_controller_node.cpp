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
                                            const std::string& imu_frame_id,
                                            const std::string& goal_frame_id,
                                          const ros::NodeHandle& n)
  : world_frame_id(world_frame_id)
  , drone_frame_id(drone_frame_id)
  , imu_frame_id(imu_frame_id)
  , goal_frame_id(goal_frame_id)
  , pid_yaw(
      func::get(n, "PIDs/Yaw/kp"),
      func::get(n, "PIDs/Yaw/kd"),
      func::get(n, "PIDs/Yaw/ki"),
      func::get(n, "PIDs/Yaw/minOutput"),
      func::get(n, "PIDs/Yaw/maxOutput"),
      func::get(n, "PIDs/Yaw/integratorMin"),
      func::get(n, "PIDs/Yaw/integratorMax"),
      "yaw")
  , controller_started(false)
  , resetPID(false)
  , observer_init(false)
  , recievedImuRot(false)
  , integral_part_z(0)
  , wind_up(0)

  // Body of class definition
{

  ros::NodeHandle nh;

  //initialization
  //K_x.setValue(18.5,0,0,0,-19.5,0,0,0,6000);

  //K_x.setValue(18.5,0,0,0,-19.5,0,0,0,6000);
  K_x.setValue(3,0,0,0,3,0,0,0,20);
  //K_v.setValue(9.5,0,0,0,-9.5,0,0,0,7000);
  K_v.setValue(5,0,0,0,5,0,0,0,5);


  // Publishers
  control_out_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  control_error_pub = nh.advertise<geometry_msgs::Vector3>("control_error_topic", 1);
  obs_posVelAcc_pub = nh.advertise<ar_land::PosVelAcc>("obs_posVelAcc_topic", 1);
  obs_vel_pub = nh.advertise<geometry_msgs::Vector3>("obs_vel_topic",1);
  controller_debug_pub = nh.advertise<ar_land::controller_debug>("z_controller_debug_topic",1);

  // Subscribers
  //pose_goal_in_world_sub = nh.subscribe("/ar_land/pose_goal_in_world_topic", 1, &flat_controller_node::goalChanged, this);
  goal_posVelAcc_sub = nh.subscribe("/ar_land/goal_posVelAcc_topic", 1, &flat_controller_node::receiveTrajectory, this);
  imuData_sub = nh.subscribe("/crazyflie/imu", 1, &flat_controller_node::receiveIMUdata, this);
  imuRotation_quat_sub = nh.subscribe("/crazyflie/log_state_estimate_quat",1,&flat_controller_node::receiveIMURot_Quat, this); // alternative using the Quaternions directly from drone
  //imuRotation_rpy_sub = nh.subscribe("/crazyflie/log_state_estimate_rpy",1,&flat_controller_node::receiveIMURot_rpy, this);


  //dynamic reconfigure
  //dynamic reconfigure overwrites PID parameters immediately from cfg file
  dynamic_reconfigure::Server<ar_land::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&flat_controller_node::dynamic_reconfigure_callback, this, _1, _2);
  m_server.setCallback(f);
  initializeRotation(); // nochmal überlegen wo und wann und wie das passieren muss...
}

void flat_controller_node::run(double frequency)
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &flat_controller_node::iteration, this);
  ros::Timer timer_obs = node.createTimer(ros::Duration(1.0/100), &flat_controller_node::getActualPosVel, this);
  ros::spin();

}

void flat_controller_node::receiveTrajectory(const ar_land::PosVelAcc::ConstPtr& msg)
{
  posVelAcc_goal_in_world_msg = *msg;

}

void flat_controller_node::receiveIMUdata(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuData_msg = *msg;
}

void flat_controller_node::receiveIMURot_Quat(const crazyflie_driver::GenericLogDataConstPtr& msg){
  if(msg->values.size() < 4){
    ROS_ERROR("recieved Message to small for Quaternion");
    return;
  }

  imuRotation = tf::Quaternion(msg->values[0],msg->values[1],msg->values[2],msg->values[3]);
  recievedImuRot =true;
}


// not working yet
/*
void flat_controller_node::receiveIMURot_rpy(const crazyflie_driver::GenericLogDataConstPtr& msg){
  if(msg->values.size() < 3){
    ROS_ERROR("recieved Message to small for rpy");
    return;
  }

  imuRotation = tf::Quaternion(msg->values[2],msg->values[1],msg->values[0]);
  tf::Matrix3x3 test ;
  test.setEulerZYX(msg->values[2],msg->values[1],msg->values[0]);
  tf::Quaternion test2;
  test.getRotation(test2);
  ROS_INFO("initial quaternion = [ %f, %f, %f, %f",imuRotation.getW(),imuRotation.getX(),imuRotation.getY(),imuRotation.getZ());
  ROS_INFO("tset quaternion = [ %f, %f, %f, %f",test2.getW(),test2.getX(),test2.getY(),test2.getZ());
  recievedImuRot =true;
}*/

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
    //nh.getParam("/ar_land/flat_controller_node/z_integral", z_integral);
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
      prev_time_ctrl = ros::Time::now();
    integral_part_z = 0.0; // because for every point on a trajectory it should be renewed
    }

    float dt = ros::Time::now().toSec() - prev_time_ctrl.toSec();
    prev_time_ctrl = ros::Time::now();

    tf::Vector3 g_vec;
    g_vec.setValue(0,0,9.81);
    tf::Vector3 x_setpoint  = tools_func::convertToTFVector3(posVelAcc_goal_in_world_msg.position);
    tf::Vector3 e_x = prefilter* x_setpoint - x_actual;  // Vorfilter macht funktioniert nicht mit I Anteil, dieser müsste parallel geschalten sein
    tf::Vector3 e_v = tools_func::convertToTFVector3(posVelAcc_goal_in_world_msg.twist) - v_obs;
    //ROS_INFO("e_x: %f, %f, %f \t e_v: %f, %f, %f", e_x.x(),e_x.y(),e_x.z(), e_v.x(), e_v.y(), e_v.z());
    a_ref = tools_func::convertToTFVector3(posVelAcc_goal_in_world_msg.acc)+ K_x*(e_x) + K_v*(e_v) + g_vec;

    float anti_wind = 1;
    integral_part_z += ((x_setpoint - x_actual).getZ() - wind_up*anti_wind/(thrust_fact*0.043))*dt;

    tf::StampedTransform tf_world_to_drone;
    try{
      tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
    }
    catch(tf::TransformException &ex)
    {
      ROS_INFO("No Transformation from World to Drone found");
    }

    //tf::Matrix3x3 R = tf::Matrix3x3(tf_world_to_drone.getRotation());
    tfScalar roll, pitch, yaw;
    /*
    tf::Matrix3x3 R = tf::Matrix3x3(
          tf::Quaternion(
            tf_world_to_drone.getRotation().x(),
            tf_world_to_drone.getRotation().y(),
            tf_world_to_drone.getRotation().z(),
            tf_world_to_drone.getRotation().w()
            )); */
    tf::Matrix3x3 R(tf_world_to_drone.getRotation()); //replace with

    R.getRPY(roll, pitch, yaw);
    //ROS_INFO("RPY: %f   %f   %f", roll, pitch, yaw);

    //ROS_INFO("quat: %f   %f   %f   %f", tf_world_to_drone.getRotation().x(), tf_world_to_drone.getRotation().y(), tf_world_to_drone.getRotation().z(), tf_world_to_drone.getRotation().w());

    double thrust = (a_ref.dot(R.getColumn(2))  + Ki_z * integral_part_z)*thrust_fact*0.043; // Factor (113000, 134000) in order to achieve a thrust of 44500 when drone should hover
                                                           // getColumn(2) returns third column

    // control limit for thrust
    if(thrust>60000.0){
      wind_up = thrust-60000.0;
    } else if(thrust<27000.0){
      wind_up = thrust-27000.0;
    } else{
      wind_up = 0.0;
    }
    thrust = std::max(27000.0, std::min(60000.0, thrust));



    tf::Vector3 R_ref_col_3 = a_ref.normalized();







    tf::Vector3  goal_final_in_world;
    float x, y, z;

    nh.getParam("/ar_land/flat_controller_node/x_final_in_world", x);
    nh.getParam("/ar_land/flat_controller_node/y_final_in_world", y);
    nh.getParam("/ar_land/flat_controller_node/z_final_in_world", z);

    goal_final_in_world.setValue(x,y,z);

    tf::Vector3 tilt_vector = (a_ref.cross(goal_final_in_world-x_actual)).normalized();

    tilt_vector = R.inverse()*tilt_vector;

    //ROS_INFO("tilt_vector: %f, %f, %f",tilt_vector.getX(),tilt_vector.getY,tilt_vector.getZ());

    tf::Vector3 R_ref_col_1 = tilt_vector.cross(R_ref_col_3);


   // tf::Matrix3x3 R_ref =  tf::Matrix3x3(R_ref_col_1.x(), R_ref_col_2.x(), R_ref_col_3.x(),
    //                                     R_ref_col_1.y(), R_ref_col_2.y(), R_ref_col_3.y(),
     //                                    R_ref_col_1.z(), R_ref_col_2.z(), R_ref_col_3.z());


    tf::Vector3 z_axis = tf::Vector3(0,0,1);
    tfScalar tilt_angle = z_axis.angle(a_ref);
    tfScalar tilt_angle_neg = a_ref.angle(z_axis);

    // now tilt_angle is always positive thus we need to change the sign depending on the quadrant

    if(tools_func::sign(x_actual.x()) == tools_func::sign(a_ref.x()) && tools_func::sign(x_actual.y()) == tools_func::sign(a_ref.y()) )
    {
      tilt_angle = -tilt_angle;
}


    //ROS_INFO("tilt_angles: %f, %f", tilt_angle, tilt_angle_neg);

tf::Quaternion q = tf::Quaternion(tilt_vector, tilt_angle);
//q = tf_world_to_drone.getRotation()*q;

    tf::Matrix3x3 R_ref = tf::Matrix3x3(q);
    //R_ref = R*R_ref;






//ROS_INFO("Quat: %f %f %f %f", q.getX(), q.getY(), q.getZ(), q.getW());


    //ROS_INFO("R_ref: %f   %f   %f", R_ref.getRow(0).x(), R_ref.getRow(0).y(), R_ref.getRow(0).z());
    //ROS_INFO("     : %f   %f   %f", R_ref.getRow(1).x(), R_ref.getRow(1).y(), R_ref.getRow(1).z());
    //ROS_INFO("     : %f   %f   %f", R_ref.getRow(2).x(), R_ref.getRow(2).y(), R_ref.getRow(2).z());


    tfScalar roll_ref, pitch_ref, yaw_ref;
/*
    R.inverse().getEulerYPR(yaw_test,pitch_test,roll_test);
    tf::Matrix3x3 R_transform;
    R_transform.setEulerYPR(yaw_test,0,0);
    R_ref = R_transform*R_ref;
    tools_func::printRotation(R_transform);
*/
    R_ref.getEulerYPR(yaw_ref, pitch_ref, roll_ref);

    //------------------------------------------------------------------------

      tf::Matrix3x3 v_matrix;
      v_matrix.setEulerYPR(yaw,0,0);
      tf::Vector3 b = v_matrix.inverse()*R_ref_col_3;
      pitch_ref = std::atan(b.getX()/b.getZ());
      roll_ref = std::atan2(-b.getY(),std::sqrt(pow(b.getX(),2) + pow(b.getZ(),2)));

      q.setEuler(yaw,pitch_ref,roll_ref);

    //-------------------------------------------------------------------------

      //tools_func::printRotation(v_matrix);
      //ROS_INFO("pitch_ref: %f     roll_ref: %f", pitch_ref,roll_ref);

      tf::StampedTransform tf_world_to_goal_pose;
          try{
            tf_lis.lookupTransform(world_frame_id, goal_frame_id, ros::Time(0), tf_world_to_goal_pose);
          }
          catch(tf::TransformException &ex)
      {}
      tf_world_to_goal_pose.setRotation(q);
      tf_world_to_goal_pose.frame_id_ = world_frame_id;
      tf_world_to_goal_pose.child_frame_id_ = "/crazyflie/goal_pose";
      tf_world_to_goal_pose.stamp_ = ros::Time::now();

      tf_broad.sendTransform(tf_world_to_goal_pose);

      tf::StampedTransform tf_world_to_drone_pose;
          try{
            tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone_pose);
          }
          catch(tf::TransformException &ex)
      {}
      tf_world_to_drone_pose.setRotation(q);
      tf_world_to_drone_pose.frame_id_ = world_frame_id;
      tf_world_to_drone_pose.child_frame_id_ = "/crazyflie/control_pose";
      tf_world_to_drone_pose.stamp_ = ros::Time::now();

      tf_broad.sendTransform(tf_world_to_drone_pose);

    ar_land::controller_debug ctrl_msg;
    ctrl_msg.I_part = Ki_z * integral_part_z*thrust_fact*0.043;
    ctrl_msg.PD_part = a_ref.dot(R.getColumn(2))*thrust_fact*0.043;
    ctrl_msg.controller_out = thrust;
    controller_debug_pub.publish(ctrl_msg);

    geometry_msgs::Twist control_out;
    control_out.linear.x = std::max(-10.0, std::min(10.0, pitch_ref/M_PI*180.0));  // maybe switch roll_ref and pitch_ref
    control_out.linear.y = std::max(-10.0, std::min(10.0, roll_ref/M_PI*180.0));
    control_out.linear.z = thrust;
    control_out.angular.z =  0;//pid_yaw.update(yaw, yaw_ref);

    control_out_pub.publish(control_out);
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
  else // if controller_enabled is false
  {
    geometry_msgs::Twist control_out;
    control_out.linear.x = 0;
    control_out.linear.y = 0;
    control_out.linear.z = 0;
    control_out.angular.z =  0;

    control_out_pub.publish(control_out);
  }

  nh.getParam("/ar_land/flat_controller_node/resetPID", resetPID);
  if(resetPID)
  {
    flat_controller_node::pidReset();
    resetPID = false;
  }



}

/*
  Implementation of Observer for simple integration model of velocity. Poles in the s-space are at s = 10 +- 5j
*/

void flat_controller_node::getActualPosVel(const ros::TimerEvent& e){



  tf::StampedTransform tf_world_to_drone;
  tf::StampedTransform tf_drone_to_imu;
  try{
    tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
    tf_lis.lookupTransform(drone_frame_id, imu_frame_id, ros::Time(0), tf_drone_to_imu);
  }
  catch(tf::TransformException &ex)
  {
    ROS_INFO("No Transformation from World to Drone found");

  }


  x_actual = tf_world_to_drone.getOrigin();
  if(!observer_init){                           // bessere Initialisierung überlegen. Also sufficient if one lands several times ? maybe needs to be reset better
    x_obs_prev = x_actual;
    x_actual_prev = x_actual;
    v_obs_prev = tf::Vector3(0.0,0.0,0.0);
    prev_time = ros::Time::now();
    observer_init = true;
  }
  float dt = ros::Time::now().toSec() - prev_time.toSec();

  float gravity = 9.81;
  tf::Vector3 imuData;
  tf::Transform rot_world_to_drone_imu(imuRotation);// // quaternion of sensor frame relative to auxiliary frame %% copied from firmware
  rot_world_to_drone_imu = rot_world_to_ImuInitial*rot_world_to_drone_imu;
  tf::Transform rot_world_to_drone(tf_world_to_drone.getRotation());
  tf::Matrix3x3 fusedRotation = fuseRotation(rot_world_to_drone_imu,tf_world_to_drone);
  imuData = tools_func::convertToTFVector3(imuData_msg.linear_acceleration); // transform in world-coordinates and subtract local gravity -> z-value in in ground position calib am Anfgang...
 // imuData = tf_world_to_drone*tf_drone_to_imu*imuData-tf_world_to_drone.getOrigin();  // aufpassen, die Transformation von der Kamera passt in der Regel nicht, da zeitlich zu verschieden, deshalb wahrscheinlich besser alles im Drone frame zu berechnen, da position immer mit pose upgedatet wird und eh übereinstimmt
  imuData = rot_world_to_drone*imuData;
  //imuData = fusedRotation*imuData;
  imuData.setZ(imuData.getZ()-gravity);

  // observer for velocities
  //float l1 = 1.4;
  //float l2 = 14.7;

  float l1 = 0.588;
  float l2 = 3.03;
  x_obs = (1-l1)*x_obs_prev + dt*v_obs_prev + dt*dt*0.5*0.0*imuData + l1*x_actual_prev;  // eventuell modifizierten Beobachter implementieren
  //v_obs = v_obs_prev + dt*0.0*imuData+l2*x_actual-l2*x_obs_prev;  // Außerdem stimmen Koordinatensysteme der einzelnen komponenten gar nicht überein, oben geändert


  // alternative as simple differentiation with low pass filter

  tf::Vector3 v_obs_unfiltered;
  float alpha = 0.3;
if(dt>0)
  {
    v_obs_unfiltered = (x_actual-x_actual_prev)/dt;
    v_obs = alpha*v_obs_unfiltered + (1-alpha)*v_obs_prev;  // low pass filter 1. order
  }
  else
  {
    v_obs = tf::Vector3(0.0,0.0,0.0);
}
  x_actual_prev = x_actual;
  v_obs_prev = v_obs;
  x_obs_prev = x_obs;
  prev_time = ros::Time::now();

  //ROS_INFO("v_obs = %f, x_obs = %f, x_actual = %f", (float)v_obs.x(), (float)x_obs.x(), (float)x_actual.x());
  ar_land::PosVelAcc posVelAcc_in_world;

  tf::vector3TFToMsg(x_obs,posVelAcc_in_world.position);
  tf::vector3TFToMsg(v_obs,posVelAcc_in_world.twist);
  tf::vector3TFToMsg(imuData,posVelAcc_in_world.acc);


  obs_posVelAcc_pub.publish(posVelAcc_in_world);
  obs_vel_pub.publish(posVelAcc_in_world.twist);

}

void flat_controller_node::dynamic_reconfigure_callback(
      ar_land::dynamic_param_configConfig& config, uint32_t level) {

  ROS_INFO("Reconfigure Request: %f %f %f,%f %f %f", config.Kp_x, config.Kd_x, config.Kp_y, config.Kd_y, config.Kp_z, config.Kd_z);


  K_x.setValue(config.Kp_x,0,0,0,config.Kp_y,0,0,0,config.Kp_z);
  K_v.setValue(config.Kd_x,0,0,0,config.Kd_y,0,0,0,config.Kd_z);

  pid_yaw.setKP(config.Kp_yaw);
  pid_yaw.setKI(config.Ki_yaw);
  pid_yaw.setKD(config.Kd_yaw);
  thrust_fact = config.thrust_fact;
  Ki_z = config.Ki_z;

  prefilter.setX(config.Prefilter_x);
  prefilter.setY(config.Prefilter_y);
  prefilter.setZ(config.Prefilter_z);

}

void flat_controller_node::initializeRotation(){
  // wait for trafos from Imu to marker coordinate frame, marker to tracking room frame, Imu to imu_inertial frame rotation
  // q_start = q_ots*q_m*inv(q)
  while(!recievedImuRot){
    ros::spinOnce();
  }
  tf::StampedTransform tf_world_to_drone;
  tf::StampedTransform tf_drone_to_imu;
  try{
    tf_lis.lookupTransform(world_frame_id, drone_frame_id, ros::Time(0), tf_world_to_drone);
    tf_lis.lookupTransform(drone_frame_id, imu_frame_id, ros::Time(0), tf_drone_to_imu);
  }
  catch(tf::TransformException &ex)
  {
    ROS_INFO("No Transformation from World to Drone found");

  }


  tf::Transform rot_world_to_drone(tf_world_to_drone.getRotation());
  tf::Transform rot_drone_to_imu(tf_drone_to_imu);
  ROS_INFO("initial quaternion = [ %f, %f, %f, %f",imuRotation.getW(),imuRotation.getX(),imuRotation.getY(),imuRotation.getZ());
  tf::Transform rot_imu_to_ImuInitial(imuRotation.inverse());

  //rot_ImuInitial_to_world = rot_ImutInitial_to_Imu*rot_Imu_to_marker*rot_marker_to_world;

  rot_world_to_ImuInitial = rot_world_to_drone*rot_drone_to_imu*rot_imu_to_ImuInitial;



}

tf::Matrix3x3 fuseRotation(tf::Transform tf_by_imu, tf::Transform tf_by_tracking_room){
  //assume down-direction of IMU estimation to be correct
  tf::Matrix3x3 imuRot(tf_by_imu.getRotation());
  tf::Matrix3x3 trackRot(tf_by_tracking_room.getRotation());

  //ROS_INFO("imu :[ %f , %f , %f ; %f , %f , %f ; %f , %f , %f ]",imuRot[0].getX(),imuRot[0].getY(),imuRot[0].getZ(),imuRot[1].getX(),imuRot[1].getY(),imuRot[1].getZ(),imuRot[2].getX(),imuRot[2].getY(),imuRot[2].getZ());
  //ROS_INFO("track :[ %f , %f , %f ; %f , %f , %f ; %f , %f , %f ]",trackRot[0].getX(),trackRot[0].getY(),trackRot[0].getZ(),trackRot[1].getX(),trackRot[1].getY(),trackRot[1].getZ(),trackRot[2].getX(),trackRot[2].getY(),trackRot[2].getZ());

  tf::Vector3 bz_star = imuRot.getColumn(2);
  tf::Vector3 bx = trackRot.getColumn(0);
  tf::Vector3 by_star = bz_star.cross(bx);
  by_star.normalize();
  tf::Vector3 bx_star = by_star.cross(bz_star);
  bx_star.normalize();

  tf::Matrix3x3 fusedRotation = tf::Matrix3x3(bx_star.getX(),by_star.getX(),bz_star.getX(),bx_star.getY(),by_star.getY(),bz_star.getY(),bx_star.getZ(),by_star.getZ(),bz_star.getZ());
  //ROS_INFO("fused :[ %f , %f , %f ; %f , %f , %f ; %f , %f , %f ]",fusedRotation[0].getX(),fusedRotation[0].getY(),fusedRotation[0].getZ(),fusedRotation[1].getX(),fusedRotation[1].getY(),fusedRotation[1].getZ(),fusedRotation[2].getX(),fusedRotation[2].getY(),fusedRotation[2].getZ());
  return fusedRotation;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "flat_controller_node");

  // Read parameters
  ros::NodeHandle n("~");
  std::string world_frame_id;
  std::string drone_frame_id;
  std::string imu_frame_id;
  std::string goal_frame_id;
  double frequency;


  n.param<std::string>("world_frame_id", world_frame_id, "/world"); // liest Parameter (1) vom Parameter Server aus und speichert in (2)

  n.param<std::string>("drone_frame_id", drone_frame_id, "/crazyflie/base_link");

  n.param<std::string>("imu_frame_id", imu_frame_id, "/imu");

  n.param<std::string>("goal_frame_id", goal_frame_id, "/crazyflie/goal");

  n.param<double>("frequency", frequency, 100.0);

  flat_controller_node controller(world_frame_id, drone_frame_id, imu_frame_id, goal_frame_id, n);
  controller.run(frequency);

  return 0;
}
