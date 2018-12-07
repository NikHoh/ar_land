#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

namespace tools_func {

tf::Vector3 convertToTFVector3(geometry_msgs::Vector3 vec)
{
  tf::Vector3 out;
  out.setValue(vec.x,vec.y,vec.z);
  return out;
}

bool isIdentity(const geometry_msgs::TransformStamped &msg){
  bool transIsId = false;
  bool rotIsId = false;

  if(msg.transform.translation.x == 0 && msg.transform.translation.y == 0 && msg.transform.translation.z == 0)
  {
    transIsId= true;
  }
  if(msg.transform.rotation.w == 1 && msg.transform.rotation.x == 0 && msg.transform.rotation.y == 0 && msg.transform.rotation.z == 0)
  {
    rotIsId = true;
  }

  return rotIsId&&transIsId;

}

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
} // namespace tools_func

#endif // TOOLS_H
