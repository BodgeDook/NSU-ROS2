#ifndef TURTLE_MULTI_TARGET_UTILITIES_HPP_
#define TURTLE_MULTI_TARGET_UTILITIES_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

namespace turtle_multi_target
{

geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Quaternion q;
  
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  
  return q;
}

}

#endif
