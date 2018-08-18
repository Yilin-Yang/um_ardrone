// MIT License
//
// Copyright (c) 2018 Yilin Yang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef UM_ARDRONE_MATH_H
#define UM_ARDRONE_MATH_H

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace um_ardrone
{

static inline constexpr double degToRad(double deg)
{
  return (deg / 180) * M_PI;
}

/**
 * @return The roll, pitch, and yaw of the given quaternion about fixed world
 *    axes in a `geometry_msgs::Vector` (where x is roll, y is pitch, and z is
 *    yaw.)
 */
static inline geometry_msgs::Vector3
  quaternionToEuler(const geometry_msgs::Quaternion& quat_msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(quat_msg, quat);
  tf2::Matrix3x3 mat{quat};

  tf2Scalar roll, pitch, yaw;
  mat.getRPY(yaw, pitch, roll);

  geometry_msgs::Vector3 to_return;
  to_return.x = roll;
  to_return.y = pitch;
  to_return.z = yaw;

  return to_return;
}

} // namespace um_ardrone

#endif // UM_ARDRONE_MATH_H
