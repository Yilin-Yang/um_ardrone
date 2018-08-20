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

#ifndef UM_ARDRONE_IMU_REBROADCASTER_H
#define UM_ARDRONE_IMU_REBROADCASTER_H

#include "templated_rebroadcaster.h"
#include <sensor_msgs/Imu.h>
#include <array>

namespace um_ardrone
{

/**
 * @brief Rebroadcast corrected AR.Drone IMU messages.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details Makes the following changes:
 *      - Overwrites the `tf` frame with the frame specified during
 *      construction.
 *      - Overwrites the covariance matrices with those specified during
 *      construction.
 *      - "Rescales" the acceleration vector to have an acceleration vector
 *      of 9.81 m/s^2 while at rest.
 *      - Yaws the coordinate frame of the orientation measurements by
 *      minus-90 degrees (from "North-West-Up" to "East-North-Up" conventions,
 *      as required by `robot_localization`.)
 */
class ImuRebroadcaster : public TemplatedRebroadcaster<sensor_msgs::Imu>
{
public:

  ImuRebroadcaster() = default;

  /**
   * @brief Construct an IMU rebroadcaster.
   * @details Covariance matrices are row-major, with the zero-indexed element
   *    corresponding to the top-left element in the 3x3 matrix.
   *
   *    Note that ROS orientation covariance matrices correspond to Euler
   *    angles.
   * @param tf_frame_id       The `tf` frame ID to be written to the outgoing
   *                          messages' headers.
   * @param euler_rpy_covar   Orientation covariance matrix for world-fixed RPY,
   *                          to be written to outgoing messages.
   * @param ang_vel_covar     Angular velocity covariance matrix, to be written
   *                          to outgoing messages.
   * @param lin_acc_covar     Linear acceleration covariance matrix, to be
   *                          written to outgoing messages.
   * @param acceleration_rescale  Multiply linear acceleration vectors by this
   *                          value in the outgoing messages. To be used
   *                          when the acceleration vector does not have a
   *                          magnitude of 9.81m/s^2 while at rest.
   */
  explicit ImuRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    const std::string& tf_frame_id,
    const std::array<double, 9>& euler_rpy_covar,
    const std::array<double, 9>& ang_vel_covar,
    const std::array<double, 9>& lin_acc_covar,
    double acceleration_rescale = 1,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  virtual void receiveMessage(const ros::MessageEvent<const sensor_msgs::Imu>&);

  /**
   * @{
   * @brief Make necessary alterations to inbound messages before rebroadcasting.
   */
  void setTfFrame(sensor_msgs::Imu::Ptr);
  void setCovarianceMatrices(sensor_msgs::Imu::Ptr);
  void rescaleAccelerations(sensor_msgs::Imu::Ptr);
  void rotateOrientationFrame(sensor_msgs::Imu::Ptr);
  /**
   * @}
   */

private:

  std::string tf_frame_id;

  double acceleration_rescale;

  std::array<double, 9> euler_rpy_covar;
  std::array<double, 9> ang_vel_covar;
  std::array<double, 9> lin_acc_covar;

}; // class ImuRebroadcaster

} // namespace um_ardrone

#endif // UM_ARDRONE_IMU_REBROADCASTER_H
