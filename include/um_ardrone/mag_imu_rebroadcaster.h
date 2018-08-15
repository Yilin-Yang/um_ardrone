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

#ifndef UM_ARDRONE_MAG_IMU_REBROADCASTER_H
#define UM_ARDRONE_MAG_IMU_REBROADCASTER_H

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>

#include "templated_rebroadcaster.h"

namespace um_ardrone
{

/**
 * @brief Republish magnetometer vectors as IMU messages.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details See @ref TemplatedRebroadcaster for details on function parameters.
 */
class MagImuRebroadcaster
: public TemplatedRebroadcaster<
    geometry_msgs::Vector3Stamped,
    sensor_msgs::Imu
  >
{
public:

  MagImuRebroadcaster() = default;

  explicit MagImuRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    const std::string& tf_frame_id,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  /**
   * @brief Return an IMU message with orientation data and angular rates.
   * @details The values of all other fields in the pose message (aside from
   *    the header) are unspecified.
   */
  virtual sensor_msgs::Imu::ConstPtr convertSubToPub(
    const geometry_msgs::Vector3Stamped::ConstPtr&
  ) override;

  /**
   * @return A "default-initialized" Imu message: identity orientation,
   *    zero-vector angular velocity and linear acceleration, zeroed covariance
   *    matrices.
   */
  static sensor_msgs::Imu::Ptr defaultImuMessage();

  /**
   * @return A quaternion rotation from the `from` vector to the `to` vector.
   */
  static geometry_msgs::Quaternion transformFrom(
    const geometry_msgs::Vector3Stamped::ConstPtr& from,
    const geometry_msgs::Vector3Stamped::ConstPtr& to
  );

private:

  /**
   * @brief The `tf` frame in which magnetometer values are to be reported.
   */
  std::string tf_frame_id;

  /**
   * @brief The magnetic heading on first initialization. Assumed to be "level".
   */
  geometry_msgs::Vector3Stamped::ConstPtr initial_mag;

  /**
   * @brief The last magnetic heading received.
   */
  geometry_msgs::Vector3Stamped::ConstPtr last_mag;

}; // class MagImuRebroadcaster

} // namespace um_ardrone

#endif // UM_ARDRONE_MAG_IMU_REBROADCASTER_H
