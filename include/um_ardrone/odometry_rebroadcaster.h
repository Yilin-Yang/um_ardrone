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

#ifndef UM_ARDRONE_ODOMETRY_REBROADCASTER_H
#define UM_ARDRONE_ODOMETRY_REBROADCASTER_H

#include "templated_rebroadcaster.h"
#include <nav_msgs/Odometry.h>
#include <array>

namespace um_ardrone
{

/**
 * @brief Rebroadcast corrected AR.Drone odometry messages.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details Makes the following changes:
 *      - Overwrites the message's `tf` frames with the frames specified during
 *      construction.
 *      - Overwrites the message's covariance matrices with those specified
 *      during construction.
 */
class OdometryRebroadcaster : public TemplatedRebroadcaster<nav_msgs::Odometry>
{
public:

  OdometryRebroadcaster() = default;

  /**
   * @brief Construct an odometry message rebroadcaster.
   * @details Covariance matrices are row-major, with the zero-indexed element
   *    corresponding to the top-left element in the 6x6 matrix.
   * @param tf_frame_id       The `tf` frame ID in which the message's pose
   *                          is specified.
   * @param child_tf_frame_id The `tf` frame ID in which the message's twist
   *                          (i.e. its velocities) are specified.
   * @param pose_covar        The 6x6 pose covariance matrix (x, y, z, r, p, yaw)
   *                          to be written to the outgoing odometry message.
   * @param twist_covar       The 6x6 twist covariance matrix (dx, dy, dz, dr,
   *                          dp, dyaw) to be written to the outgoing odometry
   *                          message.
   */
  explicit OdometryRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    const std::string& tf_frame_id,
    const std::string& child_tf_frame_id,
    const std::array<double, 36>& pose_covar,
    const std::array<double, 36>& twist_covar,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  void setTfFrames(nav_msgs::Odometry::Ptr);
  void setCovarianceMatrices(nav_msgs::Odometry::Ptr);

private:

  std::string tf_frame_id;
  std::string child_tf_frame_id;

  /**
   * @brief The number of bytes in a 6x6 covariance matrix.
   */
  static constexpr size_t NUM_MATRIX_CHARS = sizeof(double) * 36;

  boost::array<double, 36> pose_covar;
  boost::array<double, 36> twist_covar;

}; // class OdometryRebroadcaster

} // namespace um_ardrone

#endif // UM_ARDRONE_ODOMETRY_REBROADCASTER_H
