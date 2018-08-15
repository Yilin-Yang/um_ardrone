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

#ifndef UM_ARDRONE_TUM_EKF_REBROADCASTER_H
#define UM_ARDRONE_TUM_EKF_REBROADCASTER_H

#include <tum_ardrone/filter_state.h>
#include <nav_msgs/Odometry.h>

#include "templated_rebroadcaster.h"

namespace um_ardrone
{

/**
 * @brief Convert `filter_state` messages into `nav_msgs::Odometry`.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details `nav_msgs::Odometry` is one of the most "complete" of all
 *    ROS-compatible message types, including both pose (position, orientation,
 *    etc.) and rates (velocities).
 */
class TumEkfRebroadcaster
: public TemplatedRebroadcaster<
    tum_ardrone::filter_state,
    nav_msgs::Odometry
  >
{
public:

  TumEkfRebroadcaster() = default;

  explicit TumEkfRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    const std::string& world_tf_frame_id,
    const std::string& local_tf_frame_id,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  /**
   * @brief Return the converted `filter_state` message.
   */
  virtual nav_msgs::Odometry::ConstPtr convertSubToPub(
    const tum_ardrone::filter_state::ConstPtr&
  ) override;

  /**
   * @brief Set the pose field in the given `Odometry` message.
   */
  static void setPose(
    const tum_ardrone::filter_state::ConstPtr& from,
    nav_msgs::Odometry::Ptr& in
  );

  /**
   * @brief Set the twist field in the given `Odometry` message.
   */
  static void setTwist(
    const tum_ardrone::filter_state::ConstPtr& from,
    nav_msgs::Odometry::Ptr& in
  );

private:

  /**
   * @brief The `frame_id` in which pose values are reported.
   */
  std::string world_tf_frame_id;

  /**
   * @brief The `child_frame_id` in which "twist" values are reported.
   */
  std::string local_tf_frame_id;

}; // class TumEkfRebroadcaster

} // namespace um_ardrone


#endif // UM_ARDRONE_TUM_EKF_REBROADCASTER_H
