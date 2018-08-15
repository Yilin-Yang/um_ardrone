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

#ifndef UM_ARDRONE_NAVDATA_ALTITUDE_REBROADCASTER_H
#define UM_ARDRONE_NAVDATA_ALTITUDE_REBROADCASTER_H

#include <ros/ros.h>

#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "templated_rebroadcaster.h"

namespace um_ardrone
{

/**
 * @brief Extract SONAR from `Navdata`, republish as `PoseWithCovarianceStamped`.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details See @ref TemplatedRebroadcaster for details on function parameters.
 */
class NavdataAltitudeRebroadcaster
: public TemplatedRebroadcaster<
    ardrone_autonomy::Navdata,
    geometry_msgs::PoseWithCovarianceStamped
  >
{
public:

  NavdataAltitudeRebroadcaster() = default;

  explicit NavdataAltitudeRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    const std::string& tf_frame_id,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  /**
   * @brief Return a pose message containing z-height.
   * @details The values of all other fields in the pose message (aside from
   *    the header) are unspecified.
   */
  virtual geometry_msgs::PoseWithCovarianceStamped::ConstPtr convertSubToPub(
    const ardrone_autonomy::Navdata::ConstPtr&
  ) override;

private:

  /**
   * @brief The `tf` frame in which altitude measurements are to be reported.
   */
  std::string tf_frame_id;

}; // class NavdataAltitudeRebroadcaster

} // namespace um_ardrone

#endif // UM_ARDRONE_NAVDATA_ALTITUDE_REBROADCASTER_H
