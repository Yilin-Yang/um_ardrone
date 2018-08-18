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

#ifndef UM_ARDRONE_ARDRONE_TO_ROBOT_LOCALIZATION_H
#define UM_ARDRONE_ARDRONE_TO_ROBOT_LOCALIZATION_H

#include "tum_ekf_rebroadcaster.h"
#include "navdata_altitude_rebroadcaster.h"

#include <memory>
#include <string>
#include <vector>

namespace um_ardrone
{

/**
 * @brief Republish properly converted AR.Drone messages to `robot_localization`.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details Does not necessarily spin.
 */
class ARDroneToRobotLocalization
{
public:

  ARDroneToRobotLocalization() = default;

  explicit ARDroneToRobotLocalization(
    const std::string& map_tf_frame_id,
    const std::string& odom_tf_frame_id,
    const std::string& base_link_tf_frame_id,
    bool enable_altitude,
    bool enable_mag,
    bool enable_tum_ekf
  );

  // TODO: make configurable
  static const std::string ARDRONE_IMU_TOPIC;     ///< imu messages
  static const std::string ARDRONE_MAG_TOPIC;     ///< magnetometer headings
  static const std::string ARDRONE_NAVDATA_TOPIC; ///< legacy navdata

  static const std::string TUM_EKF_TOPIC; ///< raw predictions from tum_ardrone

  static const std::string UM_ALT_TOPIC;     ///< altitude messages
  static const std::string UM_MAG_TOPIC;     ///< magnetometer headings
  static const std::string UM_TUM_EKF_TOPIC; ///< rebroadcast TUM EKF output

private:

  // see robot_localization docs and REP-105 for information on these variables
  std::string map_tf_frame_id;
  std::string odom_tf_frame_id;
  std::string base_link_tf_frame_id;

  std::vector<std::unique_ptr<Rebroadcaster>> rebroadcasters;

}; // class ARDroneToRobotLocalization

} // namespace um_ardrone

#endif // UM_ARDRONE_ARDRONE_TO_ROBOT_LOCALIZATION_H
