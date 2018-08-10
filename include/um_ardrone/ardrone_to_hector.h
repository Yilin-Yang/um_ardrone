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

#ifndef UM_ARDRONE_ARDRONE_TO_HECTOR_H
#define UM_ARDRONE_ARDRONE_TO_HECTOR_H

#include <ros/ros.h>
#include "um_ardrone/templated_rebroadcaster.h"

#include <memory>
#include <string>
#include <vector>

namespace um_ardrone
{

/**
 * @brief Rebroadcasts Parrot AR.Drone messages on `hector_localization` topics.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details Does not necessarily spin.
 */
class ARDroneToHector
{
public:

  /**
   * @brief Construct an ARDroneToHector message forwarder.
   */
  explicit ARDroneToHector(
    bool broadcast_imu      = true,
    bool broadcast_mag      = true,
    bool broadcast_altitude = true
  );

  static const std::string ARDRONE_IMU_TOPIC;     ///< imu messages
  static const std::string ARDRONE_MAG_TOPIC;     ///< magnetometer headings
  static const std::string ARDRONE_NAVDATA_TOPIC; ///< legacy navdata

  static const std::string HECTOR_IMU_TOPIC; ///< imu messages
  static const std::string HECTOR_MAG_TOPIC; ///< magnetometer headings
  static const std::string HECTOR_ALT_TOPIC; ///< altitude from barometer(?)

  // TODO verify that height LIDAR/SONAR is acceptable input to "pressure_height"

private:

  std::vector<std::unique_ptr<Rebroadcaster>> rebroadcasters;

}; // class ARDroneToHector

} // namespace um_ardrone

#endif // UM_ARDRONE_ARDRONE_TO_HECTOR_H
