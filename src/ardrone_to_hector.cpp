#include "um_ardrone/ardrone_to_hector.h"
  using std::make_unique;
  using std::string;
  using std::vector;

#include "um_ardrone/templated_rebroadcaster.h"
#include "um_ardrone/navdata_altitude_rebroadcaster.h"

#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

namespace um_ardrone
{
const string ARDroneToHector::ARDRONE_IMU_TOPIC     = "ardrone/imu";
const string ARDroneToHector::ARDRONE_MAG_TOPIC     = "ardrone/mag";
const string ARDroneToHector::ARDRONE_NAVDATA_TOPIC = "ardrone/navdata";

const string ARDroneToHector::HECTOR_IMU_TOPIC = "raw_imu";
const string ARDroneToHector::HECTOR_MAG_TOPIC = "magnetic";
const string ARDroneToHector::HECTOR_ALT_TOPIC = "pressure_height";

ARDroneToHector::ARDroneToHector(
  bool broadcast_imu,
  bool broadcast_mag,
  bool broadcast_altitude
)
{
  if (broadcast_imu)
  {
    rebroadcasters.emplace_back(
      make_unique<TemplatedRebroadcaster<sensor_msgs::Imu>>(
        ARDRONE_IMU_TOPIC,
        HECTOR_IMU_TOPIC
      )
    );
  }
  if (broadcast_mag)
  {
    rebroadcasters.emplace_back(
      make_unique<TemplatedRebroadcaster<geometry_msgs::Vector3Stamped>>(
        ARDRONE_MAG_TOPIC,
        HECTOR_MAG_TOPIC
      )
    );
  }
  if (broadcast_altitude)
  {
    rebroadcasters.emplace_back(
      make_unique<NavdataAltitudeRebroadcaster>(
        ARDRONE_NAVDATA_TOPIC,
        HECTOR_ALT_TOPIC
      )
    );
  }
}

} // namespace um_ardrone
