#include "um_ardrone/ardrone_to_robot_localization.h"
  using std::make_unique;
  using std::string;

namespace um_ardrone
{
const string ARDroneToRobotLocalization::ARDRONE_IMU_TOPIC     = "ardrone/imu";
const string ARDroneToRobotLocalization::ARDRONE_MAG_TOPIC     = "ardrone/mag";
const string ARDroneToRobotLocalization::ARDRONE_NAVDATA_TOPIC = "ardrone/navdata";

const string ARDroneToRobotLocalization::TUM_EKF_TOPIC = "ardrone/predictedPose";

const string ARDroneToRobotLocalization::UM_ALT_TOPIC     = "um/sonar_altitude";
const string ARDroneToRobotLocalization::UM_MAG_TOPIC     = "um/mag_orientation";
const string ARDroneToRobotLocalization::UM_TUM_EKF_TOPIC = "um/tum_ekf";

ARDroneToRobotLocalization::ARDroneToRobotLocalization(
  const string& map_tf_frame_id_in,
  const string& odom_tf_frame_id_in,
  const string& base_link_tf_frame_id_in,
  bool enable_altitude,
  bool enable_mag,
  bool enable_tum_ekf
)
: map_tf_frame_id{map_tf_frame_id_in},
  odom_tf_frame_id{odom_tf_frame_id_in},
  base_link_tf_frame_id{base_link_tf_frame_id_in}
{
  if (enable_mag)
  {
    rebroadcasters.emplace_back(
      make_unique<MagImuRebroadcaster>(
        ARDRONE_MAG_TOPIC,
        UM_MAG_TOPIC,
        base_link_tf_frame_id
      )
    );
  }
  if (enable_altitude)
  {
    rebroadcasters.emplace_back(
      make_unique<NavdataAltitudeRebroadcaster>(
        ARDRONE_NAVDATA_TOPIC,
        UM_ALT_TOPIC,
        base_link_tf_frame_id
      )
    );
  }
  if (enable_tum_ekf)
  {
    rebroadcasters.emplace_back(
      make_unique<TumEkfRebroadcaster>(
        TUM_EKF_TOPIC,
        UM_TUM_EKF_TOPIC,
        map_tf_frame_id,
        base_link_tf_frame_id
      )
    );
  }
}

} // namespace um_ardrone
