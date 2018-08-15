#include "um_ardrone/navdata_altitude_rebroadcaster.h"
  using boost::make_shared;

  using ardrone_autonomy::Navdata;
  using geometry_msgs::PoseWithCovarianceStamped;

namespace um_ardrone
{

NavdataAltitudeRebroadcaster::NavdataAltitudeRebroadcaster(
  const std::string& subscribed_topic,
  const std::string& published_topic,
  const std::string& tf_frame_id_in,
  size_t max_sub_queue_size,
  size_t max_pub_queue_size
)
: TemplatedRebroadcaster(
    subscribed_topic,
    published_topic,
    max_sub_queue_size,
    max_pub_queue_size
  ),
  tf_frame_id{tf_frame_id_in}
{
}

PoseWithCovarianceStamped::ConstPtr NavdataAltitudeRebroadcaster::convertSubToPub(
  const Navdata::ConstPtr& received_ptr
)
{
  const Navdata& received = *received_ptr;

  PoseWithCovarianceStamped::Ptr altitude_msg
    = make_shared<PoseWithCovarianceStamped>();

  altitude_msg->header = received.header;
  altitude_msg->header.frame_id = tf_frame_id;

  auto& altitude_pose_point       = altitude_msg->pose.pose.position;
  auto& altitude_pose_orientation = altitude_msg->pose.pose.orientation;

  // `altd` is given in mm
  // (message specification appears to have a typo)
  altitude_pose_point.z = static_cast<double>(received.altd) / 1000.0;
  altitude_msg->pose.covariance =
    boost::array<double, 36>{
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
    };

  return altitude_msg;
}

} // namespace um_ardrone
