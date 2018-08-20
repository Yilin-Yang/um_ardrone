#include "um_ardrone/navdata_altitude_rebroadcaster.h"
  using ardrone_autonomy::Navdata;
  using geometry_msgs::PoseWithCovarianceStamped;
  using boost::make_shared;
  using std::string;

#include <cstring>
  using std::memcpy;

namespace um_ardrone
{

NavdataAltitudeRebroadcaster::NavdataAltitudeRebroadcaster(
  const string& subscribed_topic,
  const string& published_topic,
  const string& tf_frame_id_in,
  const std::array<double, 36>& pose_covar_in,
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
  ROS_INFO("NavdataAltitudeRebroadcaster tf_frame: %s", tf_frame_id.c_str());
  memcpy(pose_covar.data(), pose_covar_in.data(), NUM_MATRIX_CHARS);
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
  altitude_pose_point.x = 0;
  altitude_pose_point.y = 0;
  altitude_pose_point.z = static_cast<double>(received.altd) / 1000.0;

  altitude_pose_orientation.w = 1;
  altitude_pose_orientation.x = 0;
  altitude_pose_orientation.y = 0;
  altitude_pose_orientation.z = 0;

  memcpy(
    altitude_msg->pose.covariance.data(),
    pose_covar.data(),
    NUM_MATRIX_CHARS
  );

  return altitude_msg;
}

} // namespace um_ardrone
