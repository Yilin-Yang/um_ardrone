#include "um_ardrone/odometry_rebroadcaster.h"
  using nav_msgs::Odometry;
  using std::array;
  using std::string;

#include <cstring>
  using std::memcpy;

namespace um_ardrone
{

OdometryRebroadcaster::OdometryRebroadcaster(
  const string& subscribed_topic,
  const string& published_topic,
  const string& tf_frame_id_in,
  const string& child_tf_frame_id_in,
  const array<double, 36>& pose_covar_in,
  const array<double, 36>& twist_covar_in,
  size_t max_sub_queue_size,
  size_t max_pub_queue_size
)
: TemplatedRebroadcaster(
    subscribed_topic,
    published_topic,
    max_sub_queue_size,
    max_pub_queue_size
  ),
  tf_frame_id{tf_frame_id_in},
  child_tf_frame_id{child_tf_frame_id_in}
{
  memcpy(pose_covar.data(),  pose_covar_in.data(),  NUM_MATRIX_CHARS);
  memcpy(twist_covar.data(), twist_covar_in.data(), NUM_MATRIX_CHARS);
}

void OdometryRebroadcaster::receiveMessage(
  const ros::MessageEvent<const Odometry>& odom_msg
)
{
  // copy construct message
  Odometry::Ptr new_odom = boost::make_shared<Odometry>(
    *( odom_msg.getMessage() )
  );

  // modify message
  setTfFrames(new_odom);
  setCovarianceMatrices(new_odom);

  // rebroadcast message
  TemplatedRebroadcaster::receiveMessage(
    ros::MessageEvent<const Odometry>(new_odom)
  );
}

void OdometryRebroadcaster::setTfFrames(Odometry::Ptr msg)
{
  msg->header.frame_id = tf_frame_id;
  msg->child_frame_id = child_tf_frame_id;
}

void OdometryRebroadcaster::setCovarianceMatrices(Odometry::Ptr msg)
{
  boost::array<double, 36>& msg_pose_cov  = msg->pose.covariance;
  boost::array<double, 36>& msg_twist_cov = msg->twist.covariance;

  memcpy(msg_pose_cov.data(),  pose_covar.data(),  NUM_MATRIX_CHARS);
  memcpy(msg_twist_cov.data(), twist_covar.data(), NUM_MATRIX_CHARS);
}

} // namespace um_ardrone
