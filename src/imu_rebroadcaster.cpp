#include "um_ardrone/imu_rebroadcaster.h"
  using sensor_msgs::Imu;
  using std::array;
  using std::string;

#include <cstring>
  using std::memcpy;

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace um_ardrone
{

ImuRebroadcaster::ImuRebroadcaster(
  const string& subscribed_topic,
  const string& published_topic,
  const string& tf_frame_id_in,
  const array<double, 9>& euler_rpy_covar_in,
  const array<double, 9>& ang_vel_covar_in,
  const array<double, 9>& lin_acc_covar_in,
  double acceleration_rescale_in,
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
  acceleration_rescale{acceleration_rescale_in},
  euler_rpy_covar{euler_rpy_covar_in},
  ang_vel_covar{ang_vel_covar_in},
  lin_acc_covar{lin_acc_covar_in}
{
}

void ImuRebroadcaster::receiveMessage(
  const ros::MessageEvent<const Imu>& imu_msg
)
{
  // copy construct message
  Imu::Ptr new_imu = boost::make_shared<Imu>(
    *( imu_msg.getMessage() )
  );

  // modify message
  setTfFrame(new_imu);
  setCovarianceMatrices(new_imu);
  rescaleAccelerations(new_imu);
  rotateOrientationFrame(new_imu);

  // rebroadcast message
  TemplatedRebroadcaster::receiveMessage(imu_msg);
}

void ImuRebroadcaster::setTfFrame(Imu::Ptr imu_msg)
{
  imu_msg->header.frame_id = tf_frame_id;
}

void ImuRebroadcaster::setCovarianceMatrices(Imu::Ptr imu_msg)
{
  boost::array<double, 9>& msg_rpy_cov     = imu_msg->orientation_covariance;
  boost::array<double, 9>& msg_ang_vel_cov = imu_msg->angular_velocity_covariance;
  boost::array<double, 9>& msg_lin_acc_cov = imu_msg->linear_acceleration_covariance;

  static constexpr size_t NUM_MATRIX_CHARS = sizeof(double) * 9;
  memcpy(msg_rpy_cov.data(),     euler_rpy_covar.data(), NUM_MATRIX_CHARS);
  memcpy(msg_ang_vel_cov.data(), ang_vel_covar.data(),   NUM_MATRIX_CHARS);
  memcpy(msg_lin_acc_cov.data(), lin_acc_covar.data(),   NUM_MATRIX_CHARS);
}

void ImuRebroadcaster::rescaleAccelerations(Imu::Ptr imu_msg)
{
  if (acceleration_rescale == 1) return;

  geometry_msgs::Vector3& lin_acc = imu_msg->linear_acceleration;
  lin_acc.x *= acceleration_rescale;
  lin_acc.y *= acceleration_rescale;
  lin_acc.z *= acceleration_rescale;
}

void ImuRebroadcaster::rotateOrientationFrame(Imu::Ptr imu_msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(imu_msg->orientation, quat);

  static const tf2::Quaternion ROTATION(
    tf2::Vector3{0, 0, 1},
    -M_PI / 2
  );

  // TODO: quaternion rotations evaluated left-to-right?
  quat *= ROTATION;

  imu_msg->orientation = tf2::toMsg(quat);
}

} // namespace um_ardrone
