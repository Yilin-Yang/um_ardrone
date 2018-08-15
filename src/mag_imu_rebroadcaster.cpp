#include "um_ardrone/mag_imu_rebroadcaster.h"
  using boost::make_shared;

  using geometry_msgs::Vector3Stamped;
  using sensor_msgs::Imu;

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
  using std::fill;

namespace um_ardrone
{

MagImuRebroadcaster::MagImuRebroadcaster(
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
  tf_frame_id{tf_frame_id_in},
  initial_mag{nullptr},
  last_mag{nullptr}
{
  ROS_INFO("MagImuRebroadcaster tf_frame: %s", tf_frame_id.c_str());
}

Imu::Ptr MagImuRebroadcaster::defaultImuMessage()
{
  Imu::Ptr imu_msg = make_shared<Imu>();
  fill(
    imu_msg->orientation_covariance.begin(),
    imu_msg->orientation_covariance.end(),
    0
  );
  fill(
    imu_msg->angular_velocity_covariance.begin(),
    imu_msg->angular_velocity_covariance.end(),
    0
  );
  fill(
    imu_msg->linear_acceleration_covariance.begin(),
    imu_msg->linear_acceleration_covariance.end(),
    0
  );
  imu_msg->orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  imu_msg->angular_velocity.x = 0;
  imu_msg->angular_velocity.y = 0;
  imu_msg->angular_velocity.z = 0;

  imu_msg->linear_acceleration.x = 0;
  imu_msg->linear_acceleration.y = 0;
  imu_msg->linear_acceleration.z = 0;

  return imu_msg;
}

geometry_msgs::Quaternion MagImuRebroadcaster::transformFrom(
  const geometry_msgs::Vector3Stamped::ConstPtr& from_geo,
  const geometry_msgs::Vector3Stamped::ConstPtr& to_geo
)
{
  Eigen::Vector3d from, to;
  tf2::fromMsg(from_geo->vector, from);
  tf2::fromMsg(  to_geo->vector,   to);

  return tf2::toMsg(
    Eigen::Quaterniond::FromTwoVectors(from, to)
  );
}

Imu::ConstPtr MagImuRebroadcaster::convertSubToPub(
  const Vector3Stamped::ConstPtr& received_ptr
)
{
  const Vector3Stamped& received = *received_ptr;

  Imu::Ptr mag_msg = MagImuRebroadcaster::defaultImuMessage();

  mag_msg->header = received.header;
  mag_msg->header.frame_id = tf_frame_id;

  if (not initial_mag)
  {
    initial_mag = received_ptr;
    last_mag    = received_ptr;
    return mag_msg;
  }

  // set orientation
  mag_msg->orientation = transformFrom(last_mag, received_ptr);

  // calculate angular rates
  {
    Eigen::Vector3d received_vec, last_vec;
    tf2::fromMsg(received.vector, received_vec);
    tf2::fromMsg(last_mag->vector, last_vec);
    double dt = received.header.stamp.toSec() - last_mag->header.stamp.toSec();
    tf2::toMsg(
      (received_vec - last_vec) / dt,
      mag_msg->angular_velocity
    );
  }

  last_mag = received_ptr;
  return mag_msg;
}

} // namespace um_ardrone
