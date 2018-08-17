#include "um_ardrone/imu_printer.h"
  using sensor_msgs::Imu;
  using std::endl;
  using std::ostream;
  using std::string;

namespace um_ardrone
{

ImuPrinter::ImuPrinter(
  ostream& output_stream,
  MessagePrinter::OutputFormat output_format,
  const string& message_topic,
  size_t max_queue_size
)
: TemplatedMessagePrinter(
    output_stream,
    output_format,
    message_topic,
    max_queue_size
  )
{
  switch (output_format)
  {
    case (OutputFormat::CSV):
      output_stream << "sec, nsec, frame_id, w, x, y, z, dr, dp, dyaw, ax, ay, az\n";
    default:
      break;
  } // switch
}

ostream& ImuPrinter::printToStream_HUMAN_READABLE(
  ostream& os,
  const Imu::ConstPtr& msg
)
{
  const std_msgs::Header& header = msg->header;
  const geometry_msgs::Quaternion& orientation      = msg->orientation;
  const geometry_msgs::Vector3& angular_velocity    = msg->angular_velocity;
  const geometry_msgs::Vector3& linear_acceleration = msg->linear_acceleration;

  return os << "========================================"
            << "\nstampsec:\t" << header.stamp.sec
            << "\n    nsec:\t" << header.stamp.nsec
            << "\nframe_id:\t" << header.frame_id
            << "\nw:\t"        << orientation.w
            << "\nx:\t"        << orientation.x
            << "\ny:\t"        << orientation.y
            << "\nz:\t"        << orientation.z
            << "\nd-r:\t"      << angular_velocity.x
            << "\nd-p:\t"      << angular_velocity.y
            << "\nd-yaw:\t"    << angular_velocity.z
            << "\nax\t"        << linear_acceleration.x
            << "\nay:\t"       << linear_acceleration.y
            << "\naz:\t"       << linear_acceleration.z << endl;
}

ostream& ImuPrinter::printToStream_CSV(
  ostream& os,
  const Imu::ConstPtr& msg
)
{
  const std_msgs::Header& header = msg->header;
  const geometry_msgs::Quaternion& orientation      = msg->orientation;
  const geometry_msgs::Vector3& angular_velocity    = msg->angular_velocity;
  const geometry_msgs::Vector3& linear_acceleration = msg->linear_acceleration;

  return os << header.stamp.sec      << ", "
            << header.stamp.nsec     << ", "
            << header.frame_id       << ", "
            << orientation.w         << ", "
            << orientation.x         << ", "
            << orientation.y         << ", "
            << orientation.z         << ", "
            << angular_velocity.x    << ", "
            << angular_velocity.y    << ", "
            << angular_velocity.z    << ", "
            << linear_acceleration.x << ", "
            << linear_acceleration.y << ", "
            << linear_acceleration.z << '\n';
}

} // namespace um_ardrone
