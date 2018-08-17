#include "um_ardrone/odometry_printer.h"
  using std::endl;
  using std::ostream;
  using std::string;
  using nav_msgs::Odometry;
  using geometry_msgs::Vector3;

namespace um_ardrone
{

OdometryPrinter::OdometryPrinter(
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
      output_stream << "sec, nsec, frame_id, child_frame_id, dx, dy\n";
    default:
      break;
  } // switch
}

ostream& OdometryPrinter::printToStream_HUMAN_READABLE(
  ostream& os,
  const Odometry::ConstPtr& msg
)
{
  const std_msgs::Header& header = msg->header;
  const string& child_frame_id = msg->child_frame_id;
  const geometry_msgs::Vector3& linear_velocity = msg->twist.twist.linear;

  const geometry_msgs::Point& position = msg->pose.pose.position;
  const geometry_msgs::Quaternion& orientation = msg->pose.pose.orientation;
  const geometry_msgs::Vector3& angular_velocity = msg->twist.twist.linear;

  return os << "========================================"
            << "\nstampsec:\t" << header.stamp.sec
            << "\n    nsec:\t" << header.stamp.nsec
            << "\nframe_id:\t" << header.frame_id
            << "\nchildfid:\t" << child_frame_id
            << "\nx:\t"        << position.x
            << "\ny:\t"        << position.y
            << "\nz:\t"        << position.z
            << "\ndx:\t"       << linear_velocity.x
            << "\ndy:\t"       << linear_velocity.y
            << "\ndz:\t"       << linear_velocity.z
            << "\nQw:\t"       << orientation.w
            << "\nQx:\t"       << orientation.x
            << "\nQy:\t"       << orientation.y
            << "\nQz:\t"       << orientation.z
            << "\ndr:\t"       << angular_velocity.x
            << "\ndp:\t"       << angular_velocity.y
            << "\ndyaw:\t"     << angular_velocity.z << endl;
}

ostream& OdometryPrinter::printToStream_CSV(
  ostream& os,
  const Odometry::ConstPtr& msg
)
{
  const std_msgs::Header& header = msg->header;
  const string& child_frame_id = msg->child_frame_id;
  const geometry_msgs::Vector3& linear_velocity = msg->twist.twist.linear;

  return os << header.stamp.sec  << ", "
            << header.stamp.nsec << ", "
            << header.frame_id   << ", "
            << child_frame_id    << ", "
            << linear_velocity.x << ", "
            << linear_velocity.y << '\n';
}

} // namespace um_ardrone
