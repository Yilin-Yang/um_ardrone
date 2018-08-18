#include "um_ardrone/altitude_printer.h"
  using geometry_msgs::PoseWithCovarianceStamped;
  using std::endl;
  using std::ostream;
  using std::string;

namespace um_ardrone
{

AltitudePrinter::AltitudePrinter(
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
      output_stream << "sec, nsec, frame_id, z\n";
    default:
      break;
  } // switch
}

ostream& AltitudePrinter::printToStream_HUMAN_READABLE(
  ostream& os,
  const PoseWithCovarianceStamped::ConstPtr& msg
)
{
  return os << "z:\t" << msg->pose.pose.position.z << endl;
}

ostream& AltitudePrinter::printToStream_CSV(
  ostream& os,
  const PoseWithCovarianceStamped::ConstPtr& msg
)
{
  const std_msgs::Header& header = msg->header;
  const geometry_msgs::Point position = msg->pose.pose.position;
  return os << header.stamp.sec  << ", "
            << header.stamp.nsec << ", "
            << header.frame_id   << ", "
            << position.z        << '\n';
}

} // namespace um_ardrone
