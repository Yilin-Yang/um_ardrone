#include "um_ardrone/navdata_altitude_rebroadcaster.h"
  using boost::make_shared;

  using ardrone_autonomy::Navdata;
  using geometry_msgs::Point;
  using geometry_msgs::PointStamped;

namespace um_ardrone
{

NavdataAltitudeRebroadcaster::NavdataAltitudeRebroadcaster(
  const std::string& subscribed_topic,
  const std::string& published_topic,
  size_t max_sub_queue_size,
  size_t max_pub_queue_size
)
: TemplatedRebroadcaster(
    subscribed_topic,
    published_topic,
    max_sub_queue_size,
    max_pub_queue_size
  )
{
}

PointStamped::ConstPtr NavdataAltitudeRebroadcaster::convertSubToPub(
  const Navdata::ConstPtr& received_ptr
) const
{
  const Navdata& received = *received_ptr;

  PointStamped::Ptr altitude_msg = make_shared<PointStamped>();

  altitude_msg->header = received.header;

  // TODO: do the X and Y fields of the point matter?
  altitude_msg->point.x = 0;
  altitude_msg->point.y = 0;

  // `altd` is given in mm
  // (message specification appears to have a typo)
  altitude_msg->point.z = static_cast<double>(received.altd) / 1000.0;

  return altitude_msg;
}

} // namespace um_ardrone
