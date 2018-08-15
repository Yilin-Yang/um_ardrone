// MIT License
//
// Copyright (c) 2018 Yilin Yang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef UM_ARDRONE_TEMPLATED_REBROADCASTER_H
#define UM_ARDRONE_TEMPLATED_REBROADCASTER_H

#include "rebroadcaster.h"
#include <stdexcept>
#include <string>
#include <typeinfo>

namespace um_ardrone
{
/**
 * @brief Primary implementation of Rebroadcaster. Templated on two message types.
 * @author Yilin Yang (yiliny@umich.edu)
 * @tparam MsgTypeSub The message type to be received, e.g. `std_msgs::String`.
 * @tparam MsgTypePub The (potentially different) message type to be rebroadcast.
 * @details This exists as a separate data type so that multiple Rebroadcasters
 *    can be stored in a templated container.
 */
template <typename MsgTypeSub, typename MsgTypePub = MsgTypeSub>
class TemplatedRebroadcaster : public Rebroadcaster
{
public:

  typedef ros::MessageEvent<const MsgTypeSub> MsgTypeSubEvent;

  TemplatedRebroadcaster() = default;

  /**
   * @brief Construct a TemplatedRebroadcaster object.
   * @details Given topics should be "absolute," i.e. they should contain the
   *    "full name" of the topic, including the topic's namespace.
   * @param max_sub_queue_size  The maximum number of messages that can be kept
   *    in the subscriber queue before incoming messages start being dropped.
   *    `0` means an infinite queue.
   * @param max_pub_queue_size  Like `max_sub_queue_size`, but for publishing.
   */
  explicit TemplatedRebroadcaster(
    const std::string& subscribed_topic,
    const std::string& published_topic,
    size_t max_sub_queue_size = 0,
    size_t max_pub_queue_size = 0
  );

  /**
   * @brief Callback function that receives messages.
   */
  virtual void receiveMessage(const MsgTypeSubEvent&);

  /**
   * @brief Broadcasts the given message onto `published_topic`.
   */
  virtual void rebroadcastMessage(const typename MsgTypePub::ConstPtr&);

  /**
   * @brief Convert between the two message types.
   */
  virtual typename MsgTypePub::ConstPtr convertSubToPub(
    const typename MsgTypeSub::ConstPtr&
  );

private:

  ros::NodeHandle node_handle;

  ros::Subscriber subscriber;

  ros::Publisher publisher;

}; // class TemplatedRebroadcaster

template <typename MsgTypeSub, typename MsgTypePub>
TemplatedRebroadcaster<MsgTypeSub, MsgTypePub>::TemplatedRebroadcaster(
  const std::string& subscribed_topic,
  const std::string& published_topic,
  size_t max_sub_queue_size,
  size_t max_pub_queue_size
)
: node_handle{},
  subscriber{
    node_handle.subscribe(
      subscribed_topic,
      max_sub_queue_size,
      &TemplatedRebroadcaster<MsgTypeSub, MsgTypePub>::receiveMessage,
      this
    )
  },
  publisher{
    node_handle.advertise<MsgTypePub>(
      published_topic,
      max_pub_queue_size
    )
  }
{
  ROS_INFO("Subscribing to topic: %s", subscribed_topic.c_str());
  ROS_INFO("Advertising topic: %s", published_topic.c_str());
}

template <typename MsgTypeSub, typename MsgTypePub>
void TemplatedRebroadcaster<MsgTypeSub, MsgTypePub>::receiveMessage(
  const MsgTypeSubEvent& msg_event
)
{
  if constexpr (std::is_same<MsgTypeSub, MsgTypePub>::value)
  {
    rebroadcastMessage(msg_event.getMessage());
  }
  else
  {
    auto to_publish{
      convertSubToPub(msg_event.getMessage())
    };
    rebroadcastMessage(to_publish);
  }
}

template <typename MsgTypeSub, typename MsgTypePub>
void TemplatedRebroadcaster<MsgTypeSub, MsgTypePub>::rebroadcastMessage(
  const typename MsgTypePub::ConstPtr& msg
)
{
  publisher.publish(msg);
}

template <typename MsgTypeSub, typename MsgTypePub>
typename MsgTypePub::ConstPtr
TemplatedRebroadcaster<MsgTypeSub, MsgTypePub>::convertSubToPub(
  const typename MsgTypeSub::ConstPtr&
)
{
  throw std::logic_error{
    std::string{
      "(um_ardrone) Tried to use a type-converting "
      "TemplatedRebroadcaster without overriding its convertSubToPub function! "
      "(Did not define a conversion between "
    } + typeid(MsgTypeSub).name()
    + std::string{" and "} + typeid(MsgTypePub).name() + std::string{"!)"}
  };
  return typename MsgTypePub::ConstPtr{}; // silence compiler error
}

} // namespace um_ardrone

#endif // UM_ARDRONE_TEMPLATED_REBROADCASTER_H
