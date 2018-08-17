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

#ifndef UM_ARDRONE_TEMPLATED_MESSAGE_PRINTER_H
#define UM_ARDRONE_TEMPLATED_MESSAGE_PRINTER_H

#include "message_printer.h"

#include <stdexcept>
#include <string>
#include <typeinfo>

namespace um_ardrone
{

/**
 * @brief Prints ROS messages of a parameterized type to an output stream.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details TemplatedMessagePrinter internally implements its different
 *    "output formats" through a pImpl pointing to one of various member
 *    functions.
 *
 *    This isn't very extensible: adding new output formats would require
 *    adding a new virtual `printToStream` function to this class, as well as
 *    overridding that virtual function in all derived classes that might need
 *    to use it. Ideally, this class would have some generic means of
 *    "autoformatting" messages, but there isn't an easy way to do this in C++,
 *    which does not support Python-style iteration over class members.
 */
template <typename Msg>
class TemplatedMessagePrinter : public MessagePrinter
{
public:

  TemplatedMessagePrinter() = default;

  /**
   * @brief Construct a TemplatedMessagePrinter.
   * @param topic The topic to which this object should subscribe.
   */
  explicit TemplatedMessagePrinter(
    std::ostream& output_stream,
    MessagePrinter::OutputFormat,
    const std::string& topic,
    size_t max_queue_size = 100
  );

  /**
   * @brief Callback function that receives incoming messages.
   */
  virtual void receiveMessage(const typename Msg::ConstPtr&);

protected:

  virtual std::ostream& printToStream_HUMAN_READABLE(
    std::ostream&,
    const typename Msg::ConstPtr&
  )
  {
    throw not_implemented;
  }

  virtual std::ostream& printToStream_CSV(
    std::ostream&,
    const typename Msg::ConstPtr&
  )
  {
    throw not_implemented;
  }

private:

  /**
   * @brief Generic error message used when print functions aren't overridden.
   */
  std::runtime_error not_implemented{
    std::string{"printToStream function for given format not implemented for "}
    + typeid(*this).name()
  };

  ros::NodeHandle node_handle;

  ros::Subscriber subscriber;

  // grotesque pointer-to-member-function typedef
  typedef
    std::ostream&
      (TemplatedMessagePrinter::*FormattedPrintFunction)(
        std::ostream&,
        const typename Msg::ConstPtr&
      );

  FormattedPrintFunction print_to_stream_impl{nullptr};

}; // class TemplatedMessagePrinter

template <typename Msg>
TemplatedMessagePrinter<Msg>::TemplatedMessagePrinter(
  std::ostream& os,
  OutputFormat format,
  const std::string& topic,
  size_t max_queue_size
)
: MessagePrinter(os, format),
  subscriber{
    node_handle.subscribe(
      topic,
      max_queue_size,
      &TemplatedMessagePrinter<Msg>::receiveMessage,
      this
    )
  }
{
  std::string* format_name{nullptr};
  switch (format)
  {
    case OutputFormat::HUMAN_READABLE:
      print_to_stream_impl = &TemplatedMessagePrinter::printToStream_HUMAN_READABLE;
      format_name = new std::string{"HUMAN_READABLE"};
      break;
    case OutputFormat::CSV:
      print_to_stream_impl = &TemplatedMessagePrinter::printToStream_CSV;
      format_name = new std::string{"CSV"};
      break;
    default:
      throw std::runtime_error{
        std::string{"(TemplatedMessagePrinter) Unrecognized format type: "}
        + std::to_string(static_cast<int>(format))
      };
  } // switch

  ROS_INFO(
    "MessagePrinter subscribing to topic: %s "
    "and will print in format %s",
    topic.c_str(),
    format_name->c_str()
  );
  delete format_name;
}

template <typename Msg>
void TemplatedMessagePrinter<Msg>::receiveMessage(
  const typename Msg::ConstPtr& message_ptr
)
{
  (this->*print_to_stream_impl)(MessagePrinter::getOutputStream(), message_ptr);
}

} // namespace um_ardrone

#endif // UM_ARDRONE_TEMPLATED_MESSAGE_PRINTER_H
