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

#ifndef UM_ARDRONE_ALTITUDE_PRINTER_H
#define UM_ARDRONE_ALTITUDE_PRINTER_H

#include "um_ardrone/templated_message_printer.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace um_ardrone
{

/**
 * @brief Prints sonar altitude to an output stream.
 * @author Yilin Yang (yiliny@umich.edu)
 */
class AltitudePrinter
: public TemplatedMessagePrinter<
    geometry_msgs::PoseWithCovarianceStamped
  >
{
public:

  AltitudePrinter() = default;

  explicit AltitudePrinter(
    std::ostream& output_stream,
    MessagePrinter::OutputFormat output_format,
    const std::string& message_topic,
    size_t max_queue_size = 100
  );

protected:

  virtual std::ostream& printToStream_HUMAN_READABLE(
    std::ostream&,
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
  ) override;

  virtual std::ostream& printToStream_CSV(
    std::ostream&,
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
  ) override;

}; // class AltitudePrinter

} // namespace um_ardrone

#endif // UM_ARDRONE_ALTITUDE_PRINTER_H
