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

#ifndef UM_ARDRONE_ODOMETRY_PRINTER_H
#define UM_ARDRONE_ODOMETRY_PRINTER_H

#include "templated_message_printer.h"

#include <nav_msgs/Odometry.h>

namespace um_ardrone
{

/**
 * @brief Prints `nav_msgs::Odometry` messages to a CSV outfile.
 * @author Yilin Yang (yiliny@umich.edu)
 * @details As of the time of writing, this only prints linear velocities along
 *    the x- and y-axes.
 */
class OdometryPrinter
: public TemplatedMessagePrinter<nav_msgs::Odometry>
{
public:

  OdometryPrinter() = default;

  explicit OdometryPrinter(
    std::ostream& output_stream,
    MessagePrinter::OutputFormat output_format,
    const std::string& message_topic,
    size_t max_queue_size = 100
  );

protected:

  virtual std::ostream& printToStream_HUMAN_READABLE(
    std::ostream&,
    const nav_msgs::Odometry::ConstPtr&
  ) override;

  virtual std::ostream& printToStream_CSV(
    std::ostream&,
    const nav_msgs::Odometry::ConstPtr&
  ) override;

}; // class OdometryPrinter

} // namespace um_ardrone

#endif // UM_ARDRONE_ODOMETRY_PRINTER_H
