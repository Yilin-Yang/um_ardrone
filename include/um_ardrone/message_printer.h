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

#ifndef UM_ARDRONE_MESSAGE_PRINTER_H
#define UM_ARDRONE_MESSAGE_PRINTER_H

#include <ros/ros.h>

#include <iostream>
#include <unordered_map>

namespace um_ardrone
{

/**
 * @brief Abstract interface for printing ROS messages to an output stream.
 * @author Yilin Yang (yiliny@umich.edu)
 */
class MessagePrinter
{
public:

  MessagePrinter() = default;

  virtual ~MessagePrinter() {}

  /**
   * @brief How the message data is to be formatted when printed.
   */
  enum class OutputFormat
  {
    /**
     * @brief Output that can be easily read while being printed to a terminal.
     */
    HUMAN_READABLE,

    /**
     * @brief Properly-formatted CSV output, able to be used in Excel, etc.
     */
    CSV,
  }; // enum class OutputFormat

  /**
   * @brief Construct a MessagePrinter object given its output stream and format.
   */
  explicit MessagePrinter(std::ostream& out_stream, OutputFormat format);

  /**
   * @brief Convert the given string into its corresponding OutputFormat.
   */
  static OutputFormat outputFormatFromString(const std::string& str)
  {
    static const std::unordered_map<std::string, OutputFormat> STRING_TO_FORMAT{
      {"HUMAN_READABLE", OutputFormat::HUMAN_READABLE},
      {"CSV",            OutputFormat::CSV},
    };

    auto it = STRING_TO_FORMAT.find(str);
    if (it == STRING_TO_FORMAT.end())
    {
      throw std::runtime_error{
        std::string{"Given string does not correspond to an OutputFormat: "}
        + str
      };
    }

    return (*it).second;
  } // outputFormatFromString

protected:

  OutputFormat getOutputFormat() const { return out_format; }

  inline std::ostream& getOutputStream() { return *os; }

private:

  std::ostream* os;

  OutputFormat out_format;

}; // class MessagePrinter

} // namespace um_ardrone

#endif // UM_ARDRONE_MESSAGE_PRINTER_H
