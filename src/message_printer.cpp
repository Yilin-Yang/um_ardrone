#include "um_ardrone/message_printer.h"
  using std::ostream;

namespace um_ardrone
{

MessagePrinter::MessagePrinter(
  ostream& out_stream,
  MessagePrinter::OutputFormat format
)
: os{&out_stream},
  out_format{format}
{
}

} // namespace um_ardrone
