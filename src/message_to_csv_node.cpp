#include <ros/ros.h>

#include "um_ardrone/altitude_printer.h"
#include "um_ardrone/imu_printer.h"
#include "um_ardrone/odometry_printer.h"
  using std::cerr;
  using std::ostream;
  using std::string;
  using um_ardrone::AltitudePrinter;
  using um_ardrone::ImuPrinter;
  using um_ardrone::MessagePrinter;
  using um_ardrone::OdometryPrinter;


#include <fstream>
  using std::ofstream;
#include <memory>
  using std::make_unique;
  using std::make_shared;
  using std::unique_ptr;
  using std::shared_ptr;
#include <stdexcept>
  using std::runtime_error;
#include <typeinfo>
#include <utility>
  using std::move;
#include <vector>
  using std::vector;

const string DEFAULT_FORMAT{"HUMAN_READABLE"};

/**
 * @brief Constructor parameters for printer objects.
 * @details Shall continue to exist until all file output has finished.
 */
struct PrinterParams
{
  explicit PrinterParams(const string& name_in)
  : name{name_in}
  {}

  virtual ~PrinterParams() {}

  /**
   * @brief Return a `unique_ptr` to the newly constructed MessagePrinter.
   * @details Initializes `outfile_ptr`.
   */
  virtual unique_ptr<MessagePrinter> makePrinter() = 0;

  // generate ROS parameter names
  virtual string paramShouldLog() { return string{ "log_"} + name; }
  virtual string paramFormat()    { return name + string{"_format"}; }
  virtual string paramTopic()     { return name + string{"_topic"}; }
  virtual string paramOutfile()   { return name + string{"_print_to_outfile"}; }

  string name; ///< The printer "prefix" to be used with the above parameters.

  bool should_log{false};
  string format;
  string topic;
  string outfile;

  std::unique_ptr<ofstream> outfile_ptr{nullptr};

}; // struct PrinterParams

template <typename PrinterType>
struct PrinterParamsTemplate : public PrinterParams
{
  explicit PrinterParamsTemplate(const string& name_in)
  : PrinterParams(name_in)
  {}
  virtual unique_ptr<MessagePrinter> makePrinter() override
  {
    string type_name = typeid(PrinterType).name();
    if (name.empty())
    {
      throw runtime_error{
        string{
          "(C++ Error) No name prefix given in instantiation of "
          "PrinterParamsTemplate with template parameter: "
        } + type_name
      };
    }
    if (outfile.empty())
    {
      throw runtime_error{
        string{"Did not set an outfile for MessagePrinter: "}
        + type_name
      };
    }
    if (topic.empty())
    {
      throw runtime_error{
        string{"Did not set a subscription topic for MessagePrinter: "}
        + type_name
      };
    }
    outfile_ptr = make_unique<ofstream>(outfile.c_str());
    // TODO: documentation
    auto& outstream = (outfile == "stderr") ? std::cerr : *outfile_ptr;
    return make_unique<PrinterType>(
      outstream,
      MessagePrinter::outputFormatFromString(format),
      topic
    );
  }
}; // struct PrinterParamsTemplate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_to_csv");
  ros::NodeHandle node_handle{"~"};

  ROS_INFO("Initializing um_ardrone sensor_to_csv node.");
  ROS_INFO("Loading parameters.");

  vector<unique_ptr<MessagePrinter>> printers;

  // NOTE: enable construction of new printer types by adding them here
  vector<shared_ptr<PrinterParams>> printer_params{
    make_shared< PrinterParamsTemplate<ImuPrinter> >     ("imu"),
    make_shared< PrinterParamsTemplate<OdometryPrinter> >("odom"),
    make_shared< PrinterParamsTemplate<AltitudePrinter> >("sonar"),
  };

  // build all requested printer objects
  for (auto& params : printer_params)
  {
    node_handle.getParam(params->paramShouldLog(), params->should_log);
    if (not params->should_log) continue;

    node_handle.getParam(params->paramFormat(),  params->format);
    node_handle.getParam(params->paramTopic(),   params->topic);
    node_handle.getParam(params->paramOutfile(), params->outfile);

    printers.emplace_back( params->makePrinter() );
  }

  if (printers.empty())
  {
    ROS_ERROR("No message printers created! Exiting!");
    return 1;
  }

  ROS_INFO("Finished initializing.");

  ros::spin();

  while (printers.size()) printers.pop_back();

  return 0;
}
