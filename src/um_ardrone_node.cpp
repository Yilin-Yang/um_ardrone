#include <ros/ros.h>
#include "um_ardrone/ardrone_to_hector.h"
  using um_ardrone::ARDroneToHector;

#include <string>
  using std::string;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "um_ardrone");
  ros::NodeHandle node_handle;

  ROS_INFO("Initializing um_ardrone node.");
  ROS_INFO("Loading parameters...");

  bool broadcast_imu      = true;
  bool broadcast_mag      = true;
  bool broadcast_altitude = true;

  node_handle.getParam("broadcast_imu",      broadcast_imu);
  node_handle.getParam("broadcast_mag",      broadcast_mag);
  node_handle.getParam("broadcast_altitude", broadcast_altitude);

  ROS_INFO("========================================");
  ROS_INFO("Rebroadcasting:");
  ROS_INFO("AR.Drone IMU readings: %i",             broadcast_imu);
  ROS_INFO("AR.Drone magnetometer readings: %i",    broadcast_mag);
  ROS_INFO("AR.Drone sonar altimeter readings: %i", broadcast_altitude);

  // rebroadcast all AR.Drone messages
  ARDroneToHector rebroadcaster{
    broadcast_imu,
    broadcast_mag,
    broadcast_altitude
  };

  // if i
  ros::spin(); // for you like your favorite records used to,
  // (doot, doot DOOO doo dOO)

  return 0;
}
