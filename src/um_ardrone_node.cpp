#include <ros/ros.h>
#include "um_ardrone/ardrone_to_robot_localization.h"
  using um_ardrone::ARDroneToRobotLocalization;

#include <string>
  using std::string;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "um_ardrone");
  ros::NodeHandle node_handle;

  ROS_INFO("Initializing um_ardrone node.");
  ROS_INFO("Loading parameters...");

  bool broadcast_mag      = true;
  bool broadcast_altitude = true;
  bool broadcast_tum_ekf  = true;

  string map_tf_frame_id       = "um_map";
  string odom_tf_frame_id      = "um_odom";
  string base_link_tf_frame_id = "um_base_link";

  node_handle.getParam("broadcast_mag",      broadcast_mag);
  node_handle.getParam("broadcast_altitude", broadcast_altitude);
  node_handle.getParam("broadcast_tum_ekf",  broadcast_tum_ekf);

  node_handle.getParam("map_tf_frame_id",       map_tf_frame_id);
  node_handle.getParam("odom_tf_frame_id",      odom_tf_frame_id);
  node_handle.getParam("base_link_tf_frame_id", base_link_tf_frame_id);

  // rebroadcast all AR.Drone messages
  ARDroneToRobotLocalization rebroadcaster{
    map_tf_frame_id,
    odom_tf_frame_id,
    base_link_tf_frame_id,
    broadcast_altitude,
    broadcast_mag,
    broadcast_tum_ekf
  };

  // if i
  ros::spin(); // for you like your favorite records used to,
  // (doot, doot DOOO doo dOO)

  return 0;
}
