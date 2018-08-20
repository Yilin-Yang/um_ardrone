#include <ros/ros.h>

#include <string>
  using std::string;

/* #include <iostream> */
/*   using std::cerr; */
/*   using std::endl; */

/* void receiveMessage(const nav_msgs::Odometry::ConstPtr& msg) */
/* { */
/*   const auto& position = msg->pose.pose.position; */
/*   const auto& lin_vel = msg->twist.twist.linear; */
/*   const auto& ang_vel = msg->twist.twist.angular; */

/*   cerr << "========================================" */
/*        << "\nx:\t"      << position.x */
/*        << "\ny:\t"    << position.y */
/*        << "\nz:\t"    << position.z */
/*        << "\ndx:\t"   << lin_vel.x */
/*        << "\ndy:\t"   << lin_vel.y */
/*        << "\ndz:\t"   << lin_vel.z */
/*        << "\ndr:\t"   << ang_vel.x */
/*        << "\ndp:\t"   << ang_vel.y */
/*        << "\ndyaw:\t" << ang_vel.z << endl; */
/* } */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "um_ardrone");
  ros::NodeHandle node_handle{"~"};

  /* auto sub = node_handle.subscribe("/ardrone/odometry", 100, &receiveMessage); */

  /* ROS_INFO("Initializing um_ardrone node."); */
  /* ROS_INFO("Loading parameters..."); */

  /* bool broadcast_mag      = true; */
  /* bool broadcast_altitude = true; */
  /* bool broadcast_tum_ekf  = true; */

  /* string map_tf_frame_id       = "um_map"; */
  /* string odom_tf_frame_id      = "um_odom"; */
  /* string base_link_tf_frame_id = "um_base_link"; */

  /* node_handle.getParam("broadcast_mag",      broadcast_mag); */
  /* node_handle.getParam("broadcast_altitude", broadcast_altitude); */
  /* node_handle.getParam("broadcast_tum_ekf",  broadcast_tum_ekf); */

  /* node_handle.getParam("map_tf_frame_id",       map_tf_frame_id); */
  /* node_handle.getParam("odom_tf_frame_id",      odom_tf_frame_id); */
  /* node_handle.getParam("base_link_tf_frame_id", base_link_tf_frame_id); */

  /* // rebroadcast all AR.Drone messages */
  /* ARDroneToRobotLocalization rebroadcaster{ */
  /*   map_tf_frame_id, */
  /*   odom_tf_frame_id, */
  /*   base_link_tf_frame_id, */
  /*   broadcast_altitude, */
  /*   broadcast_mag, */
  /*   broadcast_tum_ekf */
  /* }; */
  /* um_ardrone::NavdataAltitudeRebroadcaster rebroadcaster{ */
  /*   "ardrone/navdata", */
  /*   "ardrone/altitude", */
  /*   "um_base_link", */
  /*   100, */
  /*   100 */
  /* }; */

  // if i
  ros::spin(); // for you like your favorite records used to,
  // (doot, doot DOOO doo dOO)

  return 0;
}
