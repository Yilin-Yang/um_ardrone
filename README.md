um_ardrone
================================================================================
Experimental autonomous flight with a Parrot AR.Drone 2.

This package is meant to act as the "glue binder" for a flight stack consisting of
"off-the-shelf" ROS packages. It is designed for use with the Parrot AR.Drone 2.

As of the time of writing, the code in this repository has been used to achieve
successful odometric state estimation in real-world test flights.

[![localization-demo-youtube](https://img.youtube.com/vi/MC6KFNkSnvc/maxresdefault.jpg)](https://youtu.be/MC6KFNkSnvc)

**This README is a work-in-progress, and the information to follow may be
outdated or incomplete.**

Dependencies
--------------------------------------------------------------------------------
As of the time of writing, this package uses the following ROS packages,
many all of which have been forked from the original repositories and
modified for improved interoperability.

- [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/)
  - This is a ROS interface for the Parrot AR.Drone 2, enabling two-way
    communication between an AR.Drone and a connected computer.
  - Communication consists of velocity setpoints _from_ and camera images/sensor
    data _to_ the user's machine.
  - `ardrone_autonomy` broadcasts IMU, height SONAR, magnetometer, front camera,
    bottom camera, and visual odometry data extracted from the AR.Drone's
    incoming datastream.
  - This package can be installed through `apt` (`ros-kinetic-ardrone-autonomy`).
- [ardrone_joystick](https://github.com/acpopescu/ardrone_joystick)
  - Enables remote control of an AR.Drone with an XBOX 360 controller through
    ROS.
  - The original package did the same, but with a Playstation DualShock
    3 instead of an XBOX controller. This fork is used due to the wider
    availability of XBOX 360 controllers, and especially for its compatibility
    with generic models of the same.
- [robot_localization](http://docs.ros.org/kinetic/api/robot_localization/html/index.html)
  - Generic implementation of a 15-DoF extended Kalman filter _and_ an
    unscented Kalman filter, [written and maintained](https://github.com/cra-ros-pkg/robot_localization)
    by a [computer intelligence/data analytics firm](https://www.cra.com/) based in Cambridge,
    Massachusetts. (Only EKF is used in this package, as of the time of writing.)
  - This package is built to support any kind of robot (e.g. wheeled ground
    robots _and_ aerial drones) and can fuse data from an arbitrary number of
    sensors, so long as the data is provided in one of several ROS-standardized
    message formats (e.g. `nav_msgs::Odometry`, `geometry_msgs::PoseWithCovariance`, etc.)
  - This package can be installed through `apt` (`ros-kinetic-robot-localization`).
- [tum_ardrone](https://github.com/Yilin-Yang/tum_ardrone)
  - Autonomous flight, visual SLAM, and EKF-based state estimation built for the
    Parrot AR.Drone family of drones.
  - The repository from which this repo was forked was, itself, a fork of the
    [original repository from the Technical University of Munich,](http://wiki.ros.org/tum_ardrone)
    modified to be compatible with ROS Kinetic Kame.
  - The linked version of the repository has been modified to fix a few
    obscure compilation errors.
  - As of now, this package is mainly being used for "proof-of-concept" testing,
    e.g. for fusing its EKF output into the EKF provided by
    `robot_localization`, which is a thing that you can totally do.
- [tum_simulator](https://github.com/Yilin-Yang/tum_simulator)
  - Full simulation of a Parrot AR.Drone 2 inside of Gazebo.
  - The repository from which this repo was forked was, itself, a fork of the
    [original repository from the Technical University of Munich,](http://wiki.ros.org/tum_simulator)
    modified to be compatible with ROS Kinetic Kame.
  - No substantial changes have been made to this package as of the time of
    writing, aside from minor changes to clean up build output and fix
    compiler errors.

Installation
--------------------------------------------------------------------------------
This package is built for use with ROS Kinetic Kame on a system running Ubuntu
16.04 LTS. This repository is, in large part, devoted to wrangling together
a disparate collection of ROS packages to run simultaneously and work with each
other; it will probably fail to compile in another ROS distribution, at
least without substantial legwork.

### Installing apt Dependencies

```bash
# add ROS apt repository for Ubuntu 16.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt update

# generic ROS environment
sudo apt install \
            ros-kinetic-desktop-full \
            ros-kinetic-catkin \
            ros-kinetic-catkin-pip \
            ros-kinetic-rqt-robot-plugins \
            python-catkin-tools \
            tf2-tools \
            python-rosmsg

# various first- and second-order dependencies
sudo apt install \
            ros-kinetic-ardrone-autonomy \
            ros-kinetic-joystick-drivers \
            ros-kinetic-joy \
            daemontools \
            libudev-dev \
            ros-kinetic-robot-localization \
            libiw-dev
```

### Building catkin Dependencies

**TODO**

Layout and Organization
--------------------------------------------------------------------------------

### Folder Structure

- `include` - C++ header for the source files in `src`.
- `launch` - `roslaunch`-compatible `*.launch` [XML files.](http://wiki.ros.org/roslaunch/XML)
- `params` - YAML-configuration files to be [loaded](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects#Parameters.2C_namespaces.2C_and_yaml_files)
  in launchfiles.
  - Notably includes configuration files for `robot_localization`.

### Nodes

This package provides the following nodes:
- `message_to_csv`
  - Reads in ROS messages (of a few specific types, see `message_printer.h`) and
    either logs them to an output file, or prints them to `stderr`.
  - Supports printing in CSV, or human-readable printing optimized for "live"
  viewing in a terminal window.
- `um_rebroadcast`
  - Reads in ROS messages (mainly sensor outputs provided by
    `ardrone_autonomy`), modifies them, and rebroadcasts them.
  - "Modification" consists mainly of:
    - Setting [tf coordinate frames](http://wiki.ros.org/tf2) for [REP-105 compliance.](http://www.ros.org/reps/rep-0105.html)
    - Setting appropriate covariance matrices (see `./params/rebroadcast.yaml`).
    - Converting between message types (e.g. `ardrone_autonomy::Navdata ->
      geometry_msgs::PoseWithCovariance`)
    - Modifying sensor messages as needed, either to comply with [REP-103,](http://www.ros.org/reps/rep-0103.html)
      or to work more nicely with [`robot_localization`.](http://docs.ros.org/kinetic/api/robot_localization/html/preparing_sensor_data.html)

### tf Tree

#### REP-105 Compliance
[REP-105](http://www.ros.org/reps/rep-0105.html) mandates the use of the
following coordinate frames: `map`, `odom`, and `base_link`. (In practice,
these frames will often have different names, but we'll use these names in this
document.)

- `base_link` is the "local frame": it is the coordinate frame of the actual
robot itself, rigidly fixed to the robot's chassis.

- `odom` is a "smoothed world map": it is a (nominally) world-fixed frame that
is subject to drift over time.
  - The pose of `base_link` in the `odom` frame is obtained from
  "continuous" odometric sensors (e.g. wheel odometers, visual odometry) and/or
  dead reckoning (e.g. double-integration of accelerations from an IMU).
  - `odom` is a "working coordinate frame" most useful _in the short term._
  Over time, the `odom` frame will drift with respect to the `map` frame due to
  the accumulation of measurement error over time.
  - `odom` is useful because it is "smooth": the pose of `base_link` in the
  `odom` frame will never abruptly "jump" due to the detection of a landmark, a
  GPS position, etc.

- `map` is a "definitive world map": it is a world-fixed frame that is _not_
subject to drift over time.
  - "Absolute" positioning (e.g. through GPS, detection of visual landmarks)
  _in addition to_ the odometrics and dead-reckoning used by `odom` is used to
  determine the _most accurate_ pose of `base_link` in the `map` frame. The
  difference between this `map -> base_link` transformation and the current
  `odom -> base_link` transformation is used to determine the `map -> odom`
  transformation.
  - `map` is useful as a long-term reference for position and orientation, but
    its tendency to "jump" makes it unreliable for tasks requiring precision.

#### tf Tree Structure
`um_ardrone` uses the tf tree given below. Transformations to frames marked with
a `?` are not currently used or broadcast, but may be in the future.

```
                                ----------->? um_kinect
                                |---------->? um_camera_bottom
                                |---------->? um_camera_front
                                |
um_map ----> um_odom ----> um_base_link ----> um_imu
                                |
                                ----------->? um_sonar
```

As one would expect: `um_map` is the `map` frame; `um_odom` is the `odom` frame;
and `um_base_link` is the `base_link` frame.

We specify a separate `um_imu` transformation because our test vehicle's IMU
is very slightly off-level when the vehicle is at rest. Without this additional
transformation, the off-level acceleration vector reported by the vehicle's IMU
may (in the absence of other sensor data, especially visual odometry) fool
`robot_localization` into thinking that the vehicle is drifting sideways and
through the floor.

Style
--------------------------------------------------------------------------------
This package aims to be complaint with the [ROS C++ Style Guide.](http://wiki.ros.org/CppStyleGuide)

Notable alterations mainly include the use of an 80-character line limit instead
of ROS's suggested 120-character line limit.

Common Issues
--------------------------------------------------------------------------------
This list is incomplete and may grow over time.

> Gazebo loads a black screen when I try to start the simulation!

This often happens when a previous Gazebo instance did not exit properly. (When
closing a simulator instance, hit Ctrl-C, and then _wait_ for the process to die
gracefully. This may take several seconds.) Use the Ubuntu System Monitor (or
`ps aux | grep gazebo` in conjunction with `kill -s 9 <PID_OF_GAZEBO>`) to kill any
lingering Gazebo processes.

> `roscore` complains about an existing ROS master, even though I've killed the
> old one.

Try killing/restarting `roscore`. Find the process ID of `rosmaster` with `ps
aux | grep rosmaster`, then run `kill -s 9 <PID>` where `<PID>` is the leftmost
number in the output from `ps`.
