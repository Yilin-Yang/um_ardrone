um_ardrone
================================================================================
Experimental autonomous flight with a Parrot AR.Drone 2.

This package is meant to act as the "glue binder" for a flight stack consisting of
"off-the-shelf" ROS packages, tailored for use with the Parrot AR.Drone 2.

**This README is a work-in-progress, and the information to follow may be
outdated or incomplete.**

Dependencies
--------------------------------------------------------------------------------
As of the time of writing, this package uses the following ROS packages,
practically all of which have been forked from the original repositories and
modified for improved interoperability.

- [ardrone_joystick](https://github.com/acpopescu/ardrone_joystick)
  - Enables remote control of an AR.Drone with an XBOX 360 controller through
    ROS.
  - The original package did the same, but with a Playstation DualShock
    3 instead of an XBOX controller. This fork is used due to the wider
    availability of XBOX 360 controllers, and especially for its compatibility
    with generic models of the same.
- [hector_localization](https://github.com/Yilin-Yang/hector_localization)
  - 6-DoF extended Kalman filter, primarily designed for determining the pose of
    an aerial drone.
  - No substantial changes have been made to this package as of the time of
    writing, aside from minor changes to clean up build output and fix
    compiler errors.
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
other; it will almost certainly fail to compile in another ROS distribution, at
least without substantial legwork.

**TODO**
