# Dual Antenna GNSS Localization - ROS 2 Setup

This guide outlines the steps to run the dual antenna GNSS localization system.

---

## Prerequisites

Make sure you have sourced the Autoware workspace:

## Build GNSS-Localization workspace

colcon build --symlink-install

## Play ros2 bag 
 ros2 bag play notf_nobaddata_drive

## Run python3 scripts

python3 gae_launch/scripts/retimer.py 


## Launch Localization Packages

ros2 launch gae_launch dual_antenna_gnss_loc.launch.py 
