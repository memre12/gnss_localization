# Dual Antenna GNSS Localization - ROS 2 Setup

This guide outlines the steps to run the dual antenna GNSS localization system using ROS 2 and Autoware.

---

## Prerequisites

Make sure you have sourced the Autoware workspace:

## Build GNSS-Localization workspace

colcon build --symlink-install

## Play ros2 bag 
 ros2 bag play notf_nobaddata_drive --start-offset 70

## Run python3 scripts

python3 retimer.py 
python3 twist_to_twist_cov.py 

## Launch Localization Packages

ros2 launch gae_launch dual_antenna_gnss_loc.launch.py 
