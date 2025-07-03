# Sensor Fusion EKF Localizer

## Overview
Integrate IMU, GNSS, and vehicle speed into an Extended Kalman Filter (EKF) to estimate the vehicle's global pose (position and orientation) with covariance. 

## Sensor Topics
- **IMU Data** (`sensor_msgs/Imu`)
- **GNSS Position Data (1)** (`sensor_msgs/NavSatFix`)
- **GNSS Position Data (2)** (`sensor_msgs/NavSatFix`)
- **Twist Data** (`geometry_msgs/Twist`)
  - Source: IMU or motor driver (Motor Driver's consistency should be taken into account. IMU is good option.)
  - Note: No covariance—attach a static covariance matrix via Python before EKF.

---

## GNSS Poser
Convert dual GNSS readings and MGRS grid zone (from Lanelet2) into a 2D pose with covariance.

**Inputs**
- `/gnss1/fix` (`NavSatFix`)
- `/gnss2/fix` (`NavSatFix`)
- `/lanelet/mgrs_zone` (`MapProjectorInfo`) # Just launch map.launch.py

**Output**
- `/pose_with_covariance` (`PoseWithCovarianceStamped`)


---

## gyro_odometer
Estimate vehicle twist by fusing IMU angular rates with measured speed.

**Inputs**
- `/vehicle/twist_with_covariance` (`TwistWithCovarianceStamped`)
- `/imu` (`Imu`)

**Output**
- `/twist_with_covariance` (`TwistWithCovarianceStamped`)


---

## ekf_localizer
Fuse pose and twist into a global pose estimate using EKF.

**Inputs**
- `/pose_with_covariance` (`PoseWithCovarianceStamped`)
- `/twist_with_covariance` (`TwistWithCovarianceStamped`)

**Output**
- `/ekf/pose` (`PoseWithCovarianceStamped`)


## Visuals

![RVIZ visualization of EKF output](visual.gif)


## Why the GNSS-Only Pose Lags

In the GIF, the GNSS-only pose (the trailing marker) falls behind the EKF output because GNSS fixes arrive at **5 Hz** (once every **0.2 s**) while the EKF integrates sensor data continuously.

At a constant speed of **3.6 m/s** (≈ 12.96 km/h), the vehicle covers:
**Distance** = **Speed** × **Interval**  
= 3.6 m/s × 0.2 s  
= 0.72 m


between GNSS updates. Consequently, each new GNSS position is about **0.72 m** behind the vehicle’s actual location, whereas the EKF estimate remains current.

## Build
Remember to source Autoware Package and:
```python
colcon build --symlink-install
```


## Launch

```python
ros2 launch gae_launch dual_antenna_gnss_loc.launch.py
```

## Test with bags


```python
source install/setup.bash
ros2 launch gae_launch dual_antenna_gnss_loc.launch.py 
ros2 bag play notf_nobaddata_drive --start-offset 100
python3 gnss_localization/gae_launch/scripts/retimer.py
```