Start of a project to rotate an RPlidar A1M8 and combine point cloud rows into a 3d point cloud for obstacle avoidance and mapping

This project uses a servo and MPU6050 IMU on a Raspberry PI to tilt and detect the pitch of the lidar to perform the rotations.

The pitch data is transimted to the laser_joint_publisher via UDP which publishes the data as a ROS joint state.

The lidar serial connection is bridged over network via usbip (https://usbip.sourceforge.net/) and data transfered over network, but as if it were the serial connection locally to ROS.

Shout out to Articulated Robotics for the template

![Cloud Scan](https://github.com/pbybee/ros2-tilt-laser/blob/master/images/20241027_103124.jpg?raw=true)
