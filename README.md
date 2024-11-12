# Difference usage of this branch

If you failed build at openzen-external-spdlog follow: ex) ROS2 humble

build and make install the openzen first, and colcon build openzenros2 later.

If you command source install/setup.bash, openzen path will be automatically added to LD_LIBRARY_PATH
```
mkdir -p ros_ws/src
cd ros_ws/src

git clone --recurse-submodules https://github.com/Deep-In-Sight/OpenZenRos.git

cd OpenZenRos/openzen && mkdir build && cd build
cmake .. && make -j4 && sudo make install
```
## Save Covariance Matrix and Publish Data
Save 3x3 covariance matrix to below file

```
/ros_ws/covariance_matrix.txt
```
each covariance is 3x3 table and it is publishing through ROS2 sensor_msgs/msg/imu_Message
| | covariance_matrix | |
|---------------|---------------|---------------|
| orientation_covariance[0] | orientation_covariance[4]  | orientation_covariance[8]  |
| angular_velocity_covariance[0]  | angular_velocity_covariance[4]  | angular_velocity_covariance[8]  |
| linear_acceleration_covariance[0]  | linear_acceleration_covariance[4]  | linear_acceleration_covariance[8]  |


# OpenZen Node for ROS 2

This software allows to forward sensor data from sensor connected via OpenZen to ROS version 2.

OpenZen is a library for high performance sensor data streaming and processing and supports multiple sensor models: <https://bitbucket.org/lpresearch/openzen/>

## Requirements

### Serial Port Access Rights

After this call, you should logout and login with this user to ensure the changed permissions are in effect.

To allow access to sensors connected via USB, you need to ensure that the user running the ROS sensor node
has access to the /dev/ttyUSB devices. You can do this by adding the user to the dialout group.

```
sudo adduser <username> dialout
```

## Compilation

To compile this driver in your ROS2 Foxy setup, follow these steps:
```
mkdir -p ros_ws/src
cd ros_ws/src

git clone --recurse-submodules https://bitbucket.org/lpresearch/openzenros2.git

# get your ROS2 environment going
source /opt/ros/foxy/setup.bash
cd ..
colcon build
source ./install/setup.bash
```


## Running the Driver

You can now run the OpenZen ROS2 driver with this command in the window
you used to compile the software:

```
ros2 run openzen_driver openzen_node --ros-args --remap __ns:=/openzen
```

By default, it will connect to the first available sensor. If you want to connect to
a specific sensor, you can use the serial name of the sensor as parameter, for example:

```
ros2 run openzen_driver openzen_node --ros-args --remap __ns:=/openzen -p sensor_name:="LPMSCU2000573"
```

If your sensor is configured for a different baud rate, you can use the baudrate parameter to
give a specfic baud rate setting:

```
ros2 run openzen_driver openzen_node --ros-args --remap __ns:=/openzen -p sensor_name:="LPMSCU2000573" -p baudrate:=115200
```

Now you can print the IMU values from ROS with:

```
ros2 topic echo /openzen/data
```

To output the values of a GPS unit (if available) use this command:

```
ros2 topic echo /openzen/nav
```

Or plot some values (for example linear acceleration) with 

```
ros2 run rqt_plot rqt_plot /openzen/data/angular_velocity
```

We have prepared a sample launch file openzen_lpms.launch to demonstrate data acquisition and plotting using openzen_sensor_node:
```
ros2 launch openzen_driver openzen_lpms.launch.py
```

To change to autocalibration setting via the command line:

```
ros2 service call /enable_gyro_autocalibration std_srvs/SetBool "{data: true}"
```

To trigger an gyroscope calibration:

```
ros2 service call /calibrate_gyroscope std_srvs/Trigger
```
