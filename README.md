# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a FastDDS implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. Plug in the VESC with a USB cable.
3. Add a JSON config file for vesc with the following properties [example here](../configs/vescConfig.json):
-   "port" [string]
-   "brake_max" [double]
-   "brake_min" [double]
-   "current_max" [double]
-   "current_min" [double]
-   "duty_cycle_max" [double]
-   "duty_cycle_min" [double]
-   "position_max" [double]
-   "position_min" [double]
-   "servo_max" [double]
-   "servo_min" [double]
-   "speed_max" [double]
-   "speed_min" [double]
-   "tire_radius" [double]
-   "sphere_radius" [double]

4. Build the sub-project VESC_Motor_Controller 
5. Launch the created executable through command line and also provide the central_config json file path (in the configs folder)
6. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
