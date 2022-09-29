# Synchronized image saver
This package uses the ROS TimeSynchronizer message_filter to record synchronized image messages from two cameras.   

## System requirements
This node has been tested on Ubuntu 20.04 and ROS2 Foxy. However, it should work on all ROS2 distributions.   

## Installation
Clone this repo in the src folder of your ROS2 workspace, source your ROS2 environment and build the package.
```bash
# in your ros_ws root
cd src
git clone https://github.com/giuschio/ros_synchronized_image_saver
cd ..
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```
## Usage
After sourcing the appropriate ROS environment, run the node with the following:   
```bash
ros2 run synchronized_image_saver node --ros-args -p output_path:=<ABSOLUTE_OUTPUT_PATH> -r image0:=<first_img_topic> -r image1:=<second_image_topic>  
```