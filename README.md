# ROS2 wrapper for irDirectSDK

## Installation
Nodes have been tested with the version Dashing Diademata (short form: Dashing)
Reference: https://index.ros.org/doc/ros2/Installation/


## Environment setup
$ source /opt/ros/dashing/setup.bash
$ source install/setup.bash

## Services
Capitalized service names are no longer valid. Use underscores to define parameters.

## Parameters
Passing ROS command line parameters is not supported in Dashing. The version eloquent will introduce this features.
Thus, a plain command line parameter has been used for this version. argv[1] must be the path to a valid xml configuration file.

## Build
$ colcon build --symlink-install

## Execute
... device driver node
$ ros2 run optris_drivers2 optris_imager_node <xml_config_path>

... color conversion node (converts temperature images to false color images)
$ ros2 run optris_drivers2 optris_colorconvert_node

... displaying images
$ ros2 run image_tools showimage -t /thermal_image_view