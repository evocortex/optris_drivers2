# ROS2 wrapper for irDirectSDK

## Installation
Nodes have been tested with the version Dashing Diademata (short form: Dashing)

Reference: https://index.ros.org/doc/ros2/Installation/


## Environment setup
```
$ source /opt/ros/dashing/setup.bash
$ source install/setup.bash
```

## Services
Capitalized service names are no longer valid. Use underscores to define parameters.

## Parameters
Passing ROS command line parameters is not supported in Dashing. The version eloquent will introduce this features.
Thus, a plain command line parameter has been used for this version. argv[1] must be the path to a valid xml configuration file.

## Build
```
$ colcon build --symlink-install
```

## Execute
... device driver node
```
$ ros2 run optris_drivers2 optris_imager_node <xml_config_path>
```

... color conversion node (converts temperature images to false color images)
```
$ ros2 run optris_drivers2 optris_colorconvert_node
```

... displaying images
```
$ ros2 run image_tools showimage -t /thermal_image_view
```

... switch palette for color conversion
```
$ ros2 service call palette optris_drivers2/srv/Palette "{palette : 3}"
```

## Image compression
The node optris_colorconvert_node can publish compressed images via image_transport. Be sure to have installed image-transport-plugins:
```
$ sudo apt install ros-dashing-image-transport-plugins
```

Bandwidth measurement can be done either raw:
```
$ ros2 topic bw /thermal_image_view
```

or compressed:
```
$ ros2 topic bw /thermal_image_view/compressed
```

## Installing IR Imager Direct SDK

In order to successfully compile this package, the IR Imager Direct SDK for Optris cameras is needed.
You can download it from http://ftp.evocortex.com/


If you want to install the latest available version using rosdep, add a rosdep source by executing

``sudo sh -c 'echo "yaml https://raw.githubusercontent.com/evocortex/optris_drivers2/master/libirimager.yaml " > /etc/ros/rosdep/sources.list.d/19-libirimager.list'``

in a Linux terminal. For further information regarding rosdep, see http://wiki.ros.org/rosdep
