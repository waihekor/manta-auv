# Sonar_mapping

## Dependencies
- OpenCV
- Numpy
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)

## Required input
- sensor_msgs/LaserScan.msg
- sensor_msgs/Image.msg
- nav_msgs/Odometry.msg

## Global mapping
Change to desired parameters and topics in config/config.yaml

Parameter explanation:
- LOCAL_MAP = True, a map of the current sonar scan will be displayed on the screen
- GLOBAL_MAP = True, a map of the global map will be displayed on the screen
- PUBLISH_GLOBAL_MAP = True, an OccupancyGrid message will be published on the rostopic /map  

Then:
```
roslaunch sonar_mapping global_mapping.launch
```

## Object placement 
darknet_ros must be built and in the same workspace as sonar_mapping

Then:
```
roslaunch sonar_mapping global_mapping_and_darknet.launch
```

Objects will be published in vortex_msgs/ObjectPlacement.msg format to /manta/object_position

**NB!** To add or remove objects the ObjectPlacement.msg file must be manually changed

