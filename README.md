# ROS Modules for Canterbury Air Patrol

These are the ROS Modules we run on our UAVs so they can participate in searches.

Do not blindly use these for any purpose, you will need to do your own testing to decide if they are useful in your specific situation.

# Building
You will need to have [ROS installed](http://wiki.ros.org/melodic/Installation) already.

Also, smm_interface uses [smm-asset-api](https://github.com/canterbury-air-patrol/smm-asset-api) to talk to the search map.

```
git clone https://github.com/canterbury-air-patrol/ros-components.git
cd ros-components
source /opt/ros/melodic/setup.bash
catkin_make
```
