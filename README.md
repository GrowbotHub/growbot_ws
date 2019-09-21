# GrowbotHub
ROS Workspace for GrowBotHub

## Getting started
- Clone repository and submodules: `git clone --recursive https://github.com/GrowbotHub/growbot_ws.git`
- Install ROS: http://wiki.ros.org/melodic/Installation/Ubuntu
- Source ROS: `source /opt/ros/melodic/setup.bash`
- Build packages: `catkin_make`
- Source the package: `source devel/setup.bash`

## Development
Quick guidelines for future development

### Adding ROS packages
```
git submodule add -b [branch] https://github.com/GrowbotHub/[package_name].git src/[package_name]
```

## RQT Graph
![rqt_grap](assets/rosgraph.png)