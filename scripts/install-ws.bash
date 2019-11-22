# ROS dependencies from apt
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential  cmake
sudo rosdep init
rosdep update

# Add internal and external dependencies
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init -j3 ./src melodic-ros_comm-wet.rosinstall
rosdep install -y --from-paths ./src --ignore-src --rosdistro melodic -r --os=debian:buster

# Add non-published packages
git clone --depth=1 https://github.com/RobotWebTools/rosbridge_suite.git src/rosbridge_suite

# Configure catkin
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j1

# Install WS specific packages
pip install rospkg smbus schedule gpiozero flask flask-cors picamera
