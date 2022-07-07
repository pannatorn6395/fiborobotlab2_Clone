cd ~/dev_ws/src
git clone -b ros2 https://github.com/ros-perception/vision_opencv.git
cd ~/dev_ws
rosdep update
rosdep install -i --from-path src/vision_opencv --rosdistro foxy -y
colcon build
