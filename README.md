## Yêu cầu cài đặt

Chạy lệnh sau:

```
sudo apt-get update
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Quét map
```
roslaunch hospital_robot gazebo.launch
roslaunch hospital_robot gmapping.launch
roslaunch hospital_robot display.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```