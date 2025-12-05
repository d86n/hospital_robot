## 0. Yêu cầu cài đặt

Chạy lệnh sau:

```
sudo apt-get update
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 1a. Quét map
```
roslaunch hospital_robot gazebo.launch
roslaunch hospital_robot gmapping.launch
roslaunch hospital_robot display.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## 1b. Lưu map
```
rosrun map_server map_saver -f ~/catkin_ws/src/hospital_robot/maps/hospital_map
```

## 2. Tự chạy
```
roslaunch hospital_robot gazebo.launch
roslaunch hospital_robot navigation.launch
roslaunch hospital_robot display.launch
```