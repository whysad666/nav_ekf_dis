依赖
```bash
sudo apt install build-essential
sudo apt install libboost-dev
sudo apt install libeigen3-dev
sudo apt install libopencv-dev
```

```bash
. install/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-up-to kfnav_gi module_slam module_zh sensor_gnss nav_record
```

导出ROSBAG至CSV
```bash
# term 1
ros2 bag play -r 倍速 -p 源文件名
# term 2
ros2 topic echo 话题名 --csv >导出文件名
```
