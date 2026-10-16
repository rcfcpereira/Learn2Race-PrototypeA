# Prototype A – ROS 2 Jazzy workspace

The packages the Raspberry Pi 4 runs. Installation: [../INSTALL.md](../INSTALL.md). Use: [../USAGE.md](../USAGE.md).

```bash
cd ~/learn2race_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch learn2race_bringup prototype_a.launch.py competition:=false teleop:=true rgb_stick:=true
```

In normal use the robot is started with `~/robot.sh`, which launches `prototype_a.launch.py` and the camera line follower with the right arguments.

The micro-ROS agent (`micro-ROS-Agent`, `micro_ros_msgs`) and the lidar driver (`ldlidar_stl_ros2`) are cloned into `src/` during installation; they are not part of this repository.

Each package has its own `README.md`. References cited as `[Rnn]` in the file headers are listed in [`docs/references.md`](../docs/references.md).
