# ROS 2 Mobile Robot: SLAM, Localization, and A* Planning

Complete ROS 2 workspace for a differential-drive robot with simulation, robot modeling, EKF state estimation, particle-filter localization, custom 2D SLAM, map publishing, and A* path planning.

## Packages

- `robot_description`: URDF, meshes, Gazebo world, RViz, sensor bridges, robot-state publisher.
- `robot_localization`: EKF, particle filter, map publisher, TF conversion, motor command, and A* planner.
- `robot_slam`: C++ occupancy-grid SLAM, scan matching, motion model, and map saving.

## Requirements

Ubuntu with ROS 2 Humble or compatible, Gazebo Sim, RViz2, Python 3, C++17, Colcon, Eigen3, `ros_gz_sim`, `ros_gz_bridge`, `tf2_ros`, `nav_msgs`, `sensor_msgs`, and `geometry_msgs`.

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep libeigen3-dev
sudo apt install -y ros-<ros_distro>-rviz2 ros-<ros_distro>-xacro ros-<ros_distro>-robot-state-publisher
sudo apt install -y ros-<ros_distro>-joint-state-publisher ros-<ros_distro>-joint-state-publisher-gui
sudo apt install -y ros-<ros_distro>-tf2-ros ros-<ros_distro>-tf2-geometry-msgs
sudo apt install -y ros-<ros_distro>-ros-gz-sim ros-<ros_distro>-ros-gz-bridge
```

Replace `<ros_distro>` with your installed distribution.

## Clone and build

```bash
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws/src
git clone https://github.com/Narjes-Gh2024/HW03_Robotic_Technology.git
cd ~/robotics_ws
source /opt/ros/<ros_distro>/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch commands

```bash
ros2 launch robot_description display.launch.py
ros2 launch robot_description gazebo.launch.py
ros2 launch robot_localization localization.launch.py
ros2 launch robot_slam slam.launch.py
```

SLAM parameters can be overridden:

```bash
ros2 launch robot_slam slam.launch.py odom_topic:=/ekf/odom scan_topic:=/scan map_frame:=map odom_frame:=odom base_frame:=base_link map_resolution:=0.05
```

## Individual executables

```bash
ros2 run robot_localization map_pub_node
ros2 run robot_localization ekf_node
ros2 run robot_localization particlefilter_node
ros2 run robot_localization astar_node
ros2 run robot_localization frame_id_converter
ros2 run robot_localization motor_command_node
ros2 run robot_slam slam_node
ros2 run robot_slam map_saver_node
```

## Interfaces

Typical topics are `/scan`, `/imu`, `/odom`, `/ekf/odom`, `/map`, `/cmd_vel`, and `/robot_description`. The principal TF chain is `map -> odom -> base_link`.

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /map --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
```

## Algorithms

- **EKF:** motion prediction followed by sensor measurement correction.
- **Particle filter:** weighted pose hypotheses, sensor weighting, and resampling.
- **A*:** occupancy-grid planning with `f(n) = g(n) + h(n)`, collision checking, and `nav_msgs/Path` output.
- **SLAM:** motion model, laser scan matching, occupancy-grid update, TF handling, and map saving.

## Configuration

- `robot_description/config/gz_bridge.yaml`: Gazebo/ROS bridges.
- `robot_description/config/params.yaml`: simulation parameters.
- `robot_localization/config/particle_filter_params.yaml`: particle-filter parameters.
- `robot_localization/maps/my_map.yaml`: map metadata.
- `robot_slam/config/slam_params.yaml`: SLAM parameters.
- RViz configurations are stored in the package `rviz/` and `config/` directories.

## Troubleshooting

Source ROS and the workspace in every terminal. If a package is not found, rebuild with `colcon build --symlink-install`. If scan or map data is missing, inspect the bridge and topic names with `ros2 topic list`. If TF fails, run `view_frames` and verify `map`, `odom`, and `base_link`.

## Author

**Narjes Ghazanfari** — B.Sc. Aerospace Engineering, Sharif University of Technology.
