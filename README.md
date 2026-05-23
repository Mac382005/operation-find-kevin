# Operation Find Kevin - Succulence Rover

This repository contains the ROS 2 package used for the Algorithmic Robotics Grand Challenge, **Operation Find Kevin**. The system integrates pose-graph SLAM, A* path planning, and path-following navigation for the Succulence rover in the Unity Mars simulation.

## Project Overview

The final system connects mapping, planning, and navigation into one complete ROS 2 pipeline.

Main ROS 2 nodes used in the final run:

- `/slam_node` - builds the SLAM map and corrected trajectory
- `/planner_node` - runs A* path planning on the SLAM occupancy grid
- `/navigator_node` - follows the planned path and sends velocity commands
- `/map_to_odom_publisher` - publishes the static transform between map and odometry
- `/base_to_lidar_publisher` - publishes the static transform between robot base and LiDAR

Main topics used:

| Topic | Message Type | Purpose |
|---|---|---|
| `/succulence/odom` | `nav_msgs/msg/Odometry` | Raw odometry from the rover |
| `/succulence/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan data |
| `/succulence/map` | `nav_msgs/msg/OccupancyGrid` | SLAM occupancy grid map |
| `/succulence/slam/odometry` | `nav_msgs/msg/Odometry` | Corrected SLAM odometry |
| `/succulence/slam/path` | `nav_msgs/msg/Path` | SLAM trajectory for RViz2 visualisation |
| `/succulence/plan` | `nav_msgs/msg/Path` | Planned A* path |
| `/succulence/plan/inflated` | `nav_msgs/msg/OccupancyGrid` | Inflated planning grid for obstacle safety |
| `/succulence/plan/reachable` | `nav_msgs/msg/OccupancyGrid` | Reachable planning grid |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command sent to the rover |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transform information |

## System Pipeline

The Unity Mars simulation publishes raw odometry and LiDAR scan data. The `/slam_node` uses these inputs to build a pose-graph SLAM map and publish corrected odometry. The `/planner_node` uses the SLAM map and corrected pose to generate an A* path. The `/navigator_node` follows the planned path and publishes `/cmd_vel` commands back to the simulated rover.

Basic flow:

```text
Unity Mars Simulation
    ↓ /succulence/odom, /succulence/scan

/slam_node
    ↓ /succulence/map, /succulence/slam/odometry

/planner_node
    ↓ /succulence/plan

/navigator_node
    ↓ /cmd_vel

Unity Mars Simulation
```

## Build Instructions

Clone this package into the ROS 2 workspace:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws/src
git clone https://github.com/Mac382005/operation-find-kevin
cd ..
colcon build --packages-select succulence_rover_ros --symlink-install
source install/setup.bash
```

If the package already exists in the workspace, rebuild it with:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
colcon build --packages-select succulence_rover_ros --symlink-install
source install/setup.bash
```

## Launch Instructions

Start the Unity Mars simulation in the first terminal:

```bash
cd ~/algorithmic-robots-world
docker compose -f compose-simulation.yaml up
```

Launch the full SLAM, planning, and navigation pipeline in a second terminal:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
source install/setup.bash
ros2 launch succulence_rover_ros mission_sim.launch.py
```

Open RViz2 in a third terminal:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
source install/setup.bash
rviz2 -d src/succulence_rover_ros/config/succulance_slam.rviz
```

## Useful Debug Commands

List active ROS 2 nodes:

```bash
ros2 node list
```

List active topics with message types:

```bash
ros2 topic list -t
```

Check planner output:

```bash
ros2 topic echo /succulence/plan --once
```

Check velocity commands:

```bash
ros2 topic echo /cmd_vel
```
## Team Members: Group 3

- Hoang Nhat Minh Mac
- Nima
- Bishaw
- Joe
- 
