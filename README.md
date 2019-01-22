# Dual_sim
ROS gazebo simlator for mecanum wheeled mobile platform

## Dependency
```bash
sudo apt-get install ros-kinetic-slam-gmapping
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt-get install ros-kinetic-joint-state-publisher
```
Dependency repository:
https://github.com/MLCS-Yonsei/teb_local_planner.git

## Installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dual_sim.git
cd ..
catkin_make
source ~/.bashrc && source ~/catkin_ws/devel/setup.bash
```

## Usage

### Running an environment
Bringup
```bash
roslaunch bringup_dual robot.launch
```

Slam
```bash
roslaunch slam_navi slam.launch
```

Navi
```bash
roslaunch slam_navi navi.launch
```

Teleop
```bash
roslaunch teleop teleop_key.launch
```

Auto slam
```bash
roslaunch slam_navi auto_slam.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dual_sim/slam_navi/maps/map
```

Goal pub
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseSmped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.1, y: 7.9, z: 0.0}, orientation: {z: -0.1, w: 1.0}}}'
```
