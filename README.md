# Dual_sim
ROS gazebo simlator for mecanum wheeled mobile platform

## Installation
Dependency
```bash
sudo apt-get install ros-kinetic-robot-gmapping
sudo apt-get install ros-kinetic-robot-amcl
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt-get install ros-kinetic-joint-state-publisher
sudo apt-get install ros-kinetic-teb-local-planner
```
Installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dual_sim.git
cd ..
catkin_make
source ~/.bashrc && source ~/catkin_ws/devel/setup.bash
```
Gazebo model import
```bash
Gazebo mopdel copy
roscd bringup_dual/models
sudo cp -r 'test1_origin_4m' 'test2_origin_4m' 'test3_origin_4m' '/usr/share/gazebo-8/models'
```

## Usage

### Running an environment
Bringup
```bash
roslaunch bringup_dual robot.launch
```

Slam (static environment)
```bash
roslaunch slam_navi slam_st.launch
```

Navi (static environment)
```bash
roslaunch slam_navi navi_st.launch
```

Slam (dynamic environment)
```bash
roslaunch slam_navi slam_dy.launch
```

Navi (dynamic environment)
```bash
roslaunch slam_navi navi_dy.launch
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
