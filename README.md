# Dual_sim
Dual_arm simlator (SLAM &amp; Navi)

## Installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dual_sim.git
cd ..
catkin_make
```

## Dependent pkg
```bash
cd catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
cd ..
catkin_make
```

## Usage

### Running an environment
Bringup
```bash
roslaunch bringup_dual robot.launch
```

Slam
```bash
roslaunch slam_dual slam_gmapping.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dual_sim/navigation_dual/maps/map
```

Navi
```bash
roslaunch navigation_dual navigation.launch
```