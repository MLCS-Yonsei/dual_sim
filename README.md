# Dual_sim
Dual_arm simlator (SLAM &amp; Navi)

## Installation
One-line installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dual_sim.git -b hsc
cd dual_sim
sh setup.sh
```

## Usage

### Running on gazebo simulator
Bringup
```bash
roslaunch bringup_dual dual_sim.launch
```

```
Slam & Navi
```bash
roslaunch slam_navi slam_navi.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dual_sim/slam_navi/maps/map
```

### Running on real-robot
Bringup
```bash
roslaunch bringup_dual dual_real.launch
```

```
Slam & Navi
```bash
roslaunch slam_navi slam_navi.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dual_sim/slam_navi/maps/map
```