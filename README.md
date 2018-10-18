# Dual_sim
Dual_arm simlator (SLAM &amp; Navi)

## Installation
One-line installation
```bash
cd catkin_ws/src
git clone https://github.com/MLCS-Yonsei/dual_sim.git
cd dual_sim
sh setup.sh
```

## Usage

### Running an environment
Bringup
```bash
roslaunch bringup_dual robot.launch
```

```
Slam & Navi
```bash
roslaunch slam_navi slam_navi.launch
```

Map saver
```bash
rosrun map_server map_saver -f /home/seungchul/catkin_ws/src/dual_sim/navigation_dual/maps/map
```