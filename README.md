# industrial-4dof-arm-ros2

URDF/Xacro model of a 4-DOF industrial robotic arm (waist, shoulder, elbow, wrist) for
ROS 2 Humble / Iron, with an RViz2 display launch file.

The ROS 2 package is in [`industrial_4dof_arm/`](industrial_4dof_arm/). Its
[README](industrial_4dof_arm/README.md) is the full project report: design, link and joint
tables, build and run commands, and RViz and TF tree verification. A PDF rendering is in
[`industrial_4dof_arm/docs/Project_Report.pdf`](industrial_4dof_arm/docs/Project_Report.pdf).

## Quick start

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ArpitAwasthi2411/industrial-4dof-arm-ros2.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select industrial_4dof_arm
source install/setup.bash
ros2 launch industrial_4dof_arm display.launch.py
```
