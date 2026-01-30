Assignment 1
--------
Check aisd_spring_mass.cpp history for checkpoints in assignment 1

Assignment 2 Checkpoint 1 Steps
-----------------------------------
Gazebo Terminal
```bash 
colcon build --symlink-install
source install/setup.bash
ros2 launch my_bot_description sim.launch.py
```

**Controller terminal:**

```bash
source install/setup.bash
ros2 run my_bot_description wasd_teleop.py
```

Assignment 2 Checkpoint 2 Steps
-----------------------------------
Gazebo Terminal
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch my_bot_description sim.launch.py
```
**Terminal 2:**
RViz
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2
```
The white conical lines denote the camera and the blue lines denote the LiDAR

Checkpoint 2 has a separate folder containing files specific to it
