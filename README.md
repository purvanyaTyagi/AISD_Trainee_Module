Assignment 1
--------
Check aisd_spring_mass.cpp history for checkpoints in assignment 1

Assignment 2 Checkpoint 1 Steps
-----------------------------------
colcon build --symlink-install

source install/setup.bash

ros2 launch my_bot_description sim.launch.py

**Controller terminal:**

source install/setup.bash

ros2 run my_bot_description wasd_teleop.py
