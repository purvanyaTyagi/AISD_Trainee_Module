
# 🤖 Autonomous Mobile Robot (AISD Module) Autonomous integration and simulation development

**Current Status:** ✅ Checkpoint 2 Complete (Sensors & Physics Tuned)  
**Next Step:** Checkpoint 3 (Camera & Mapping)

---

## 📝 Project Summary (What has been done)

We have successfully built a simulation of a 3-wheeled Differential Drive Robot in Gazebo. The robot is fully capable of teleoperation and environmental perception.

### 🔧 Key Features Implemented

#### 1. Robot "Body" (URDF/Xacro)
* **Chassis Design:** Designed a custom rectangular chassis with a caster wheel for stability.
* **Differential Drive:** Implemented 2 active wheels controlled by the `diff_drive` Gazebo plugin.
* **Lidar Integration:** Mounted a generic Ray Sensor (Lidar) on top of the chassis to scan obstacles (360° view).

#### 2. Physics & Tuning (Problem Solving)
* **Stabilization:** Initially, the robot toppled (did a "wheelie") during high acceleration.
    * *Fix:* Increased chassis mass to **15kg** and lowered the **Center of Mass (CoM)** to `-0.05m`.
* **Inertia Management:** Tuned the `<inertial>` matrix to simulate a heavy, grounded vehicle.
* **Movement Smoothing:** Adjusted `max_wheel_acceleration` (Slew Rate) to **2.0** to prevent jerking while maintaining responsive braking.

#### 3. Sensor Calibration
* **Lidar Obstruction Fix:** The initial Lidar position caused the laser rays to hit the robot's own wheels.
    * *Fix:* Raised the Lidar joint `z-offset` to **0.15m** to clear the wheel radius.
* **Visualization:** Configured **RViz2** to visualize the `/scan` topic, showing accurate wall detection in real-time.
note - i have not done the camera and mapping that is checkpoint 3 but soon i will do after perception may be .
---

## 🚀 How to Test My Robot

**1. Clone the Repository**
```bash
git clone <YOUR_GITHUB_LINK_HERE>
cd ~/a2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
2. Launch Simulation (Gazebo)

Bash
ros2 launch gazebo_ros gazebo.launch.py
3. Spawn the Robot

Bash
ros2 run gazebo_ros spawn_entity.py -entity my_mobile_bot -file src/AISD_Trainee_Module/src/my_robot_description/urdf/my_robot.urdf.xacro
4. See What the Robot Sees (RViz)

Bash
# Starts the State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/AISD_Trainee_Module/src/my_robot_description/urdf/my_robot.urdf.xacro)"

# Open RViz (Add 'LaserScan' to view the Lidar)
rviz2
5. Drive It!

Bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
(Use q and z to adjust speed/acceleration)
How to Configure RViz (Once it opens)
When RViz opens, it will be empty. You must manually add the displays.

Set the Fixed Frame (Top Left):

Find the setting named "Fixed Frame".

Change it from map to odom.

(If odom isn't there, type it manually).

Add the Robot Model:

Click the "Add" button (Bottom Left).

Scroll down and select RobotModel.

Click OK.

Result: You should see your robot (Blue body, Black wheels) in the center.

Add the Lidar (LaserScan):

Click "Add" again.

Select LaserScan.

Click OK.

Important: Expand the LaserScan settings on the left.

Find "Topic" and select /scan from the dropdown menu.

Find "Size (m)" and change it to 0.1 (makes dots bigger).

Add the Coordinates (TF):

Click "Add" -> Select TF.

This shows the "Frames" (arrows) for your wheels and chassis moving in real-time.

Summary of Controls
Move Camera: Shift + Click & Drag (on trackpad).

Drive Robot: Open a 5th Terminal and run:

Bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
Pro Tip: Once you have RViz set up perfectly, you can hit File -> Save Config (Ctrl+S). Next time you run rviz2, it will remember everything!
## 🗺️ Checkpoint 3: Mapping the World (SLAM)

We have successfully integrated the `slam_toolbox` to generate a 2D occupancy grid map of the Gazebo environment.

### 🔧 Key Features Implemented

#### 1. Perception (Camera & Lidar)
* **RGB Camera:** Added a camera sensor plugin to the robot URDF (`libgazebo_ros_camera.so`) to provide visual feedback.
* **Lidar Mapping:** Configured the existing Lidar to work with SLAM, ensuring the `lidar_link` frame is correctly transformed to the `map` frame.

#### 2. Simultaneous Localization and Mapping (SLAM)
* **Asynchronous Mapping:** utilized `slam_toolbox` in `online_async` mode. This allows the robot to:
    * Estimate its position (`odom` -> `map` transform).
    * Detect obstacles (walls, cylinders).
    * Update the map in real-time as it explores.

#### 3. Map Artifacts
* **Generated Map:** Successfully explored the world and saved the environment as `my_map.pgm` and `my_map.yaml`.
* **Map Visualization:** Verified the occupancy grid in RViz2 (White = Free Space, Black = Obstacles).

---

## 🚀 How to Run Mapping (SLAM)

**1. Launch Simulation**
```bash
ros2 launch gazebo_ros gazebo.launch.py
2. Spawn Robot

Bash
ros2 run gazebo_ros spawn_entity.py -entity my_mobile_bot -file src/AISD_Trainee_Module/src/my_robot_description/urdf/my_robot.urdf.xacro
3. Start State Publisher

Bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/AISD_Trainee_Module/src/my_robot_description/urdf/my_robot.urdf.xacro)" -p use_sim_time:=true
4. Start SLAM

Bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
5. Visualize & Drive

Bash
ros2 run rviz2 rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard

