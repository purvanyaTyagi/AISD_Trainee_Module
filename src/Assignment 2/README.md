
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

