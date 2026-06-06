<?xml version="1.0"?>
<robot name = "my_robot">
<link name = "base_link">
  <visual>
     <geometry>
       <box size = "1 0.4 0.1"/>
     </geometry>
     <material name="blue">
       <color rgba="0 0 0.8 1"/>
     </material>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <!--axis-->
  </visual>
  <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <box size = "1 0.4 0.1"/>
     </geometry>
  </collision>
  <inertial>
    <mass value = "10"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.138" ixy="0" ixz="0" iyy="0.843" iyz="0" izz="0.967"/>
  </inertial>
</link>

<link name = "weight_link">
   <visual>
     <geometry>
       <box size = "0.35 0.35 0.35"/>
     </geometry>
     <material name="red">
       <color rgba="0.8 0 0 1"/>
     </material>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <!--axis-->
   </visual>
   <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
       <box size = "0.5 0.5 0.5"/>
     </geometry>
   </collision>
   <inertial>
     <mass value = "5"/>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <inertia ixx="0.208" ixy="0" ixz="0" iyy="0.208" iyz="0" izz="0.208"/>
   </inertial>
</link>

<joint name = "base_weight_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "weight_link"/>
  <origin xyz="-0.25 0 0.3" rpy="0 0 0"/>
  <!--axis-->
</joint>

<link name = "lifter_1_link">
  <visual>
     <geometry>
        <box size = "0.1 0.1 0.8"/>
     </geometry>
     <material name="yellow">
       <color rgba="0.8 0.8 0 1"/>
     </material>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <!--axis-->
  </visual>
  <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
        <box size = "0.1 0.1 0.8"/>
     </geometry>
  </collision>
  <inertial>
    <mass value = "4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.215" iyz="0" izz="0.215"/>
  </inertial>
</link>

<link name = "lifter_2_link">
  <visual>
     <geometry>
        <box size = "0.1 0.1 0.8"/>
     </geometry>
     <material name="yellow">
       <color rgba="0.8 0.8 0 1"/>
     </material>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <!--axis-->
  </visual>
  <collision>
     <origin xyz="0 0 0" rpy="0 0 0"/>
     <geometry>
        <box size = "0.1 0.1 0.8"/>
     </geometry>
  </collision>
  <inertial>
    <mass value = "4"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.215" iyz="0" izz="0.215"/>
  </inertial>
</link>

<joint name = "base_lifter_1_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "lifter_1_link"/>
  <origin xyz="0.4 0.15 0.45" rpy="0 0 0"/>
  <!--axis-->
</joint>

<joint name = "base_lifter_2_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "lifter_2_link"/>
  <origin xyz="0.4 -0.15 0.45" rpy="0 0 0"/>
  <!--axis-->
</joint>

<link name = "support_link">
  <visual>
    <geometry>
      <box size = "0.1 0.4 0.1"/>
    </geometry>
    <material name="orange">
      <color rgba="1 0.5 0 1"/>
    </material>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!--axis-->
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size = "0.4 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value = "2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.014" iyz="0" izz="0.014"/>
  </inertial>
</link>

<joint name = "lifter_1_support_joint" type = "fixed">
  <parent link = "lifter_1_link"/>
  <child link = "support_link"/>
  <origin xyz="0 -0.15 0.45" rpy="0 0 0"/>
  <!--axis-->
</joint>

<link name="gripper_link">
  <!-- left fork — long prong sticking forward -->
  <visual>
    <origin xyz="0.2 0.1 0"/>
    <geometry><box size="0.35 0.05 0.04"/></geometry>
    <material name="light_pink"><color rgba="1 0.75 0.8 1"/></material>
  </visual>
  <!-- right fork — long prong sticking forward -->
  <visual>
    <origin xyz="0.2 -0.1 0"/>
    <geometry><box size="0.35 0.05 0.04"/></geometry>
    <material name="light_pink"><color rgba="1 0.75 0.8 1"/></material>
  </visual>
  <!-- back plate connecting both forks -->
  <visual>
    <origin xyz="0 0 0"/>
    <geometry><box size="0.05 0.3 0.20"/></geometry>
    <material name="light_pink"><color rgba="1 0.75 0.8 1"/></material>
  </visual>

  <collision>
    <origin xyz="0.3 0.1 0"/>
    <geometry><box size="0.6 0.06 0.04"/></geometry>
  </collision>
  <collision>
    <origin xyz="0.3 -0.1 0"/>
    <geometry><box size="0.6 0.06 0.04"/></geometry>
  </collision>
  <collision>
    <origin xyz="0 0 0"/>
    <geometry><box size="0.06 0.3 0.25"/></geometry>
  </collision>

  <inertial>
    <mass value="1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lifter_gripper_joint" type="prismatic">
  <parent link="support_link"/>
  <child link="gripper_link"/>
  <origin xyz="0.05 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 -1"/>
  <limit lower="0" upper="0.3" effort="10" velocity="0.5"/>
</joint>

<link name = "wheel_1_link">
  <visual>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <!--axis-->
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
  </collision>
  <inertial>
    <mass value = "1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0006" ixy="0" ixz="0" iyy="0.0006" iyz="0"  izz="0.00125"/>
  </inertial>
</link>

<link name = "wheel_2_link">
  <visual>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <!--axis-->
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
  </collision>
  <inertial>
    <mass value = "1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0006" ixy="0" ixz="0"  iyy="0.0006" iyz="0"   izz="0.00125"/>
  </inertial>
</link>

<link name = "wheel_3_link">
  <visual>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <!--axis-->
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
  </collision>
  <inertial>
    <mass value = "1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0006" ixy="0" ixz="0"  iyy="0.0006" iyz="0"   izz="0.00125"/>
  </inertial>
</link>

<link name = "wheel_4_link">
  <visual>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <!--axis-->
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder length="0.04" radius="0.08"/>
    </geometry>
  </collision>
  <inertial>
    <mass value = "1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0006" ixy="0" ixz="0" iyy="0.0006" iyz="0" izz="0.00125"/>
  </inertial>
</link>

<joint name = "base_wheel_1_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "wheel_1_link"/>
  <origin xyz="0.4 0.2 -0.1" rpy="0 0 0"/>
  <!--axis-->
</joint>

<joint name = "base_wheel_2_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "wheel_2_link"/>
  <origin xyz="0.4 -0.2 -0.1" rpy="0 0 0"/>
  <!--axis-->
</joint>

<joint name = "base_wheel_3_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "wheel_3_link"/>
  <origin xyz="-0.4 0.2 -0.1" rpy="0 0 0"/>
  <!--axis-->
</joint>

<joint name = "base_wheel_4_joint" type = "fixed">
  <parent link = "base_link"/>
  <child link = "wheel_4_link"/>
  <origin xyz="-0.4 -0.2 -0.1" rpy="0 0 0"/>
</joint>

</robot>
