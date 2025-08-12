# Launch Scripts for Robots

## 目录
- [常用robosuite使用技巧](#常用robosuite使用技巧)
  - [基本操作](#基本操作)
  - [ROS2 Joy Controller Setup](#ros2-joy-controller-setup)
  - [配置Spacemouse](#配置spacemouse)
  - [如何使用Mujoco GUI操作机器人](#如何使用mujoco-gui操作机器人)
  - [如何回放数据集](#如何回放数据集)
- [ARX Robotics](#arx-robotics)
  - [ARX R5 Series](#arx-r5-series)
  - [ARX X5 Series](#arx-x5-series)
  - [ARX X7 Series](#arx-x7-series)
- [Dobot 越疆机器人](#dobot-越疆机器人)
  - [Dobot CR5](#dobot-cr5)
- [SO Arms](#so-arms)
  - [SO101 Series](#so101-series)

---

<details>
<summary><strong>常用robosuite使用技巧</strong></summary>

### 基本操作
* When Viewer launch:
  * press `Tab` to show the panel
  * Press `[` and `]`  to switch the camera

### ROS2 Joy Controller Setup
To use ROS2 Joy controller with any robot, follow these steps:

1. **Install ROS2 Joy dependencies**:
   ```bash
   pip install rclpy
   sudo apt install ros-humble-joy
   ```

2. **Run teleop with ROS2 Joy**:
   ```bash
   python examples/teleop_robosuite.py --device.type ros2_joy
   ```

**ROS2 Joy Controls**:
- Left Stick: Move robot end-effector in X-Y plane
- Right Stick X: Rotate robot end-effector (Yaw)
- Right Stick Y: Move robot end-effector in Z-axis
- D-pad X: Rotate robot end-effector (Roll)
- D-pad Y: Rotate robot end-effector (Pitch)
- A Button: Switch active arm (if multi-armed robot)
- B Button: Reset simulation
- X Button: Toggle gripper (open/close)
- Y Button: Switch active robot (if multi-robot environment)
- LB Button: Toggle arm/base mode (if applicable)
- RB Button: Toggle torso mode (if applicable)

**Custom Configuration**:
```bash
# Custom joy topic
python examples/teleop_robosuite.py --device.type ros2_joy --device.joy_topic /custom_joy

# Custom sensitivity
python examples/teleop_robosuite.py --device.type ros2_joy --device.pos_sensitivity 1.5 --device.rot_sensitivity 0.8
```

### 配置Spacemouse
* 查询设备的信息，其中<#>是从0递增的设备序号
```bash
cat /sys/class/hidraw/hidraw<#>/device/uevent
```
```bash
sudo chmod 666 /dev/hidraw<#>
```

### 如何使用Mujoco GUI操作机器人

会在场景中生成一个方块，鼠标单击这个方块，按住键盘的ctrl键和鼠标右键可以控制方块的移动，按住鼠标左键可以控制方块的旋转。

### 如何回放数据集

在robosuite目录下，执行以下命令
```bash
python robosuite/scripts/playback_demonstrations_from_hdf5.py --use-actions --folder   robosuite/models/assets/demonstrations_private/1751959069_4537017/ 
```

如果想要使用isaac来渲染录制的轨迹，可以使用以下命令
```bash
python robosuite/scripts/render_dataset_with_omniverse.py  --ds_format robosuite --episode 1 --camera agentview frontview --width 1920 --height 1080 --renderer RayTracedLighting --save_video --hide_sites --rgb --normals --dataset robosuite/models/assets/demonstrations_private/1751959069_4537017/demo.hdf5
```

</details>

## ARX Robotics

### ARX R5 Series
<details>
<summary><strong>ARX R5</strong></summary>

```bash
python examples/random_action.py
```

</details>

<details>
<summary><strong>ARX Dual</strong></summary>

```bash
python examples/random_action.py --robots ArxR5Dual
```

</details>

<details>
<summary><strong>ARX Lift (with dual R5)</strong></summary>

```bash
python examples/random_action.py --robots ArxLift
```
```bash
python examples/teleop_robosuite.py --env.robots ArxLift
```

</details>

<details>
<summary><strong>ARX Lift with ROS2 Joy controller</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots ArxLift --device.type ros2_joy
```

</details>

<details>
<summary><strong>ARX Lift with ROS2 Joy and data collection</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots ArxLift --device.type ros2_joy --collection.enabled true --collection.directory datasets/arxlift_joy_demos
```

</details>

### ARX X5 Series
<details>
<summary><strong>ARX X5</strong></summary>

```bash
python examples/random_action.py --robots ArxX5
```

</details>

<details>
<summary><strong>ARX Dual</strong></summary>

```bash
python examples/random_action.py --robots ArxX5Dual
```

</details>

<details>
<summary><strong>ARX Lift (with dual X5)</strong></summary>

```bash
python examples/random_action.py --robots ArxLift2
```
```bash
python examples/teleop_robosuite.py --env.robots ArxLift2
```

</details>

<details>
<summary><strong>ARX Lift2 with ROS2 Joy controller</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots ArxLift2 --device.type ros2_joy
```

</details>

### ARX X7 Series

<details>
<summary><strong>ARX X7S Arms Only</strong></summary>

```bash
python examples/random_action.py --robots ArxX7sArmsOnly
```

</details>

<details>
<summary><strong>ARX X7S</strong></summary>

```bash
python examples/random_action.py --robots ArxX7s
```
```bash
python examples/teleop_robosuite.py --env.robots ArxX7s
```
```bash
python examples/teleop_robosuite.py --env.robots ArxX7s --control.controller WHOLE_BODY_IK --device.type mjgui
```

</details>

<details>
<summary><strong>ARX X7S with ROS2 Joy controller</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots ArxX7s --device.type ros2_joy
```

</details>

<details>
<summary><strong>ARX X7S with ROS2 Joy and whole body control</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots ArxX7s --control.controller WHOLE_BODY_IK --device.type ros2_joy
```

</details>

## Dobot 越疆机器人

### Dobot CR5
<details>
<summary><strong>Random Action</strong></summary>

```bash
python examples/random_action.py --robots DobotCR5
```

</details>

<details>
<summary><strong>Teleop in RoboSuite</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True --device.type dualsense
```

</details>

<details>
<summary><strong>Teleop with ROS2 Joy</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True --device.type ros2_joy
```

</details>

<details>
<summary><strong>Record Demonstration</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots DobotCR5 --collection.enabled true --collection.directory datasets/dobot_lift
```
```bash
python examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True --env.translucent_robot True --collection.enabled true --collection.directory datasets/dobot_microwave
```

</details>

<details>
<summary><strong>Playback Demonstration</strong></summary>

```bash
python -m robosuite.scripts.playback_demonstrations_from_hdf5 --use-actions --folder ./datasets/dobot_microwave/
```

</details>

<details>
<summary><strong>Isaac Render</strong></summary>

```bash
python -m robosuite.scripts.render_dataset_with_omniverse --ds_format robosuite --episode 1 --camera agentview robot0_eye_in_hand --width 1920 --height 1080 --renderer PathTracing --save_video --hide_sites --rgb --normals --dataset ./datasets/dobot_microwave/demo.hdf5
```

</details>

## SO Arms

### SO101 Series
<details>
<summary><strong>Random Action</strong></summary>

```bash
python examples/random_action.py --robots SO101
```

</details>

<details>
<summary><strong>Teleop in RoboSuite</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots SO101 --control.controller BASIC
```
```bash
python examples/teleop_robosuite.py --env.robots SO101 --control.controller WHOLE_BODY_IK --device.type mjgui
```

</details>

<details>
<summary><strong>Teleop with ROS2 Joy</strong></summary>

```bash
python examples/teleop_robosuite.py --env.robots SO101 --control.controller BASIC --device.type ros2_joy
```
```bash
python examples/teleop_robosuite.py --env.robots SO101 --control.controller WHOLE_BODY_IK --device.type ros2_joy
```
```bash
python examples/teleop_robosuite.py --env.robots SO101 --device.type lerobot_lead --device.teleoperator.type=so101_leader --device.teleoperator.port=/dev/ttyACM0 --device.teleoperator.id=my_awesome_leader_arm
```

</details>

<details>
<summary><strong>Teleop in Robocasa</strong></summary>

```bash
python examples/teleop_robocasa.py --robot SO101Omron --device spacemouse
```

</details>