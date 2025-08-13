# Launch Commands for ARX Robots

## Basic Teleop Commands

### Single Arm Robots

#### Arx5 Robot
```bash
# 使用默认UMI夹爪
python ../examples/teleop_robosuite.py --env.robots Arx5 --device.type ros2_joy

# 指定使用UMI夹爪
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy

# 使用其他夹爪类型
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types ArxGripper --device.type ros2_joy
```

#### ArxR5 Robot
```bash
# 使用默认Arx夹爪
python ../examples/teleop_robosuite.py --env.robots ArxR5 --device.type ros2_joy

# 指定夹爪类型
python ../examples/teleop_robosuite.py --env.robots ArxR5 --env.gripper_types UMIGripper --device.type ros2_joy
```

#### ArxX5 Robot
```bash
# 使用默认夹爪
python ../examples/teleop_robosuite.py --env.robots ArxX5 --device.type ros2_joy

# 自定义夹爪
python ../examples/teleop_robosuite.py --env.robots ArxX5 --env.gripper_types UMIGripper --device.type ros2_joy
```

### Dual Arm Robots

#### ArxR5Dual (双臂配置)
```bash
# 使用默认Arx夹爪（双臂）
python ../examples/teleop_robosuite.py --env.robots ArxR5Dual --env.environment TwoArmLift --env.config bimanual --device.type ros2_joy

# 双臂使用不同夹爪
python ../examples/teleop_robosuite.py --env.robots ArxR5Dual --env.environment TwoArmLift --env.config bimanual --env.gripper_types ArxGripper,UMIGripper --device.type ros2_joy

# 双臂使用相同夹爪
python ../examples/teleop_robosuite.py --env.robots ArxR5Dual --env.environment TwoArmLift --env.config bimanual --env.gripper_types UMIGripper,UMIGripper --device.type ros2_joy
```

#### ArxX7S (双臂配置)
```bash
# 使用默认夹爪配置
python ../examples/teleop_robosuite.py --env.robots ArxX7S --env.environment TwoArmLift --env.config bimanual --device.type ros2_joy

# 自定义夹爪配置
python ../examples/teleop_robosuite.py --env.robots ArxX7S --env.environment TwoArmLift --env.config bimanual --env.gripper_types ArxGripper,ArxGripper --device.type ros2_joy
```

## Environment Examples

### Lift Environment
```bash
# 单臂举升任务
python ../examples/teleop_robosuite.py --env.environment Lift --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy

# 双臂举升任务
python ../examples/teleop_robosuite.py --env.environment TwoArmLift --env.robots ArxR5Dual --env.config bimanual --env.gripper_types ArxGripper,UMIGripper --device.type ros2_joy
```

### Pick and Place Environment
```bash
# 单臂抓取放置
python ../examples/teleop_robosuite.py --env.environment PickPlace --env.robots ArxR5 --env.gripper_types ArxGripper --device.type ros2_joy

# 双臂抓取放置
python ../examples/teleop_robosuite.py --env.environment TwoArmPickPlace --env.robots ArxX7S --env.config bimanual --env.gripper_types ArxGripper,ArxGripper --device.type ros2_joy
```

### Door Environment
```bash
# 开门任务
python ../examples/teleop_robosuite.py --env.environment Door --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy
```

## Gripper Configuration

### Available Gripper Types
- `UMIGripper`: UMI夹爪，适用于Arx5等机器人
- `ArxGripper`: ARX专用夹爪，适用于ArxR5等机器人
- `G1ThreeFingerGripper`: 三指夹爪，适用于G1等机器人
- `SchunkSvhHand`: Schunk SVH手部，适用于人形机器人

### Gripper Selection Tips
1. **单臂机器人**: 使用单个夹爪名称，如 `--env.gripper_types UMIGripper`
2. **双臂机器人**: 使用逗号分隔的两个夹爪名称，如 `--env.gripper_types ArxGripper,UMIGripper`
3. **默认夹爪**: 如果不指定 `--env.gripper_types`，将使用机器人预定义的默认夹爪
4. **夹爪兼容性**: 确保选择的夹爪与机器人机械结构兼容

## Data Collection with Teleop

### Enable Data Collection
```bash
# 启用数据收集的teleop
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy --collection.enabled true

# 指定数据保存目录
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy --collection.enabled true --collection.directory /path/to/demos
```

### Data Collection Tips
- 数据收集会自动保存成功的演示
- 每个演示包含状态、动作和模型XML文件
- 可以通过 `--collection.directory` 指定保存路径
- 数据以HDF5格式保存，便于后续训练和分析

## Controller Configuration

### Basic Controller
```bash
# 使用基础控制器
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --control.controller BASIC --device.type ros2_joy
```

### Whole Body IK Controller
```bash
# 使用全身逆运动学控制器
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --control.controller WHOLE_BODY_IK --device.type ros2_joy
```

## Device Configuration

### ROS2 Joy Controller
```bash
# 使用默认ROS2 Joy话题
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy

# 自定义Joy话题
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy --device.joy_topic /custom_joy

# 调整灵敏度
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type ros2_joy --device.pos_sensitivity 1.5 --device.rot_sensitivity 2.0
```

### Other Input Devices
```bash
# 键盘控制
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type keyboard

# SpaceMouse控制
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type spacemouse

# DualSense控制器
python ../examples/teleop_robosuite.py --env.robots Arx5 --env.gripper_types UMIGripper --device.type dualsense
```