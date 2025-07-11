# Launch Scripts for Robots

* When Viewer launch:
  * press `Tab` to show the panel
  * Press `[` and `]`  to switch the camera 

## 常用robosuite使用技巧
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

## ARX Robotics

### ARX R5 Series
* ARX R5
```bash
python examples/random_action.py
```

* ARX Dual
```bash
python examples/random_action.py --robots ArxR5Dual
```

* ARX Lift (with dual R5)
```bash
python examples/random_action.py --robots ArxLift
```
```bash
python examples/teleop_robosuite.py --env.robots ArxLift
```

### ARX X5 Series
* ARX X5
```bash
python examples/random_action.py --robots ArxX5
```
* ARX Dual
```bash
python examples/random_action.py --robots ArxX5Dual
```

* ARX Lift (with dual X5)
```bash
python examples/random_action.py --robots ArxLift2
```
```bash
python examples/teleop_robosuite.py --env.robots ArxLift2
```

### ARX X7 Series

* ARX X7S Arms Only
```bash
python examples/random_action.py --robots ArxX7sArmsOnly
```

* ARX X7S
```bash
python examples/random_action.py --robots ArxX7s
```
```bash
python examples/teleop_robosuite.py --env.robots ArxX7s
```
```bash
python examples/teleop_robosuite.py --env.robots ArxX7s --control.controller WHOLE_BODY_IK --device.type mjgui
```

## Dobot 越疆机器人
### Dobot CR5
* Random Action
  ```bash
  python examples/random_action.py --robots DobotCR5
  ```
* Teleop in RoboSuite
  ```bash
  python examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True
  ```
* Record Demonstration
  ```bash
  python examples/teleop_robosuite.py --env.robots DobotCR5 --collection.enabled true --collection.directory datasets/dobot_lift
  ```
  ```bash
  python examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True --env.translucent_robot True --collection.enabled true --collection.directory datasets/dobot_microwave
  ```
  
* Playback Demonstration
  ```bash
  python -m robosuite.scripts.playback_demonstrations_from_hdf5 --use-actions --folder ./datasets/dobot_microwave/
  ```

* Isaac Render
  ```bash
  python -m robosuite.scripts.render_dataset_with_omniverse --ds_format robosuite --episode 1 --camera agentview robot0_eye_in_hand --width 1920 --height 1080 --renderer PathTracing --save_video --hide_sites --rgb --normals --dataset ./datasets/dobot_microwave/demo.hdf5
  ```

## SO Arms
### SO101 Series
* Random Action
  ```bash
  python examples/random_action.py --robots SO101
  ```
* Teleop in RoboSuite
  ```bash
  python examples/teleop_robosuite.py --env.robots SO101 --control.controller BASIC
  ```
  ```bash
  python examples/teleop_robosuite.py --env.robots SO101 --control.controller WHOLE_BODY_IK --device.type mjgui
  ```
  ```bash
  python examples/teleop_robosuite.py --env.robots SO101 --device.type lerobot_lead --device.teleoperator.type=so101_leader --device.teleoperator.port=/dev/ttyACM0 --device.teleoperator.id=my_awesome_leader_arm
  ```
* Teleop in Robocasa
  ```bash
  python examples/teleop_robocasa.py --robot SO101Omron --device spacemouse
  ```