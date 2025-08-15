# Launch command for Dobot CR5

* Random Action
    ```bash
    python ../examples/random_action.py --robots DobotCR5
    ```

## Teleop with ROS2 Joy

### Microwave
* AG2F90-C Gripper
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Microwave --env.mirror_actions True --device.type ros2_joy
    ```
  
### Pick and Place
* AG2F90-C Gripper
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment PickPlace --device.type ros2_joy
    ```
* With Robotiq 85 Adaptive Gripper
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment PickPlace --device.type ros2_joy --env.gripper_types Robotiq85AdaptiveGripper
    ```

### Open Door
* AG2F90-C Gripper
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Door --device.type ros2_joy
    ```
* Robotiq 85 Adaptive Gripper
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment Door --device.type ros2_joy --env.gripper_types Robotiq85AdaptiveGripper
    ```
  
### Pick Place Tabletop
* Pick and Place
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment PickPlaceTable --device.type ros2_joy
    ```
* Pick and Place (With Robotiq 85 Adaptive Gripper)
    ```bash
    python ../examples/teleop_robosuite.py --env.robots DobotCR5 --env.environment PickPlace --device.type ros2_joy --env.gripper_types Robotiq85AdaptiveGripper
    ```