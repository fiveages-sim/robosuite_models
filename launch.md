# Launch Scripts for Robots

* When Viewer launch:
  * press `Tab` to show the panel
  * Press `[` and `]`  to switch the camera 

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

### ARX X7 Series

* ARX X7S Arms Only
```bash
python examples/random_action.py --robots ArxX7sArmsOnly
```

* ARX X7S
```bash
python examples/random_action.py --robots ArxX7s
```

## SO Arms
### SO101 Series
* Random Action
  ```bash
  python examples/random_action.py --robots SO101
  ```
* Teleop in RoboSuite
  ```bash
  python examples/teleop_robosuite.py --robots SO101
  ```
* Teleop in Robocasa
  ```bash
  python examples/teleop_robocasa.py --robot SO101Omron --device spacemouse
  ```