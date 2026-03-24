# supre-robot-sdk

Standalone Python SDK for Supre dual-arm robots.

## Install

```bash
python -m pip install -e ".[dev]"
```

## Runtime dependencies

- `eu_motor_py` for Eyou CAN motors
- `jodell_gripper_py` for Jodell grippers

The test suite mocks these bindings, so physical hardware is not required for unit tests.

## Usage

```python
from supre_robot_sdk import SupreRobot

robot = SupreRobot("examples/robot_config.yaml", use_interpolation=True)
robot.connect()

print(robot.get_joint_positions())
robot.move_joint("left_arm_joint_1", 45.0)
robot.open_gripper("right")

robot.disconnect()
```

## Config

See `examples/robot_config.yaml`.

# supre_robot_sdk
