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

URDF-based gravity compensation also requires Pinocchio in the runtime environment. It is not installed by the
default SDK dependencies because most SDK workflows do not need robot dynamics.

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

## Gravity Compensation

The repository includes an application-level gravity compensation runner in `app/gravity_compensation.py`. It keeps
the SDK API small while using the existing torque-control path:

1. read joint positions from `SupreRobot.get_joint_positions()`;
2. slowly move controlled joints to `initial_positions_deg` in position mode;
3. switch supported joints to torque mode;
4. compute generalized gravity torque with Pinocchio;
5. convert Nm to Eyou `torque_milli` and send commands with `SupreRobot.send_joint_torques()`.

Install the normal SDK dependencies and make sure the runtime environment also provides `pinocchio`, `eu_motor_py`,
and the physical Eyou CAN motor bindings:

```bash
python -m pip install -e .
```

Edit `app/gravity_compensation_config.yaml` before running:

- `urdf_path`: absolute or relative path to the robot URDF.
- `controlled_joints`: joints that should receive gravity compensation torque. Do not include grippers or other
  hardware that does not support torque control.
- `rated_torque_nm`: rated motor torque for each controlled joint. The runner converts with
  `torque_milli = torque_nm / rated_torque_nm * 1000`.
- `initial_positions_deg`: starting pose for gravity compensation. The runner moves here before torque mode.
- `initial_move_duration_s`, `initial_move_frequency`, and `initial_move_max_step_deg`: slow position-mode move into
  the initial pose.
- `max_initial_deviation_deg`: allowed deviation from the initial pose during gravity compensation.
- `max_abs_torque_milli`: safety clamp for every commanded torque value.
- `torque_scale`: per-joint multiplier applied to model torque. Start with `0.1` on real hardware.
- `max_torque_step_milli` and `ramp_duration_s`: limit sudden command changes at startup and during the loop.
- `max_position_step_deg` and `max_velocity_deg_s`: watchdog thresholds that stop the runner on unexpected motion.
- `position_limits_deg`: optional extra absolute limits. The main runtime workspace should normally be controlled by
  `initial_positions_deg` and `max_initial_deviation_deg`.
- `frequency` and `interpolation_period_ms`: loop frequency and Eyou CST interpolation period.

Before first hardware testing, follow `docs/gravity_compensation_safety.md`.

First validate the model, signs, and torque magnitudes without sending gravity compensation torque commands:

```bash
python app/gravity_compensation.py \
  --robot-config examples/robot_config.yaml \
  --gravity-config app/gravity_compensation_config.yaml \
  --duration 5 \
  --dry-run
```

When the printed torque values look reasonable, run against hardware:

```bash
python app/gravity_compensation.py \
  --robot-config examples/robot_config.yaml \
  --gravity-config app/gravity_compensation_config.yaml \
  --duration 10
```

Use `--duration 0` to run until interrupted. On exit, the runner sends zero torque to the controlled joints and then
disconnects the robot.

# supre_robot_sdk
