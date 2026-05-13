# Gravity Compensation Safety Procedure

This procedure is for first hardware bring-up of `app/gravity_compensation.py`. Treat the URDF, motor torque
constants, joint signs, and payload data as untrusted until each joint has been checked on the real robot.

## Before Power-On

- Verify the physical emergency stop cuts motor power or motor enable.
- Remove payloads and tools from the end effector.
- Put soft support under the arm so an unexpected drop cannot hit hard limits or the table.
- Keep people outside the arm workspace and motion plane.
- Start with one joint, or one low-risk arm section, in `controlled_joints`; do not begin with all joints enabled.
- Confirm `urdf_path` points to the intended robot URDF and that URDF joint names match `joint_order`.

## Conservative Initial Config

Use low torque and slow changes for first tests:

```yaml
max_abs_torque_milli: 100.0
max_torque_step_milli: 5.0
ramp_duration_s: 5.0
max_position_step_deg: 5.0
max_velocity_deg_s: 90.0
initial_move_duration_s: 10.0
initial_move_frequency: 30.0
initial_move_max_step_deg: 2.0
max_initial_deviation_deg: 10.0

torque_scale:
  left_arm_joint_1: 0.1

initial_positions_deg:
  left_arm_joint_1: 0.0
```

Keep every tested joint at `0.1` scale until the torque sign is proven correct. Increase scale only in small steps,
for example `0.1`, `0.2`, `0.3`, and stop before the arm starts lifting or drifting on its own.

Set `initial_positions_deg` to a pose where gravity compensation is meaningful and mechanically safe. The runner
slowly moves controlled joints into that pose in position mode before switching to torque mode. During gravity
compensation, `max_initial_deviation_deg` limits how far the user can drag away from that initial pose.

## Dry Run

Run dry-run before enabling torque output:

```bash
python app/gravity_compensation.py \
  --robot-config examples/robot_config.yaml \
  --gravity-config app/gravity_compensation_config.yaml \
  --duration 5 \
  --dry-run
```

Dry-run connects and reads positions, prints the planned initial-pose move size and step count, computes gravity
compensation, applies scale, ramp, and torque-step limiting, then prints the command dictionary. It does not execute
the initial-pose move, does not call `set_enable_torque()`, does not configure torque mode, and does not send gravity
compensation torque commands. The current SDK connection path may still activate configured hardware, so keep the same
physical precautions in place for dry-run.

Stop before hardware testing if:

- any printed torque is unexpectedly large;
- torque signs do not match the expected gravity direction;
- a controlled joint is missing from feedback;
- the URDF does not contain a controlled joint.

## First Hardware Test

1. Select one controlled joint, set a nearby safe `initial_positions_deg`, and keep `torque_scale` at `0.1`.
2. Keep one hand near the emergency stop, not in the arm workspace.
3. Run for a short duration:

   ```bash
   python app/gravity_compensation.py \
     --robot-config examples/robot_config.yaml \
     --gravity-config app/gravity_compensation_config.yaml \
     --duration 3
   ```

4. Confirm the joint becomes slightly lighter and does not drift, lift, drop, oscillate, or accelerate.
5. If direction is wrong, stop and fix joint sign, URDF joint axis, or torque sign before increasing scale.
6. Add the next joint only after the current joint is stable.

## Runtime Safety Behavior

The runner applies these software guards:

- `max_abs_torque_milli`: clamps each commanded torque.
- `torque_scale`: scales model torque per joint before clamping.
- `ramp_duration_s`: ramps from zero command to target command after startup.
- `max_torque_step_milli`: limits command change per loop.
- `initial_positions_deg`: target pose reached in position mode before torque mode.
- `max_initial_deviation_deg`: stops if a joint is dragged too far from the initial pose.
- `max_position_step_deg`: stops if feedback jumps too far in one loop.
- `max_velocity_deg_s`: stops if estimated joint velocity is too high.
- `position_limits_deg`: optional extra hard limit if configured.
- On normal exit or watchdog failure, it sends zero torque to controlled joints and disconnects.

Software guards do not replace physical emergency stop, mechanical support, or careful single-joint bring-up.

## Tuning Notes

- If the arm still feels heavy but stable, increase `torque_scale` gradually.
- If the arm floats upward or moves by itself, reduce `torque_scale` or correct the model/sign.
- If behavior is good in one pose but poor in another, update URDF link masses and centers of mass.
- If drag remains even with good gravity compensation, that is likely friction, reducer behavior, cables, or missing
  damping/admittance control rather than pure gravity error.
