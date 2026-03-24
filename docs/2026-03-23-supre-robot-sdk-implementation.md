# Supre Robot SDK — Standalone Implementation Plan（修正版）

**Goal:** 在新的项目路径中创建独立 `supre_robot_sdk`，从当前仓库抽取 Supre 硬件控制核心，但不携带 `lerobot` 运行时依赖。  
**Recommended root:** `~/projects/supre-robot-sdk/`

---

## 0. 先修正原计划的执行假设

以下几件事必须先改正，否则实现会跑偏：

1. **独立 SDK 可以做，但不能直接搬 `SupreRobotFollower` 整体**
   `supre_robot_follower.py` 不是纯硬件封装，里面混入了 LeRobot 适配、安全、相机和指标逻辑。

2. **必须把共享 manager 当成主要抽取对象**
   目前真正的核心在 `src/lerobot/robots/supre_robot/supre_robot_hardware_manager.py`，不是 follower/leader 任一单文件。

3. **必须一起处理 async interpolation**
   当前 manager 已依赖 `AsyncInterpolator`。如果独立 SDK 要保持接近现有行为，不能忽略它。

4. **必须先消灭 `lerobot` 依赖**
   独立 SDK 代码中不得继续出现 `lerobot.*` 导入。

---

## 1. Create External Project Skeleton

**Project root:** `~/projects/supre-robot-sdk/`

**Files to create**

- `pyproject.toml`
- `README.md`
- `src/supre_robot_sdk/__init__.py`
- `src/supre_robot_sdk/exceptions.py`
- `src/supre_robot_sdk/core/__init__.py`
- `src/supre_robot_sdk/core/interfaces.py`
- `src/supre_robot_sdk/core/config.py`
- `src/supre_robot_sdk/core/hardware_manager.py`
- `src/supre_robot_sdk/core/robot.py`
- `src/supre_robot_sdk/hardware/__init__.py`
- `src/supre_robot_sdk/hardware/base.py`
- `src/supre_robot_sdk/hardware/interpolation/__init__.py`
- `src/supre_robot_sdk/hardware/interpolation/async_interpolator.py`
- `src/supre_robot_sdk/hardware/eyou/__init__.py`
- `src/supre_robot_sdk/hardware/eyou/motor.py`
- `src/supre_robot_sdk/hardware/jodell/__init__.py`
- `src/supre_robot_sdk/hardware/jodell/gripper.py`
- `examples/robot_config.yaml`
- `tests/...`

**Suggested `pyproject.toml`**

```toml
[build-system]
requires = ["setuptools>=61.0", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "supre-robot-sdk"
version = "0.1.0"
description = "Standalone hardware SDK for Supre dual-arm robots"
readme = "README.md"
requires-python = ">=3.10"
dependencies = ["pyyaml"]

[project.optional-dependencies]
dev = ["pytest>=7.0"]

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = ["test_*.py"]
```

**Validation**

```bash
cd ~/projects/supre-robot-sdk
python -m pip install -e ".[dev]"
python -c "import supre_robot_sdk; print('ok')"
```

---

## 2. Implement Base Interfaces and Exceptions

**Source references**

- `src/lerobot/motors/eyou/hardware_interface.py`

**Files**

- `src/supre_robot_sdk/hardware/base.py`
- `src/supre_robot_sdk/core/interfaces.py`
- `src/supre_robot_sdk/exceptions.py`
- `tests/test_interfaces.py`

**What to implement**

- `HardwareInterface`
- `RobotInterface` or a light façade contract for `SupreRobot`
- dedicated exceptions:
  - `ConfigurationError`
  - `HardwareInitError`
  - `HardwareActivationError`
  - `NotConnectedError`

**Corrections versus original plan**

- 可以保留抽象接口，但不要把它设计成 LeRobot 风格的完整平行框架
- 重点是支持 SDK 自己的 façade，不是复制 `Robot`/`Teleoperator`

**Validation**

```bash
pytest tests/test_interfaces.py -v
```

---

## 3. Implement Typed Config Loader

**Why this task is required**

原计划缺少独立 SDK 内的集中配置校验层，而当前仓库已经暴露真实配置问题：

- `baud_rate"` 错键
- joint mapping 靠运行时才发现
- 多份 YAML 容易分叉

**Files**

- `src/supre_robot_sdk/core/config.py`
- `tests/test_config.py`
- `examples/robot_config.yaml`

**What to implement**

- dataclass models:
  - `JointBinding`
  - `HardwareInterfaceConfig`
  - `SupreRobotConfig`
- `load_robot_config(path)` loader

**Loader responsibilities**

- load YAML
- validate `joint_order`
- validate `hardware_interfaces`
- detect duplicate joints
- detect unmapped joints
- detect unknown hardware type
- normalize optional interpolation config

**Important extraction correction**

独立 SDK 不应继续使用 `joint_config_file` / `joint_config_path` 这种二级配置入口。SDK 只接受：

```python
SupreRobot("path/to/robot_config.yaml")
```

**Validation**

```bash
pytest tests/test_config.py -v
```

---

## 4. Port AsyncInterpolator

**Source reference**

- `src/lerobot/motors/eyou/async_interpolator.py`

**Files**

- `src/supre_robot_sdk/hardware/interpolation/async_interpolator.py`
- `tests/test_async_interpolator.py`

**What to preserve**

- `interpolation_n <= 1` 时直通
- 后台线程异步写入
- `Queue(maxsize=1)` 只保留最新目标
- 兼容 `HardwareInterface`

**What to clean**

- 去除对仓库内路径和注释的依赖
- 修正类型标注
- 确保 `activate()` bootstrap 逻辑能处理 `(pos, force)` 形式的 read 结果

**Validation**

```bash
pytest tests/test_async_interpolator.py -v
```

---

## 5. Port EyouMotorHardware

**Source reference**

- `src/lerobot/motors/eyou/eyou_motor_hardware.py`

**Files**

- `src/supre_robot_sdk/hardware/eyou/motor.py`
- `tests/test_eyou_motor.py`

**What to preserve**

- baud rate mapping
- `start_enabled`
- activation readback
- `read()` 读取最新反馈
- `set_enable_torque()` 中启用前同步当前位置

**What to change**

- 不再导入 `lerobot.*`
- 清理 `__main__` demo
- 用明确异常代替无结构的 `print + return False`
- 当 `eu_motor_py` 不可用时，给出清晰错误

**Validation**

```bash
pytest tests/test_eyou_motor.py -v
```

---

## 6. Port JodellGripperHardware

**Source reference**

- `src/lerobot/motors/gripper/jodell_gripper_hardware.py`

**Files**

- `src/supre_robot_sdk/hardware/jodell/gripper.py`
- `tests/test_jodell_gripper.py`

**What to preserve**

- 0.0-1.0 gripper normalization
- cached read behavior
- `None` command handling
- bus/client lifecycle

**What to change**

- 去掉 `lerobot` 依赖
- 用结构化异常
- 清理 `__main__` demo

**Validation**

```bash
pytest tests/test_jodell_gripper.py -v
```

---

## 7. Implement HardwareManager

**Primary source**

- `src/lerobot/robots/supre_robot/supre_robot_hardware_manager.py`

**Files**

- `src/supre_robot_sdk/core/hardware_manager.py`
- `tests/test_hardware_manager.py`

**What to implement**

- config-driven hardware construction
- optional interpolation wrapping
- joint map building
- `init()`
- `activate()`
- `deactivate()`
- `read()`
- `write()`
- `set_enable_torque()`

**Hardware resolution**

第一版用简单映射表即可：

```python
HARDWARE_TYPE_MAP = {
    "EyouMotorHardware": EyouMotorHardware,
    "JodellGripperHardware": JodellGripperHardware,
}
```

不要求一开始就做自注册装饰器系统。

**Required fixes over current code**

- 从 typed config 而不是裸 dict 驱动
- 不用 `print` 当主错误处理
- 明确处理重复 joint 映射
- 在 wrapper 情况下仍能正确 `read()` / `write()`

**Validation**

```bash
pytest tests/test_hardware_manager.py -v
```

---

## 8. Implement `SupreRobot` Façade

**Files**

- `src/supre_robot_sdk/core/robot.py`
- `src/supre_robot_sdk/__init__.py`
- `tests/test_robot.py`

**Reference sources**

- manager behavior from `src/lerobot/robots/supre_robot/supre_robot_hardware_manager.py`
- API semantics参考 `src/lerobot/robots/supre_robot_follower/supre_robot_follower.py`
- gripper convenience semantics参考 `src/lerobot/teleoperators/supre_robot_leader/supre_robot_leader.py`

**Important correction**

这里的 `SupreRobot` 不是 LeRobot `Robot` 子类，而是 SDK 自己的高层 façade。

**Suggested API**

- `connect()`
- `disconnect()`
- `is_connected`
- `get_joint_positions()`
- `get_joint_forces()`
- `send_joint_positions()`
- `move_joint()`
- `move_joints()`
- `open_gripper("left" | "right")`
- `close_gripper("left" | "right")`
- `set_enable_torque()`
- `execute_trajectory(goal_positions, duration)`

**Out of scope**

- cameras
- safety clamping
- Prometheus
- LeRobot feature dictionaries

**Validation**

```bash
pytest tests/test_robot.py -v
```

---

## 9. Write Example Config and README

**Files**

- `examples/robot_config.yaml`
- `README.md`

**README must include**

- install
- dependency expectations
- example config
- basic usage
- torque enable/disable
- interpolation option
- note that `eu_motor_py` and `jodell_gripper_py` are external native bindings

**Usage example**

```python
from supre_robot_sdk import SupreRobot

robot = SupreRobot("examples/robot_config.yaml", use_interpolation=True)
robot.connect()

print(robot.get_joint_positions())
robot.move_joint("left_arm_joint_1", 45.0)
robot.open_gripper("right")

robot.disconnect()
```

---

## 10. Add Independence Tests

**Files**

- `tests/test_no_lerobot_dependency.py`

**What to verify**

- top-level import does not require `lerobot`
- package can import in a minimal environment with only declared dependencies plus mocked native bindings

**Example**

```bash
python -c "from supre_robot_sdk import SupreRobot, HardwareManager; print('OK')"
```

This should succeed without `lerobot` installed.

---

## 11. Full Verification

```bash
cd ~/projects/supre-robot-sdk
python -m pip install -e ".[dev]"
pytest tests -v
python -c "from supre_robot_sdk import SupreRobot; print('OK')"
```

如有真实硬件，再补充最小冒烟验证：

```python
from supre_robot_sdk import SupreRobot

robot = SupreRobot("/path/to/robot_config.yaml")
robot.connect()
positions = robot.get_joint_positions()
print(positions)
robot.disconnect()
```

---

## 12. Final Notes for Executor

1. **不要把 `supre_robot_follower.py` 整体复制成 SDK robot**
   只能借用其中和 façade 相关的小部分语义。

2. **不要把 `lerobot` 留在依赖里**
   独立 SDK 一旦依赖 `lerobot`，整个抽取就失去意义。

3. **必须修正当前 joint YAML 的错误 key**
   独立 SDK 的 example config 必须使用正确的 `baud_rate`。

4. **如果保留 interpolation，就必须一起迁移 `AsyncInterpolator`**
   不能只在文档里写 `use_interpolation=True` 却没有实现。

5. **如果未来需要回接到 `lerobot`**
   应该让 `lerobot` 依赖这个 SDK，或做薄适配层，而不是继续维护两套并行实现。
