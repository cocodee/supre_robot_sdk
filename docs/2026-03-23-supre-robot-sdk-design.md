# Supre Robot SDK — Standalone Extraction Design（修正版）

**Date:** 2026-03-23  
**Status:** Revised after repository review  
**Target:** 新路径下的独立 Python SDK，不在 `lerobot` 仓库内实现

## Overview

目标仍然是把 Supre 机器人的硬件控制层抽成独立 SDK，落到新的项目路径，例如：

```text
~/projects/supre-robot-sdk/
```

但在审视当前仓库代码后，原设计需要修正：不能把 `SupreRobotFollower` 整体当作 SDK 直接抽走，也不能忽略 `SupreRobotLeader` 与共享硬件管理层的关系。独立 SDK 应抽取的是：

- 硬件抽象接口
- Eyou 电机驱动适配
- Jodell 夹爪驱动适配
- 多硬件聚合管理器
- 可选异步插值器
- 面向集成方的轻量 `SupreRobot` façade

而不应直接携带 LeRobot 专属适配层。

## Goals

- **Project location:** 新路径下的独立项目
- **Scope:** Hardware Manager + Joint Control + Gripper Control + optional interpolation
- **Deployment:** Standalone Python，不依赖 LeRobot 框架
- **Users:** 需要将 Supre 机器人接入其现有系统的集成方
- **API style:** 高层 `SupreRobot` + 低层 `HardwareManager`
- **Config:** YAML 驱动，支持双臂与多硬件组合

## Non-Goals

- 不保留 LeRobot 的 `Robot` / `Teleoperator` 抽象
- 不移植相机接入
- 不移植 Prometheus 指标
- 不移植 follower 的安全验证与相对位移限制
- 不移植 WebXR / ROS2 / HIL 逻辑
- 不在本阶段处理标定/归零流程

## Repository Review: 原设计的问题

### 1. 抽取源文件选错了

原设计将 `src/lerobot/robots/supre_robot_follower/supre_robot_follower.py` 当作 SDK `robot.py` 的主要来源，这是不成立的。该文件混合了：

- LeRobot `Robot` 接口适配
- cameras
- safety validator
- Prometheus
- action clamp
- trajectory helper

这些都不是独立 SDK 的核心能力。

### 2. 漏掉了 leader 与共享 manager 的关系

当前 `SupreRobotFollower` 和 `SupreRobotLeader` 都直接依赖 `SupreRobotHardwareManager`，并且都自行解析 joint config。若只从 follower 抽，会漏掉：

- 共享硬件生命周期
- joint config 解析共性
- gripper 语义与方向映射相关经验

### 3. Standalone SDK 必须彻底去掉 `lerobot` 依赖

现有实现存在直接依赖：

- `lerobot.robots.robot.Robot`
- `lerobot.teleoperators.teleoperator.Teleoperator`
- `lerobot.utils.*`
- `lerobot.cameras.*`

独立 SDK 不能继续依赖这些模块。原设计没有把“去依赖”作为第一等约束写清楚。

### 4. 配置现实比原设计复杂

从当前仓库看，配置并不统一：

- config dataclass 字段是 `joint_config_file`
- 多个 YAML 却写的是 `joint_config_path`
- joint config 中存在 `baud_rate"` 错别字
- follower / leader / trunk 有多份相近但不完全一致的 joint YAML

原设计只写了一个理想化 schema，没有覆盖这些真实迁移问题。

### 5. 插件架构描述过于理想化

当前代码并没有真正的自注册插件体系，只有固定映射：

```python
HARDWARE_TYPE_MAP = {
    "EyouMotorHardware": EyouMotorHardware,
    "JodellGripperHardware": JodellGripperHardware,
}
```

Standalone SDK 可以保留“可扩展”的设计，但第一版不应把复杂插件系统当成主任务。

### 6. 原测试计划没有验证“真正独立”

原测试只验证行为，没有验证：

- SDK 安装后是否可在无 `lerobot` 环境导入
- `eu_motor_py` / `jodell_gripper_py` 缺失时错误是否清晰
- 示例 config 是否真的可被 loader 接受

## Corrected Architecture

### Project Structure

```text
supre_robot_sdk/
├── src/
│   └── supre_robot_sdk/
│       ├── __init__.py
│       ├── core/
│       │   ├── __init__.py
│       │   ├── interfaces.py
│       │   ├── config.py
│       │   ├── hardware_manager.py
│       │   └── robot.py
│       ├── hardware/
│       │   ├── __init__.py
│       │   ├── base.py
│       │   ├── interpolation/
│       │   │   ├── __init__.py
│       │   │   └── async_interpolator.py
│       │   ├── eyou/
│       │   │   ├── __init__.py
│       │   │   └── motor.py
│       │   └── jodell/
│       │       ├── __init__.py
│       │       └── gripper.py
│       └── exceptions.py
├── examples/
│   └── robot_config.yaml
├── tests/
│   ├── test_config.py
│   ├── test_hardware_manager.py
│   ├── test_robot.py
│   ├── test_async_interpolator.py
│   ├── test_eyou_motor.py
│   └── test_jodell_gripper.py
├── pyproject.toml
└── README.md
```

## Design Principles

1. **Standalone first**: SDK 中不得导入 `lerobot.*`
2. **Port only the core**: 只迁移硬件控制核心，不迁移 LeRobot 适配层
3. **Typed config loader**: SDK 内部统一配置校验，而不是裸 dict 到处传
4. **Minimal extensibility**: 第一版用简单映射表即可，不强制做复杂插件系统
5. **Behavioral parity where it matters**: 读写、扭矩开关、插值行为尽量与现实现一致

## Core Interfaces

### HardwareInterface

```python
class HardwareInterface(ABC):
    def init(self, config: dict[str, Any]) -> bool: ...
    def activate(self) -> bool: ...
    def deactivate(self) -> None: ...
    def read(self) -> list[tuple[Optional[float], Optional[float]]]: ...
    def write(self, commands: list[float | None]) -> None: ...
    def get_joint_count(self) -> int: ...
    def set_enable_torque(self, enable: bool) -> None: ...
```

### SupreRobot

```python
class SupreRobot:
    def __init__(
        self,
        config_path: str | Path,
        *,
        control_frequency: float = 30.0,
        use_interpolation: bool = False,
    ) -> None: ...

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    @property
    def is_connected(self) -> bool: ...
    def get_joint_positions(self) -> dict[str, float]: ...
    def get_joint_forces(self) -> dict[str, float]: ...
    def send_joint_positions(self, positions: dict[str, float]) -> None: ...
    def set_enable_torque(self, enable: bool) -> None: ...
    def move_joint(self, joint_name: str, target: float) -> None: ...
    def move_joints(self, targets: dict[str, float]) -> None: ...
    def open_gripper(self, arm: str) -> None: ...
    def close_gripper(self, arm: str) -> None: ...
    def execute_trajectory(self, goal_positions: dict[str, float], duration: float) -> None: ...
```

## Hardware Manager

`HardwareManager` 是独立 SDK 的核心聚合层，主要来自当前：

- `src/lerobot/robots/supre_robot/supre_robot_hardware_manager.py`

但应做以下修正：

1. 不直接在构造函数里读裸 YAML 后散落使用，而是先走 typed config loader
2. 不再依赖 `print` + `False` 作为主要错误处理
3. 保留 interpolation wrapper，但把它作为 SDK 内部能力一起移植
4. 对 joint mapping 做显式校验：
   - 重复 joint
   - 未映射 joint
   - 未知 hardware type
   - 命令长度不匹配

## Async Interpolator

原设计遗漏了当前真实存在的插值实现：

- `src/lerobot/motors/eyou/async_interpolator.py`

由于当前 manager 已支持在 `use_interpolation=True` 时包装 hardware instance，独立 SDK 若希望保持行为一致，应一并移植该模块。否则需要在设计中明确“外部 SDK 第一版不支持 interpolation”。

本修正版选择：

- **移植 `AsyncInterpolator`**
- 放入 `hardware/interpolation/async_interpolator.py`
- 作为可选包装器使用

## Hardware Plugins

### EyouMotorHardware

来源：

- `src/lerobot/motors/eyou/eyou_motor_hardware.py`
- `src/lerobot/motors/eyou/hardware_interface.py`

需要修正：

- 去掉 `lerobot` 依赖
- 清理演示 `__main__` 代码
- 保留 `start_enabled`
- 保留 `set_enable_torque()` 中“启用前同步当前位置”的安全逻辑
- 对 `eu_motor_py` 缺失给出清晰异常

### JodellGripperHardware

来源：

- `src/lerobot/motors/gripper/jodell_gripper_hardware.py`

需要修正：

- 去掉 `lerobot` 依赖
- 保留缓存读取机制
- 保留 `None` command 语义
- 对 `jodell_gripper_py` 缺失给出清晰异常

## Configuration Schema

### Canonical Example

```yaml
joint_order:
  - left_arm_joint_1
  - left_arm_joint_2
  - left_arm_joint_3
  - left_arm_joint_4
  - left_arm_joint_5
  - left_arm_joint_6
  - left_arm_joint_7
  - right_arm_joint_1
  - right_arm_joint_2
  - right_arm_joint_3
  - right_arm_joint_4
  - right_arm_joint_5
  - right_arm_joint_6
  - right_arm_joint_7

hardware_interfaces:
  - name: arm_motors
    type: EyouMotorHardware
    interpolation:
      interpolation_n: 3
    config:
      can_device_index: 1
      can_baud_rate: "1M"
      joints:
        - { name: left_arm_joint_1, parameters: { node_id: 21 } }
        - { name: left_arm_joint_2, parameters: { node_id: 22 } }
        - { name: left_arm_joint_3, parameters: { node_id: 23 } }
        - { name: left_arm_joint_4, parameters: { node_id: 24 } }
        - { name: left_arm_joint_5, parameters: { node_id: 25 } }
        - { name: left_arm_joint_6, parameters: { node_id: 26 } }
        - { name: right_arm_joint_1, parameters: { node_id: 11 } }
        - { name: right_arm_joint_2, parameters: { node_id: 12 } }
        - { name: right_arm_joint_3, parameters: { node_id: 13 } }
        - { name: right_arm_joint_4, parameters: { node_id: 14 } }
        - { name: right_arm_joint_5, parameters: { node_id: 15 } }
        - { name: right_arm_joint_6, parameters: { node_id: 16 } }

  - name: left_gripper
    type: JodellGripperHardware
    interpolation:
      interpolation_n: 1
    config:
      device: "/dev/ttyTHS1"
      baud_rate: 115200
      default_speed_percent: 100
      default_torque_percent: 100
      joints:
        - name: left_arm_joint_7
          parameters: { slave_id: 27 }

  - name: right_gripper
    type: JodellGripperHardware
    interpolation:
      interpolation_n: 1
    config:
      device: "/dev/ttyTHS2"
      baud_rate: 115200
      default_speed_percent: 100
      default_torque_percent: 100
      joints:
        - name: right_arm_joint_7
          parameters: { slave_id: 17 }
```

### Required Config Fixes From Current Repo

从当前仓库抽取时必须显式修正：

- `baud_rate"` -> `baud_rate`
- 不再使用 `joint_config_path` / `joint_config_file` 这类 LeRobot adapter 字段
- 独立 SDK 只接受一个 `robot_config.yaml` 路径作为入口

## Source Files Mapping

### Primary extraction sources

| Current repo path | SDK target path | Notes |
|---|---|---|
| `src/lerobot/robots/supre_robot/supre_robot_hardware_manager.py` | `src/supre_robot_sdk/core/hardware_manager.py` | 主体来源，需要做 typed config 和异常处理清理 |
| `src/lerobot/motors/eyou/hardware_interface.py` | `src/supre_robot_sdk/hardware/base.py` | 迁移为通用硬件接口 |
| `src/lerobot/motors/eyou/async_interpolator.py` | `src/supre_robot_sdk/hardware/interpolation/async_interpolator.py` | 原设计遗漏，修正版纳入 |
| `src/lerobot/motors/eyou/eyou_motor_hardware.py` | `src/supre_robot_sdk/hardware/eyou/motor.py` | 去掉 `lerobot` 依赖 |
| `src/lerobot/motors/gripper/jodell_gripper_hardware.py` | `src/supre_robot_sdk/hardware/jodell/gripper.py` | 去掉 `lerobot` 依赖 |

### Reference-only sources

这些文件只作为行为参考，不应直接整体迁移：

| Current repo path | Why reference-only |
|---|---|
| `src/lerobot/robots/supre_robot_follower/supre_robot_follower.py` | 含 `Robot` 适配、安全、相机、监控 |
| `src/lerobot/teleoperators/supre_robot_leader/supre_robot_leader.py` | 含 `Teleoperator` 适配与方向映射语义 |
| `src/lerobot/robots/supre_robot_follower/supre_robot_follower_config.py` | 是 LeRobot config，不是 SDK config |
| `src/lerobot/teleoperators/supre_robot_leader/supre_robot_leader_config.py` | 是 LeRobot config，不是 SDK config |

## Testing Strategy

测试必须证明这个包是“真的独立”，不是“从仓库里复制出来但仍偷偷依赖 `lerobot`”。

- `test_config.py`
  - YAML 解析
  - 重复 joint 检测
  - 未映射 joint 检测
  - 错误 hardware type 检测
- `test_hardware_manager.py`
  - mock hardware instance，验证 read/write 聚合逻辑
- `test_async_interpolator.py`
  - interpolation_n 开关逻辑
  - pass-through 行为
- `test_eyou_motor.py`
  - mock `eu_motor_py`
  - init/activate/read/write/torque
- `test_jodell_gripper.py`
  - mock `jodell_gripper_py`
  - cache/read/write/disable
- `test_robot.py`
  - 高层 façade API
  - `move_joint` / gripper convenience
  - trajectory helper
- `test_no_lerobot_dependency.py`
  - 确认 SDK 导入不要求存在 `lerobot`

## Recommended Delivery

### Phase 1

- 在新路径创建独立 SDK 项目
- 迁移并清理底层硬件与 manager
- 建立 typed config loader
- 写全 mock 测试

### Phase 2

- 增加 README、示例配置、安装说明
- 在目标机器上做最小真实硬件验证

### Phase 3

- 如有需要，在 `lerobot` 中反向接入该 SDK，作为其下游依赖，而不是继续内嵌一份重复实现

## Summary

修正后的设计仍然支持“在新路径创建独立 SDK”，但修正了原方案的几个关键误判：

- 不再把 follower 整体当作 SDK 核心
- 显式纳入共享 manager 与 async interpolator
- 去掉对 `lerobot` 框架的任何依赖
- 把当前仓库真实存在的配置问题和迁移风险写进设计

这样抽出的 SDK 才是可安装、可测试、可独立运行的，而不是 LeRobot 实现的另一份拷贝。
