import datetime
from types import SimpleNamespace

import pytest

import supre_robot_sdk.hardware.eyou.motor as motor_module
from supre_robot_sdk.exceptions import DependencyUnavailableError
from supre_robot_sdk.hardware.eyou import EyouMotorHardware


class FakeFeedback:
    def __init__(self):
        self.last_update_time = datetime.timedelta(milliseconds=1)
        self.position_deg = 12.5
        self.velocity_dps = 1.2
        self.torque_milli = 345.0
        self.status_word = 0
        self.error_code = 0
        self.in_fault = False


class FakeMotorNode:
    def __init__(self, can_device_index, node_id):
        self.can_device_index = can_device_index
        self.node_id = node_id
        self.disabled = False
        self.position = float(node_id)
        self.commands = []
        self.torque_commands = []
        self.csp_configs = []
        self.cst_configs = []
        self.status_word = 0
        self.error_code = 0
        self.operation_mode = 8

    def get_position(self):
        return self.position

    def get_velocity(self):
        return 0.0

    def get_torque(self):
        return 0.5

    def get_status_word(self):
        return self.status_word

    def get_error_code(self):
        return self.error_code

    def get_operation_mode(self):
        return self.operation_mode

    def clear_fault(self):
        return True

    def configure_csp_mode(self, *args):
        self.csp_configs.append(args)
        return True

    def configure_cst_mode(self, *args):
        self.cst_configs.append(args)
        return True

    def start_auto_feedback(self, *_args):
        return True

    def start_error_feedback_tpdo(self, *_args):
        return True

    def get_latest_feedback(self):
        feedback = FakeFeedback()
        feedback.status_word = self.status_word
        feedback.error_code = self.error_code
        feedback.in_fault = bool(self.status_word & 0x0008)
        return feedback

    def read_u8(self, index, sub_index):
        if (index, sub_index) == (0x1003, 0):
            return 0
        raise RuntimeError("unexpected read")

    def read_u32(self, index, sub_index):
        raise RuntimeError("unexpected read")

    def send_csp_target_position(self, position, *_args):
        self.commands.append(position)
        return 0

    def send_cst_target_torque(self, torque, *args):
        self.torque_commands.append((torque, *args))

    def disable(self):
        self.disabled = True


class FakeCanManager:
    def init_device(self, *_args):
        return True


class FakeFeedbackManager:
    @staticmethod
    def get_instance():
        return FakeFeedbackManager()

    def register_callback(self):
        return True


def build_fake_module():
    return SimpleNamespace(
        Baudrate=SimpleNamespace(BPS_1M=1, BPS_500K=2, BPS_250K=3),
        DeviceType=SimpleNamespace(Canable=0),
        CanNetworkManager=FakeCanManager,
        EuMotorNode=FakeMotorNode,
        MotorFeedbackManager=FakeFeedbackManager,
    )


def test_eyou_requires_native_binding(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", None)
    with pytest.raises(DependencyUnavailableError):
        EyouMotorHardware().init({"can_device_index": 1, "can_baud_rate": "1M", "joints": []})


def test_eyou_init_activate_read_write(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [
                {"name": "joint_1", "parameters": {"node_id": 21}},
                {"name": "joint_2", "parameters": {"node_id": 22, "start_enabled": False}},
            ],
        }
    )
    assert hardware.activate() is True
    assert hardware.get_joint_count() == 2
    assert hardware.get_control_mode() == "position"
    assert hardware.read()[0][0] == 12.5
    hardware.write([1.0, 2.0])
    assert hardware.motor_nodes_[0].commands == [1.0]
    assert hardware.motor_nodes_[1].commands == []
    hardware.set_enable_torque(True)
    assert hardware.hw_start_enabled_ == [True, True]


def test_eyou_torque_control(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [
                {"name": "joint_1", "parameters": {"node_id": 21}},
                {"name": "joint_2", "parameters": {"node_id": 22, "start_enabled": False}},
            ],
        }
    )
    assert hardware.activate() is True
    assert hardware.supports_torque_control() is True

    hardware.configure_torque_control(interpolation_period_ms=4, use_sync=True)
    assert hardware.get_control_mode() == "torque"
    assert hardware.motor_nodes_[0].cst_configs == [(4, 0, True)]
    assert hardware.motor_nodes_[1].cst_configs == []

    hardware.write_torques([50.4, -20.0])
    assert hardware.motor_nodes_[0].torque_commands == [(50, 0, True)]
    assert hardware.motor_nodes_[1].torque_commands == []

    with pytest.raises(ValueError, match="length"):
        hardware.write_torques([1.0])
    with pytest.raises(ValueError, match="dense"):
        hardware.write_torques([1.0, None])


def test_eyou_activate_can_start_in_torque_control_mode(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "control_mode": "torque",
            "torque_interpolation_period_ms": 8,
            "torque_use_sync": False,
            "joints": [
                {"name": "joint_1", "parameters": {"node_id": 21}},
                {"name": "joint_2", "parameters": {"node_id": 22, "start_enabled": False}},
            ],
        }
    )

    assert hardware.activate() is True
    assert hardware.get_control_mode() == "torque"
    assert hardware.motor_nodes_[0].cst_configs == [(8, 0, False)]
    assert hardware.motor_nodes_[0].csp_configs == []
    assert hardware.motor_nodes_[1].cst_configs == []


def test_eyou_set_enable_torque_preserves_torque_control_mode(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [
                {"name": "joint_1", "parameters": {"node_id": 21}},
                {"name": "joint_2", "parameters": {"node_id": 22, "start_enabled": False}},
            ],
        }
    )
    assert hardware.activate() is True
    hardware.configure_torque_control(interpolation_period_ms=8, use_sync=False)

    hardware.set_enable_torque(False)
    hardware.set_enable_torque(True)

    assert hardware.motor_nodes_[0].cst_configs[-1] == (8, 0, False)
    assert hardware.motor_nodes_[1].cst_configs[-1] == (8, 0, False)
    assert hardware.hw_start_enabled_ == [True, True]


def test_eyou_diagnose_reports_fallback_status(monkeypatch):
    monkeypatch.setattr(motor_module, "eu_motor_py", build_fake_module())
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [{"name": "joint_1", "parameters": {"node_id": 21}}],
        }
    )
    hardware.motor_nodes_[0].status_word = 0x0008
    hardware.motor_nodes_[0].error_code = 0x2310

    diagnostics = hardware.diagnose()

    assert diagnostics["ok"] is False
    assert diagnostics["joints"][0]["joint_name"] == "joint_1"
    assert diagnostics["joints"][0]["node_id"] == 21
    assert diagnostics["joints"][0]["in_fault"] is True
    assert diagnostics["joints"][0]["error_code"] == 0x2310
    assert "fault state" in diagnostics["joints"][0]["message"]


def test_eyou_diagnose_uses_native_diagnostics(monkeypatch):
    class NativeDiagnosticMotor(FakeMotorNode):
        def get_diagnostics(self):
            return SimpleNamespace(
                status_word=0,
                error_code=0,
                in_fault=False,
                operation_mode=8,
                latest_feedback_age_ms=12.0,
                error_history=[0x1111],
                last_emcy=SimpleNamespace(
                    node_id=self.node_id,
                    error_code=0x2222,
                    error_register=1,
                    manufacturer_specific=(1, 2, 3, 4, 5),
                ),
                warnings=["history warning"],
            )

    fake_module = build_fake_module()
    fake_module.EuMotorNode = NativeDiagnosticMotor
    monkeypatch.setattr(motor_module, "eu_motor_py", fake_module)
    hardware = EyouMotorHardware()
    assert hardware.init(
        {
            "can_device_index": 1,
            "can_baud_rate": "1M",
            "joints": [{"name": "joint_1", "parameters": {"node_id": 21}}],
        }
    )

    diagnostics = hardware.diagnose()

    assert diagnostics["ok"] is True
    joint = diagnostics["joints"][0]
    assert joint["error_history"] == [0x1111]
    assert joint["last_emcy"]["error_code"] == 0x2222
    assert joint["warnings"] == ["history warning"]


def test_eyou_diagnose_keeps_init_failure_reason(monkeypatch):
    class FailingCanManager:
        def init_device(self, *_args):
            raise RuntimeError("CAN adapter missing")

    fake_module = build_fake_module()
    fake_module.CanNetworkManager = FailingCanManager
    monkeypatch.setattr(motor_module, "eu_motor_py", fake_module)
    hardware = EyouMotorHardware()

    assert (
        hardware.init(
            {
                "can_device_index": 1,
                "can_baud_rate": "1M",
                "joints": [{"name": "joint_1", "parameters": {"node_id": 21}}],
            }
        )
        is False
    )

    diagnostics = hardware.diagnose()
    assert diagnostics["ok"] is False
    assert diagnostics["events"][0]["stage"] == "init"
    assert "CAN adapter missing" in diagnostics["message"]
