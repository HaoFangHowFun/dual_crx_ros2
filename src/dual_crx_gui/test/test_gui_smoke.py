import os
os.environ.setdefault("QT_QPA_PLATFORM", "minimal")

from types import SimpleNamespace

from PyQt5 import QtCore, QtWidgets
from dual_crx_interfaces.msg import SystemState
from sensor_msgs.msg import JointState
from dual_crx_gui.app import LEFT, MainWindow, RosApi


class Signal(QtCore.QObject):
    emitted = QtCore.pyqtSignal(object)


class RobotStatusSignal(QtCore.QObject):
    emitted = QtCore.pyqtSignal(int, object)


class BoolTextSignal(QtCore.QObject):
    emitted = QtCore.pyqtSignal(bool, str)


class FakeApi:
    def __init__(self):
        self._state_signal = Signal()
        self._robot_status_signal = RobotStatusSignal()
        self._result_signal = Signal()
        self._workcell_signal = Signal()
        self._calibration_feedback_signal = Signal()
        self._calibration_result_signal = BoolTextSignal()
        self.state_received = self._state_signal.emitted
        self.robot_status_received = self._robot_status_signal.emitted
        self.result_received = self._result_signal.emitted
        self.workcell_received = self._workcell_signal.emitted
        self.calibration_feedback_received = self._calibration_feedback_signal.emitted
        self.calibration_result_received = self._calibration_result_signal.emitted
        self.calls = []
        self.state = None
        self.plan_id = "p1"
        self.cartesian_plan_id = "cp1"
    def acquire(self, scope, callback=None):
        self.calls.append(("acquire", scope))
        if callback:
            callback(True, "acquired")
    def jog(self, *args): self.calls.append(("jog",) + args)
    def cartesian_jog(self, *args): self.calls.append(("cartesian_jog",) + args)
    def position(self, *args, **kwargs): self.calls.append(("position", args, kwargs))
    def cartesian_pose(self, *args, **kwargs):
        self.calls.append(("cartesian", args, kwargs))
        if len(args) >= 4:
            self.cartesian_plan_id = "cp1"
    def cancel(self): self.calls.append(("cancel",))
    def stop(self, reason="GUI stop"): self.calls.append(("stop", reason))
    def switch_motion(self, arm, enable): self.calls.append(("switch_motion", arm, enable))
    def request_workcell_info(self): self.calls.append(("workcell_info",))
    def tcp_pose(self, frame, _tcp):
        pose = self.state.left_end_effector
        pose.header.frame_id = frame
        return pose
    def transform_pose(self, pose, frame):
        pose.header.frame_id = frame
        return pose
    def calibration_check(self, *args): self.calls.append(("calibration_check",) + args)
    def cancel_calibration_check(self): self.calls.append(("cancel_calibration_check",))
    def release(self): self.calls.append(("release",))
    def close(self): self.calls.append(("close",))


def test_quaternion_frame_transform_rotates_cartesian_axes():
    half = 2 ** -0.5

    rotated = RosApi._rotate_vector((0.0, 0.0, half, half), (1.0, 0.0, 0.0))

    assert abs(rotated[0]) < 1e-12
    assert abs(rotated[1] - 1.0) < 1e-12
    assert abs(rotated[2]) < 1e-12


def test_headless_state_copy_jog_and_physical_state_gates_widgets():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    api = FakeApi(); window = MainWindow(api); window.show(); app.processEvents()
    msg = SystemState(operating_mode="mock", control_state=SystemState.READY,
                      complete=True, fresh=True, left_controller_state="active",
                      right_controller_state="active")
    msg.left_joints = JointState(name=[f"left_J{i}" for i in range(1, 7)],
                                 position=[0.1] * 6, velocity=[0.0] * 6)
    msg.right_joints = JointState(name=[f"right_J{i}" for i in range(1, 7)],
                                  position=[0.2] * 6, velocity=[0.0] * 6)
    api.state = msg; window.update_state(msg); window.copy_current()
    assert abs(window.targets[0][1].value() - 5.73) < 0.01
    window.start_jog(0, 1); window._send_jog(); window.stop_jog()
    assert any(call[0] == "jog" and call[1] == LEFT for call in api.calls)
    assert any(call[0] == "stop" for call in api.calls)
    msg.left_end_effector.header.frame_id = "world"
    msg.left_end_effector.pose.position.x = 0.42
    msg.left_end_effector.pose.orientation.w = 1.0
    window.copy_current_pose(); window.plan_cartesian(); window.execute_cartesian()
    assert abs(window.cart_fields["x"].value() - 0.42) < 1e-6
    assert sum(call[0] == "cartesian" for call in api.calls) == 2
    window.start_cartesian_jog("Z", 1); window._send_cartesian_jog()
    window.stop_cartesian_jog()
    assert any(call[0] == "cartesian_jog" for call in api.calls)
    window.cart_frame.setCurrentText("Robot Base")
    window.copy_current_pose(); window.plan_cartesian()
    cartesian_calls = [call for call in api.calls if call[0] == "cartesian"]
    assert cartesian_calls[-1][1][3].header.frame_id == "left_base_link"
    window.start_cartesian_jog("X", 1); window._send_cartesian_jog()
    window.stop_cartesian_jog()
    jog_calls = [call for call in api.calls if call[0] == "cartesian_jog"]
    assert jog_calls[-1][3] == "left_base_link"
    window.cart_jog_frame.setCurrentText("Tool / TCP")
    window.start_cartesian_jog("Z", 1); window._send_cartesian_jog()
    window.stop_cartesian_jog()
    jog_calls = [call for call in api.calls if call[0] == "cartesian_jog"]
    assert jog_calls[-1][3] == "left_flange"
    info = SimpleNamespace(
        loaded=True, valid=True, world_frame="world", table_frame="table_frame",
        placement_file="/tmp/robot_placement_physical.yaml",
        profile_name="test-profile", generated_at="2026-09-02",
        source_calibration="", sha256="a" * 64,
        left_eef_z_offset_m=-0.035, right_eef_z_offset_m=-0.035, reason="loaded")
    window.update_workcell_info(info)
    assert window.workcell_labels["Profile"].text() == "test-profile"
    window.start_calibration_check(True)
    assert any(call[0] == "calibration_check" for call in api.calls)
    assert window.check_clearance.minimum() == 15.0
    window.calibration_check_finished(True, "planned")
    assert window.check_progress.value() == 100
    msg.operating_mode = "physical"; msg.control_state = SystemState.READ_ONLY
    window.update_state(msg)
    assert not window.targets[0][1].isEnabled()
    msg.control_state = SystemState.READY
    window.update_state(msg)
    assert window.targets[0][1].isEnabled()
    window.deleteLater(); app.processEvents()


def test_cartesian_rpy_degrees_copy_and_command():
    import math
    import pytest
    from geometry_msgs.msg import PoseStamped

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow(FakeApi())
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.orientation.z = math.sqrt(0.5)
    pose.pose.orientation.w = math.sqrt(0.5)
    window._populate_cartesian_fields(pose)
    assert window.cart_fields["yaw"].value() == pytest.approx(90.0)
    assert window.cart_fields["roll"].value() == pytest.approx(0.0)
    assert window.cart_fields["pitch"].value() == pytest.approx(0.0)
    window.cart_fields["yaw"].setValue(0.0)
    window.cart_fields["roll"].setValue(180.0)
    window.plan_cartesian()
    call = next(call for call in window.api.calls if call[0] == "cartesian")
    q = call[1][3].pose.orientation
    assert (q.x, q.y, q.z, q.w) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-12)
    window.deleteLater()
    app.processEvents()


def test_rpy_round_trip_preserves_mixed_and_singular_rotations():
    import pytest
    from geometry_msgs.msg import Quaternion
    from dual_crx_gui.app import quaternion_to_rpy_degrees, rpy_degrees_to_quaternion

    for angles in ((30, 45, -60), (25, 90, 70), (25, -90, 70), (0, 89.999, 180)):
        original = rpy_degrees_to_quaternion(*angles)
        q = Quaternion(x=original[0], y=original[1], z=original[2], w=original[3])
        restored = rpy_degrees_to_quaternion(*quaternion_to_rpy_degrees(q))
        assert abs(sum(a*b for a, b in zip(original, restored))) == pytest.approx(1.0, abs=1e-10)
