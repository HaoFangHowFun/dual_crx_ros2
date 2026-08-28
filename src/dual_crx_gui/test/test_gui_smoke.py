import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtCore, QtWidgets
from dual_crx_interfaces.msg import SystemState
from sensor_msgs.msg import JointState
from dual_crx_gui.app import LEFT, MainWindow


class Signal(QtCore.QObject):
    emitted = QtCore.pyqtSignal(object)


class FakeApi:
    def __init__(self):
        self._state_signal = Signal()
        self._result_signal = Signal()
        self.state_received = self._state_signal.emitted
        self.result_received = self._result_signal.emitted
        self.calls = []
        self.state = None
        self.plan_id = "p1"
        self.cartesian_plan_id = "cp1"
    def acquire(self, scope): self.calls.append(("acquire", scope))
    def jog(self, *args): self.calls.append(("jog",) + args)
    def cartesian_jog(self, *args): self.calls.append(("cartesian_jog",) + args)
    def position(self, *args, **kwargs): self.calls.append(("position", args, kwargs))
    def cartesian_pose(self, *args, **kwargs): self.calls.append(("cartesian", args, kwargs))
    def cancel(self): self.calls.append(("cancel",))
    def stop(self, reason="GUI stop"): self.calls.append(("stop", reason))
    def release(self): self.calls.append(("release",))
    def close(self): self.calls.append(("close",))


def test_headless_state_copy_jog_and_physical_disable():
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
    msg.operating_mode = "physical"; msg.control_state = SystemState.READ_ONLY
    window.update_state(msg)
    assert not window.targets[0][1].isEnabled()
    window.deleteLater(); app.processEvents()
