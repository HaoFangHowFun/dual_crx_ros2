import math
import os
import sys
import uuid

from PyQt5 import QtCore, QtWidgets
import rclpy
from control_msgs.msg import JointJog
from dual_crx_interfaces.action import JointPosition
from dual_crx_interfaces.action import CartesianPose
from dual_crx_interfaces.msg import SystemState
from dual_crx_interfaces.srv import (AcquireControl, CartesianJog, Heartbeat,
                                     JointJog as JointJogSrv)
from dual_crx_interfaces.srv import ReleaseControl, SoftwareStop
from rclpy.action import ActionClient
from rclpy.node import Node

LEFT, RIGHT, BOTH = 1, 2, 3


class RosApi(QtCore.QObject):
    state_received = QtCore.pyqtSignal(object)
    result_received = QtCore.pyqtSignal(str)

    def __init__(self, client_id):
        super().__init__()
        self.client_id = client_id
        self.node = Node("dual_crx_gui_client")
        self.state = None
        self.scope = LEFT
        self.plan_id = ""
        self.cartesian_plan_id = ""
        self.goal_handle = None
        self.node.create_subscription(SystemState, "/dual_crx/state", self._state, 10)
        self.acquire_client = self.node.create_client(AcquireControl, "/dual_crx/acquire_control")
        self.release_client = self.node.create_client(ReleaseControl, "/dual_crx/release_control")
        self.heartbeat_client = self.node.create_client(Heartbeat, "/dual_crx/heartbeat")
        self.jog_client = self.node.create_client(JointJogSrv, "/dual_crx/jog")
        self.cartesian_jog_client = self.node.create_client(
            CartesianJog, "/dual_crx/cartesian_jog")
        self.stop_client = self.node.create_client(SoftwareStop, "/dual_crx/stop")
        self.position_client = ActionClient(self.node, JointPosition, "/dual_crx/joint_position")
        self.cartesian_client = ActionClient(self.node, CartesianPose, "/dual_crx/cartesian_pose")

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _state(self, msg):
        self.state = msg
        self.state_received.emit(msg)

    def acquire(self, scope):
        self.scope = scope
        req = AcquireControl.Request(client_id=self.client_id, source_type="GUI",
                                     arm_scope=scope, requested_mode="JOINT", lease_duration=1.0)
        self.acquire_client.call_async(req)

    def heartbeat(self):
        if self.scope:
            self.heartbeat_client.call_async(Heartbeat.Request(
                client_id=self.client_id, arm_scope=self.scope, lease_duration=1.0))

    def release(self):
        if self.scope:
            self.release_client.call_async(ReleaseControl.Request(
                client_id=self.client_id, arm_scope=self.scope))

    def jog(self, scope, joint, velocity):
        command = JointJog()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.joint_names = [joint]
        command.velocities = [velocity]
        self.jog_client.call_async(JointJogSrv.Request(
            client_id=self.client_id, arm_scope=scope, command=command))

    def cartesian_jog(self, scope, tcp_link, linear, angular):
        from geometry_msgs.msg import TwistStamped
        command = TwistStamped(); command.header.frame_id = "world"
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = linear
        command.twist.angular.x, command.twist.angular.y, command.twist.angular.z = angular
        self.cartesian_jog_client.call_async(CartesianJog.Request(
            client_id=self.client_id, arm_scope=scope, tcp_link=tcp_link,
            command=command))

    def position(self, operation, group, names=None, positions=None, plan_id=""):
        goal = JointPosition.Goal(client_id=self.client_id, planning_group=group,
                                  operation=operation, joint_names=names or [],
                                  positions=positions or [], plan_id=plan_id)
        future = self.position_client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def cartesian_pose(self, operation, scope, tcp_link, target=None, plan_id=""):
        goal = CartesianPose.Goal(client_id=self.client_id, arm_scope=scope,
                                  operation=operation, tcp_link=tcp_link,
                                  plan_id=plan_id, position_tolerance=0.002,
                                  orientation_tolerance=0.01)
        if target is not None:
            goal.target = target
            goal.target.header.stamp = self.node.get_clock().now().to_msg()
        future = self.cartesian_client.send_goal_async(
            goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, feedback):
        self.result_received.emit(f"{feedback.feedback.phase}: {feedback.feedback.detail}")

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.result_received.emit("command rejected")
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._result)

    def _result(self, future):
        result = future.result().result
        if result.plan_id:
            self.plan_id = result.plan_id
            if result.reason.startswith("Cartesian"):
                self.cartesian_plan_id = result.plan_id
        self.result_received.emit(result.reason)

    def cancel(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def stop(self, reason="GUI stop"):
        self.stop_client.call_async(SoftwareStop.Request(
            client_id=self.client_id, arm_scope=BOTH, reason=reason))

    def close(self):
        self.stop("GUI close/focus loss")
        self.release()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, api):
        super().__init__()
        self.api = api
        self.setWindowTitle("Dual CRX Unified Control")
        self.resize(1050, 720)
        self._jog_request = None
        self._jog_timer = QtCore.QTimer(self)
        self._jog_timer.setInterval(100)
        self._jog_timer.timeout.connect(self._send_jog)
        self._cart_jog_request = None
        self._cart_jog_timer = QtCore.QTimer(self)
        self._cart_jog_timer.setInterval(50)
        self._cart_jog_timer.timeout.connect(self._send_cartesian_jog)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)

        status = QtWidgets.QGridLayout()
        self.labels = {}
        for i, name in enumerate(("Connection", "Mode", "Control state", "Ownership",
                                  "Controllers", "Freshness", "Active command", "Reason")):
            status.addWidget(QtWidgets.QLabel(name + ":"), i // 2, (i % 2) * 2)
            self.labels[name] = QtWidgets.QLabel("—")
            status.addWidget(self.labels[name], i // 2, (i % 2) * 2 + 1)
        layout.addLayout(status)
        self.stop_button = QtWidgets.QPushButton("STOP")
        self.stop_button.setStyleSheet("font-size: 22px; font-weight: bold; color: white; background: #b00020")
        self.stop_button.clicked.connect(lambda: self.api.stop())
        layout.addWidget(self.stop_button)

        arms = QtWidgets.QHBoxLayout()
        self.joint_labels = {LEFT: [], RIGHT: []}
        for arm, title in ((LEFT, "Left arm"), (RIGHT, "Right arm")):
            box = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout(box)
            grid.addWidget(QtWidgets.QLabel("Joint"), 0, 0)
            grid.addWidget(QtWidgets.QLabel("Position (deg)"), 0, 1)
            grid.addWidget(QtWidgets.QLabel("Velocity (deg/s)"), 0, 2)
            for j in range(6):
                grid.addWidget(QtWidgets.QLabel(f"J{j+1}"), j + 1, 0)
                pos, vel = QtWidgets.QLabel("—"), QtWidgets.QLabel("—")
                grid.addWidget(pos, j + 1, 1); grid.addWidget(vel, j + 1, 2)
                self.joint_labels[arm].append((pos, vel))
            arms.addWidget(box)
        layout.addLayout(arms)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self._jog_panel(), "Jog")
        tabs.addTab(self._position_panel(), "Joint position")
        tabs.addTab(self._cartesian_panel(), "Cartesian pose")
        layout.addWidget(tabs)
        self.api.state_received.connect(self.update_state)
        self.api.result_received.connect(self.command_result.setText)

    def _jog_panel(self):
        panel = QtWidgets.QWidget(); grid = QtWidgets.QGridLayout(panel)
        self.jog_arm = QtWidgets.QComboBox(); self.jog_arm.addItems(["left", "right"])
        self.jog_speed = QtWidgets.QComboBox(); self.jog_speed.addItems(["1", "2", "5", "10"])
        grid.addWidget(QtWidgets.QLabel("Arm"), 0, 0); grid.addWidget(self.jog_arm, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Speed (deg/s)"), 0, 2); grid.addWidget(self.jog_speed, 0, 3)
        self.jog_deadman = QtWidgets.QLabel("Deadman: released")
        grid.addWidget(self.jog_deadman, 0, 4)
        for j in range(6):
            grid.addWidget(QtWidgets.QLabel(f"J{j+1}"), j + 1, 0)
            for col, sign in ((1, -1), (2, 1)):
                button = QtWidgets.QPushButton("−" if sign < 0 else "+")
                button.pressed.connect(lambda j=j, sign=sign: self.start_jog(j, sign))
                button.released.connect(self.stop_jog)
                grid.addWidget(button, j + 1, col)
        return panel

    def _position_panel(self):
        panel = QtWidgets.QWidget(); grid = QtWidgets.QGridLayout(panel)
        self.group = QtWidgets.QComboBox(); self.group.addItems(["left_arm", "right_arm", "both_arms"])
        self.group.currentTextChanged.connect(self._targets_for_group)
        grid.addWidget(QtWidgets.QLabel("Planning group"), 0, 0); grid.addWidget(self.group, 0, 1)
        self.targets = []
        for i in range(12):
            label = QtWidgets.QLabel(f"J{i+1}")
            field = QtWidgets.QDoubleSpinBox(); field.setRange(-360.0, 360.0); field.setDecimals(3)
            grid.addWidget(label, 1 + i // 6, (i % 6) * 2)
            grid.addWidget(field, 1 + i // 6, (i % 6) * 2 + 1)
            self.targets.append((label, field))
        buttons = QtWidgets.QHBoxLayout()
        for text, callback in (("Copy current", self.copy_current), ("Validate", self.validate),
                               ("Plan", self.plan), ("Execute", self.execute),
                               ("Cancel", self.api.cancel)):
            b = QtWidgets.QPushButton(text); b.clicked.connect(callback); buttons.addWidget(b)
        grid.addLayout(buttons, 4, 0, 1, 12)
        self.command_result = QtWidgets.QLabel("No command")
        grid.addWidget(self.command_result, 5, 0, 1, 12)
        self.command_widgets = [field for _, field in self.targets] + [self.group]
        self._targets_for_group()
        return panel

    def _cartesian_panel(self):
        panel = QtWidgets.QWidget(); grid = QtWidgets.QGridLayout(panel)
        self.cart_arm = QtWidgets.QComboBox(); self.cart_arm.addItems(["left", "right"])
        grid.addWidget(QtWidgets.QLabel("Arm / TCP"), 0, 0); grid.addWidget(self.cart_arm, 0, 1)
        self.cart_fields = {}
        defaults = {"x": 0.0, "y": 0.0, "z": 0.0,
                    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        for i, (name, value) in enumerate(defaults.items()):
            grid.addWidget(QtWidgets.QLabel(name), 1, i)
            field = QtWidgets.QDoubleSpinBox(); field.setRange(-5.0, 5.0)
            field.setDecimals(6); field.setSingleStep(0.001); field.setValue(value)
            grid.addWidget(field, 2, i); self.cart_fields[name] = field
        buttons = QtWidgets.QHBoxLayout()
        for text, callback in (("Copy current TCP", self.copy_current_pose),
                               ("Validate", self.validate_cartesian),
                               ("Plan", self.plan_cartesian),
                               ("Execute", self.execute_cartesian),
                               ("Cancel", self.api.cancel)):
            button = QtWidgets.QPushButton(text); button.clicked.connect(callback)
            buttons.addWidget(button)
        grid.addLayout(buttons, 3, 0, 1, 7)
        self.cart_result = QtWidgets.QLabel("No Cartesian command")
        grid.addWidget(self.cart_result, 4, 0, 1, 7)
        jog_box = QtWidgets.QGroupBox("MoveIt Servo jog (press and hold, world frame)")
        jog_grid = QtWidgets.QGridLayout(jog_box)
        self.cart_linear_speed = QtWidgets.QComboBox()
        self.cart_linear_speed.addItems(["2", "5", "10", "20"])
        self.cart_angular_speed = QtWidgets.QComboBox()
        self.cart_angular_speed.addItems(["1", "2", "5", "10"])
        jog_grid.addWidget(QtWidgets.QLabel("Linear mm/s"), 0, 0)
        jog_grid.addWidget(self.cart_linear_speed, 0, 1)
        jog_grid.addWidget(QtWidgets.QLabel("Angular deg/s"), 0, 2)
        jog_grid.addWidget(self.cart_angular_speed, 0, 3)
        self.cart_deadman = QtWidgets.QLabel("Deadman: released")
        jog_grid.addWidget(self.cart_deadman, 0, 4, 1, 2)
        for row, axis in enumerate(("X", "Y", "Z", "Rx", "Ry", "Rz"), start=1):
            jog_grid.addWidget(QtWidgets.QLabel(axis), row, 0)
            for column, sign in ((1, -1), (2, 1)):
                button = QtWidgets.QPushButton("−" if sign < 0 else "+")
                button.pressed.connect(
                    lambda axis=axis, sign=sign: self.start_cartesian_jog(axis, sign))
                button.released.connect(self.stop_cartesian_jog)
                jog_grid.addWidget(button, row, column)
        grid.addWidget(jog_box, 5, 0, 1, 7)
        self.api.result_received.connect(self.cart_result.setText)
        self.command_widgets.extend([self.cart_arm, self.cart_linear_speed,
                                     self.cart_angular_speed] + list(self.cart_fields.values()))
        return panel

    def _scope_group(self):
        group = self.group.currentText()
        return {"left_arm": LEFT, "right_arm": RIGHT, "both_arms": BOTH}[group]

    def _target_data(self):
        group = self.group.currentText(); count = 12 if group == "both_arms" else 6
        prefix = "left" if group == "left_arm" else "right"
        names = ([f"left_J{i}" for i in range(1, 7)] + [f"right_J{i}" for i in range(1, 7)]) \
            if count == 12 else [f"{prefix}_J{i}" for i in range(1, 7)]
        return names, [math.radians(self.targets[i][1].value()) for i in range(count)]

    def _targets_for_group(self, *_):
        both = self.group.currentText() == "both_arms"
        for i, (label, field) in enumerate(self.targets):
            visible = i < 6 or both
            label.setVisible(visible); field.setVisible(visible)

    def copy_current(self):
        if not self.api.state: return
        group = self.group.currentText()
        values = list(self.api.state.left_joints.position if group == "left_arm" else self.api.state.right_joints.position)
        if group == "both_arms": values = list(self.api.state.left_joints.position) + list(self.api.state.right_joints.position)
        for (_, field), value in zip(self.targets, values): field.setValue(math.degrees(value))

    def _position(self, operation):
        scope = self._scope_group(); self.api.acquire(scope)
        names, values = self._target_data()
        self.api.position(operation, self.group.currentText(), names, values)

    def validate(self): self._position(JointPosition.Goal.VALIDATE)
    def plan(self): self._position(JointPosition.Goal.PLAN)
    def execute(self):
        self.api.position(JointPosition.Goal.EXECUTE, self.group.currentText(), plan_id=self.api.plan_id)

    def _cartesian_scope(self):
        return LEFT if self.cart_arm.currentText() == "left" else RIGHT

    def copy_current_pose(self):
        if not self.api.state: return
        pose = (self.api.state.left_end_effector if self._cartesian_scope() == LEFT
                else self.api.state.right_end_effector)
        if not pose.header.frame_id:
            self.cart_result.setText("TCP transform unavailable")
            return
        p, q = pose.pose.position, pose.pose.orientation
        for name, value in (("x", p.x), ("y", p.y), ("z", p.z),
                            ("qx", q.x), ("qy", q.y), ("qz", q.z), ("qw", q.w)):
            self.cart_fields[name].setValue(value)

    def _cartesian_target(self):
        from geometry_msgs.msg import PoseStamped
        target = PoseStamped(); target.header.frame_id = "world"
        target.pose.position.x = self.cart_fields["x"].value()
        target.pose.position.y = self.cart_fields["y"].value()
        target.pose.position.z = self.cart_fields["z"].value()
        target.pose.orientation.x = self.cart_fields["qx"].value()
        target.pose.orientation.y = self.cart_fields["qy"].value()
        target.pose.orientation.z = self.cart_fields["qz"].value()
        target.pose.orientation.w = self.cart_fields["qw"].value()
        return target

    def _cartesian(self, operation):
        scope = self._cartesian_scope(); self.api.acquire(scope)
        tcp = "left_flange" if scope == LEFT else "right_flange"
        self.api.cartesian_pose(operation, scope, tcp, self._cartesian_target())

    def validate_cartesian(self): self._cartesian(CartesianPose.Goal.VALIDATE)
    def plan_cartesian(self): self._cartesian(CartesianPose.Goal.PLAN)
    def execute_cartesian(self):
        scope = self._cartesian_scope()
        self.api.cartesian_pose(CartesianPose.Goal.EXECUTE, scope,
                                "left_flange" if scope == LEFT else "right_flange",
                                plan_id=self.api.cartesian_plan_id)

    def start_cartesian_jog(self, axis, sign):
        scope = self._cartesian_scope(); self.api.acquire(scope)
        linear, angular = [0.0] * 3, [0.0] * 3
        if axis in ("X", "Y", "Z"):
            linear[("X", "Y", "Z").index(axis)] = (
                sign * float(self.cart_linear_speed.currentText()) / 1000.0)
        else:
            angular[("Rx", "Ry", "Rz").index(axis)] = (
                sign * math.radians(float(self.cart_angular_speed.currentText())))
        tcp = "left_flange" if scope == LEFT else "right_flange"
        self._cart_jog_request = (scope, tcp, linear, angular)
        QtCore.QTimer.singleShot(100, self._send_cartesian_jog)
        self._cart_jog_timer.start()
        self.cart_deadman.setText("Deadman: HELD")

    def _send_cartesian_jog(self):
        if self._cart_jog_request:
            self.api.cartesian_jog(*self._cart_jog_request)

    def stop_cartesian_jog(self):
        self._cart_jog_timer.stop(); self._cart_jog_request = None
        self.api.stop("Cartesian jog button released")
        self.cart_deadman.setText("Deadman: released")

    def start_jog(self, joint_index, sign):
        arm = LEFT if self.jog_arm.currentText() == "left" else RIGHT
        self.api.acquire(arm)
        velocity = math.radians(float(self.jog_speed.currentText())) * sign
        self._jog_request = (arm, f"{'left' if arm == LEFT else 'right'}_J{joint_index + 1}", velocity)
        # Allow the asynchronous lease acquisition response to arrive before the
        # first command, then stream while the button remains held.
        QtCore.QTimer.singleShot(100, self._send_jog)
        self._jog_timer.start()
        self.jog_deadman.setText("Deadman: HELD")

    def _send_jog(self):
        if self._jog_request:
            self.api.jog(*self._jog_request)

    def stop_jog(self):
        self._jog_timer.stop()
        self._jog_request = None
        self.api.stop("jog button released")
        self.jog_deadman.setText("Deadman: released")

    @QtCore.pyqtSlot(object)
    def update_state(self, msg):
        names = ["OFFLINE", "READ_ONLY", "READY", "JOGGING", "PLANNING", "EXECUTING", "STOPPING", "FAULT", "ESTOP"]
        self.labels["Connection"].setText("connected")
        self.labels["Mode"].setText(msg.operating_mode)
        self.labels["Control state"].setText(names[msg.control_state] if msg.control_state < len(names) else str(msg.control_state))
        self.labels["Ownership"].setText(f"L={msg.left_owner or '—'} R={msg.right_owner or '—'}")
        self.labels["Controllers"].setText(f"L={msg.left_controller_state} R={msg.right_controller_state}")
        self.labels["Freshness"].setText("fresh/complete" if msg.fresh and msg.complete else "NOT READY")
        self.labels["Active command"].setText(msg.active_command or "—")
        self.labels["Reason"].setText(msg.reason)
        for arm, state in ((LEFT, msg.left_joints), (RIGHT, msg.right_joints)):
            for pair, pos, vel in zip(self.joint_labels[arm], state.position, state.velocity):
                pair[0].setText(f"{math.degrees(pos):.3f}"); pair[1].setText(f"{math.degrees(vel):.3f}")
        writable = msg.operating_mode == "mock" and msg.control_state not in (SystemState.READ_ONLY, SystemState.ESTOP)
        for widget in self.command_widgets: widget.setEnabled(writable)

    def focusOutEvent(self, event):
        self._jog_timer.stop(); self._jog_request = None
        self._cart_jog_timer.stop(); self._cart_jog_request = None
        self.api.stop("GUI focus loss")
        super().focusOutEvent(event)

    def closeEvent(self, event):
        self.api.close()
        event.accept()


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    api = RosApi("gui-" + str(uuid.uuid4())[:8])
    window = MainWindow(api); window.show()
    spin = QtCore.QTimer(); spin.timeout.connect(api.spin_once); spin.start(10)
    heartbeat = QtCore.QTimer(); heartbeat.timeout.connect(api.heartbeat); heartbeat.start(300)
    code = app.exec_()
    api.node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
