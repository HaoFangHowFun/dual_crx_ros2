import math
import os
import sys
import uuid

from fanuc_msgs.msg import RobotStatus
from fanuc_msgs.srv import SwitchControlState
from PyQt5 import QtCore, QtWidgets
import rclpy
from control_msgs.msg import JointJog
from dual_crx_interfaces.action import CalibrationCheck
from dual_crx_interfaces.action import JointPosition
from dual_crx_interfaces.action import CartesianPose
from dual_crx_interfaces.msg import SystemState
from dual_crx_interfaces.srv import (AcquireControl, CartesianJog, Heartbeat,
                                     JointJog as JointJogSrv, GetWorkcellInfo)
from dual_crx_interfaces.srv import ReleaseControl, SoftwareStop
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

LEFT, RIGHT, BOTH = 1, 2, 3


class RosApi(QtCore.QObject):
    state_received = QtCore.pyqtSignal(object)
    robot_status_received = QtCore.pyqtSignal(int, object)
    result_received = QtCore.pyqtSignal(str)
    workcell_received = QtCore.pyqtSignal(object)
    calibration_feedback_received = QtCore.pyqtSignal(object)
    calibration_result_received = QtCore.pyqtSignal(bool, str)

    def __init__(self, client_id):
        super().__init__()
        self.client_id = client_id
        self.node = Node("dual_crx_gui_client")
        self.state = None
        self.robot_status = {}
        self.scope = None
        self.plan_id = ""
        self.cartesian_plan_id = ""
        self.goal_handle = None
        self.calibration_goal_handle = None
        self.calibration_request_pending = False
        self.node.create_subscription(SystemState, "/dual_crx/state", self._state, 10)
        self.node.create_subscription(
            RobotStatus, "/left_arm/fanuc_gpio_controller/robot_status",
            lambda msg: self._robot_status(LEFT, msg), 10)
        self.node.create_subscription(
            RobotStatus, "/right_arm/fanuc_gpio_controller/robot_status",
            lambda msg: self._robot_status(RIGHT, msg), 10)
        self.acquire_client = self.node.create_client(AcquireControl, "/dual_crx/acquire_control")
        self.release_client = self.node.create_client(ReleaseControl, "/dual_crx/release_control")
        self.heartbeat_client = self.node.create_client(Heartbeat, "/dual_crx/heartbeat")
        self.jog_client = self.node.create_client(JointJogSrv, "/dual_crx/jog")
        self.cartesian_jog_client = self.node.create_client(
            CartesianJog, "/dual_crx/cartesian_jog")
        self.stop_client = self.node.create_client(SoftwareStop, "/dual_crx/stop")
        self.position_client = ActionClient(self.node, JointPosition, "/dual_crx/joint_position")
        self.cartesian_client = ActionClient(self.node, CartesianPose, "/dual_crx/cartesian_pose")
        self.calibration_client = ActionClient(
            self.node, CalibrationCheck, "/dual_crx/calibration_check")
        self.workcell_client = self.node.create_client(
            GetWorkcellInfo, "/dual_crx/workcell_info")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.motion_clients = {
            LEFT: self.node.create_client(
                SwitchControlState, "/left_arm/fanuc_gpio_controller/switch_control_state"),
            RIGHT: self.node.create_client(
                SwitchControlState, "/right_arm/fanuc_gpio_controller/switch_control_state"),
        }

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _state(self, msg):
        self.state = msg
        self.state_received.emit(msg)

    def _robot_status(self, arm, msg):
        self.robot_status[arm] = msg
        self.robot_status_received.emit(arm, msg)

    def acquire(self, scope, callback=None):
        req = AcquireControl.Request(client_id=self.client_id, source_type="GUI",
                                     arm_scope=scope, requested_mode="JOINT", lease_duration=1.0)
        future = self.acquire_client.call_async(req)
        future.add_done_callback(
            lambda f, scope=scope, callback=callback:
            self._acquire_result(f, scope, callback))

    def _acquire_result(self, future, scope, callback=None):
        try:
            response = future.result()
        except Exception as exc:
            self.scope = None
            self.result_received.emit("acquire failed")
            if callback:
                callback(False, str(exc))
            return
        if response.accepted:
            self.scope = scope
            if callback:
                callback(True, response.reason)
        else:
            self.scope = None
            self.result_received.emit(response.reason or "acquire rejected")
            if callback:
                callback(False, response.reason or "acquire rejected")

    def heartbeat(self):
        if self.scope:
            self.heartbeat_client.call_async(Heartbeat.Request(
                client_id=self.client_id, arm_scope=self.scope, lease_duration=1.0))

    def release(self):
        if self.scope:
            self.release_client.call_async(ReleaseControl.Request(
                client_id=self.client_id, arm_scope=self.scope))
            self.scope = None

    def switch_motion(self, arm, enable):
        client = self.motion_clients[arm]
        if not client.service_is_ready():
            self.result_received.emit(
                f"{'left' if arm == LEFT else 'right'} motion service unavailable")
            return
        req = SwitchControlState.Request()
        req.status = (SwitchControlState.Request.START
                      if enable else SwitchControlState.Request.STOP)
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, arm=arm, enable=enable: self._switch_motion_result(f, arm, enable))

    def _switch_motion_result(self, future, arm, enable):
        arm_name = "left" if arm == LEFT else "right"
        try:
            response = future.result()
        except Exception as exc:
            self.result_received.emit(f"{arm_name} motion switch failed: {exc}")
            return
        if response.result == 0:
            self.result_received.emit(
                f"{arm_name} motion {'enabled' if enable else 'disabled'}")
        else:
            self.result_received.emit(
                f"{arm_name} motion {'enable' if enable else 'disable'} rejected ({response.result})")

    def jog(self, scope, joint, velocity):
        command = JointJog()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.joint_names = [joint]
        command.velocities = [velocity]
        self.jog_client.call_async(JointJogSrv.Request(
            client_id=self.client_id, arm_scope=scope, command=command))

    def cartesian_jog(self, scope, tcp_link, frame_id, linear, angular):
        from geometry_msgs.msg import TwistStamped
        command = TwistStamped(); command.header.frame_id = frame_id
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.twist.linear.x, command.twist.linear.y, command.twist.linear.z = linear
        command.twist.angular.x, command.twist.angular.y, command.twist.angular.z = angular
        self.cartesian_jog_client.call_async(CartesianJog.Request(
            client_id=self.client_id, arm_scope=scope, tcp_link=tcp_link,
            command=command))

    def request_workcell_info(self):
        if not self.workcell_client.service_is_ready():
            self.result_received.emit("workcell info service unavailable")
            return
        future = self.workcell_client.call_async(GetWorkcellInfo.Request())
        future.add_done_callback(self._workcell_result)

    def _workcell_result(self, future):
        try:
            self.workcell_received.emit(future.result())
        except Exception as exc:
            self.result_received.emit(f"workcell info failed: {exc}")

    def tcp_pose(self, frame_id, tcp_link):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id, tcp_link, rclpy.time.Time())
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @classmethod
    def _rotate_vector(cls, quaternion, vector):
        qx, qy, qz, qw = quaternion
        rotated = cls._quaternion_multiply(
            cls._quaternion_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
            (-qx, -qy, -qz, qw),
        )
        return rotated[:3]

    def transform_pose(self, pose, target_frame):
        if pose.header.frame_id == target_frame:
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, pose.header.frame_id, rclpy.time.Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_q = (rotation.x, rotation.y, rotation.z, rotation.w)
        position = pose.pose.position
        rotated = self._rotate_vector(transform_q, (position.x, position.y, position.z))
        source_q = (
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
        )
        output = PoseStamped()
        output.header.frame_id = target_frame
        output.pose.position.x = translation.x + rotated[0]
        output.pose.position.y = translation.y + rotated[1]
        output.pose.position.z = translation.z + rotated[2]
        (
            output.pose.orientation.x, output.pose.orientation.y,
            output.pose.orientation.z, output.pose.orientation.w,
        ) = self._quaternion_multiply(transform_q, source_q)
        return output

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

    def calibration_check(self, plan_only, clearance_m, transit_height_m):
        self.calibration_request_pending = True

        def send(acquired, reason):
            if not self.calibration_request_pending:
                if acquired:
                    self.release()
                return
            if not acquired:
                self.calibration_request_pending = False
                self.calibration_result_received.emit(False, reason)
                return
            if not self.calibration_client.server_is_ready():
                self.calibration_request_pending = False
                self.release()
                self.calibration_result_received.emit(
                    False, "five-point verification action unavailable")
                return
            goal = CalibrationCheck.Goal(
                client_id=self.client_id,
                plan_only=plan_only,
                clearance_m=clearance_m,
                transit_height_m=transit_height_m,
            )
            future = self.calibration_client.send_goal_async(
                goal, feedback_callback=self._calibration_feedback)
            future.add_done_callback(self._calibration_goal_response)
        self.acquire(BOTH, send)

    def _calibration_feedback(self, feedback):
        self.calibration_feedback_received.emit(feedback.feedback)

    def _calibration_goal_response(self, future):
        try:
            self.calibration_goal_handle = future.result()
        except Exception as exc:
            self.calibration_request_pending = False
            self.release()
            self.calibration_result_received.emit(False, str(exc))
            return
        if not self.calibration_goal_handle.accepted:
            self.calibration_request_pending = False
            self.release()
            self.calibration_result_received.emit(False, "five-point command rejected")
            return
        if not self.calibration_request_pending:
            self.calibration_goal_handle.cancel_goal_async()
            self.release()
            self.calibration_result_received.emit(False, "canceled before execution")
            return
        result = self.calibration_goal_handle.get_result_async()
        result.add_done_callback(self._calibration_result)

    def _calibration_result(self, future):
        try:
            result = future.result().result
            self.calibration_result_received.emit(result.success, result.reason)
        except Exception as exc:
            self.calibration_result_received.emit(False, str(exc))
        finally:
            self.calibration_request_pending = False
            self.calibration_goal_handle = None
            self.release()

    def cancel_calibration_check(self):
        self.calibration_request_pending = False
        if self.calibration_goal_handle:
            self.calibration_goal_handle.cancel_goal_async()
        self.stop("five-point verification canceled from GUI")
        self.calibration_result_received.emit(False, "cancel requested")

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
        if self.calibration_goal_handle:
            self.calibration_goal_handle.cancel_goal_async()

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
        self.resize(1250, 900)
        self._jog_request = None
        self._jog_timer = QtCore.QTimer(self)
        self._jog_timer.setInterval(100)
        self._jog_timer.timeout.connect(self._send_jog)
        self._cart_jog_request = None
        self._cart_jog_timer = QtCore.QTimer(self)
        self._cart_jog_timer.setInterval(50)
        self._cart_jog_timer.timeout.connect(self._send_cartesian_jog)
        self._verification_active = False
        self.workcell_info = None
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
        layout.addWidget(self._workcell_panel())
        self.stop_button = QtWidgets.QPushButton("STOP")
        self.stop_button.setStyleSheet("font-size: 22px; font-weight: bold; color: white; background: #b00020")
        self.stop_button.clicked.connect(self.stop_all)
        layout.addWidget(self.stop_button)
        self.motion_box = QtWidgets.QGroupBox("Physical motion control")
        motion_layout = QtWidgets.QGridLayout(self.motion_box)
        self.motion_labels = {}
        for row, (arm, title) in enumerate(((LEFT, "Left"), (RIGHT, "Right"))):
            motion_layout.addWidget(QtWidgets.QLabel(title), row, 0)
            label = QtWidgets.QLabel("status unavailable")
            self.motion_labels[arm] = label
            motion_layout.addWidget(label, row, 1)
            enable_button = QtWidgets.QPushButton("Enable Motion")
            enable_button.clicked.connect(lambda _=False, arm=arm: self.api.switch_motion(arm, True))
            motion_layout.addWidget(enable_button, row, 2)
            disable_button = QtWidgets.QPushButton("Disable Motion")
            disable_button.clicked.connect(lambda _=False, arm=arm: self.api.switch_motion(arm, False))
            motion_layout.addWidget(disable_button, row, 3)
        self.motion_box.setVisible(False)
        layout.addWidget(self.motion_box)

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
        tabs.addTab(self._calibration_check_panel(), "5-point check")
        layout.addWidget(tabs)
        self.api.state_received.connect(self.update_state)
        self.api.robot_status_received.connect(self.update_robot_status)
        self.api.result_received.connect(self.command_result.setText)
        self.api.workcell_received.connect(self.update_workcell_info)
        self.api.calibration_feedback_received.connect(self.update_calibration_feedback)
        self.api.calibration_result_received.connect(self.calibration_check_finished)
        QtCore.QTimer.singleShot(1000, self.api.request_workcell_info)

    def _workcell_panel(self):
        panel = QtWidgets.QGroupBox("Loaded world / table calibration")
        grid = QtWidgets.QGridLayout(panel)
        self.workcell_labels = {}
        fields = ("World frame", "Profile", "Active file", "Status", "EEF Z", "SHA-256")
        for index, field in enumerate(fields):
            row, column = divmod(index, 2)
            column *= 2
            grid.addWidget(QtWidgets.QLabel(field + ":"), row, column)
            value = QtWidgets.QLabel("—")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.workcell_labels[field] = value
            grid.addWidget(value, row, column + 1)
        refresh = QtWidgets.QPushButton("Refresh info")
        refresh.clicked.connect(self.api.request_workcell_info)
        grid.addWidget(refresh, 3, 0, 1, 4)
        return panel

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
        self.cart_frame = QtWidgets.QComboBox()
        self.cart_frame.addItems(["Table / World", "Robot Base"])
        grid.addWidget(QtWidgets.QLabel("Reference frame"), 0, 2)
        grid.addWidget(self.cart_frame, 0, 3)
        self.cart_frame_resolved = QtWidgets.QLabel("world")
        grid.addWidget(self.cart_frame_resolved, 0, 4, 1, 3)
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
        jog_box = QtWidgets.QGroupBox("MoveIt Servo jog (press and hold, selected frame)")
        jog_grid = QtWidgets.QGridLayout(jog_box)
        self.cart_linear_speed = QtWidgets.QComboBox()
        self.cart_linear_speed.addItems(["2", "5", "10", "20"])
        self.cart_angular_speed = QtWidgets.QComboBox()
        self.cart_angular_speed.addItems(["1", "2", "5", "10"])
        self.cart_jog_frame = QtWidgets.QComboBox()
        self.cart_jog_frame.addItems(["Pose reference", "Tool / TCP"])
        jog_grid.addWidget(QtWidgets.QLabel("Linear mm/s"), 0, 0)
        jog_grid.addWidget(self.cart_linear_speed, 0, 1)
        jog_grid.addWidget(QtWidgets.QLabel("Angular deg/s"), 0, 2)
        jog_grid.addWidget(self.cart_angular_speed, 0, 3)
        jog_grid.addWidget(QtWidgets.QLabel("Jog frame"), 0, 4)
        jog_grid.addWidget(self.cart_jog_frame, 0, 5)
        self.cart_deadman = QtWidgets.QLabel("Deadman: released")
        jog_grid.addWidget(self.cart_deadman, 0, 6)
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
        self.command_widgets.extend([
            self.cart_arm, self.cart_frame, self.cart_linear_speed,
            self.cart_angular_speed, self.cart_jog_frame,
        ] + list(self.cart_fields.values()))
        self._last_cart_frame = "world"
        self._cart_target_frame_valid = True
        self.cart_frame.currentTextChanged.connect(self._cartesian_frame_changed)
        self.cart_arm.currentTextChanged.connect(self._cartesian_frame_changed)
        self.cart_frame.currentTextChanged.connect(self._invalidate_cartesian_plan)
        self.cart_arm.currentTextChanged.connect(self._invalidate_cartesian_plan)
        for field in self.cart_fields.values():
            field.valueChanged.connect(self._invalidate_cartesian_plan)
        return panel

    def _calibration_check_panel(self):
        panel = QtWidgets.QWidget(); grid = QtWidgets.QGridLayout(panel)
        explanation = QtWidgets.QLabel(
            "Verification only: CENTER plus four corners, left arm first, then right. "
            "Each arm returns to its captured starting joints.")
        explanation.setWordWrap(True)
        grid.addWidget(explanation, 0, 0, 1, 4)
        self.check_clearance = QtWidgets.QDoubleSpinBox()
        self.check_clearance.setRange(15.0, 100.0)
        self.check_clearance.setSingleStep(5.0)
        self.check_clearance.setValue(20.0)
        self.check_clearance.setSuffix(" mm")
        self.check_transit = QtWidgets.QDoubleSpinBox()
        self.check_transit.setRange(50.0, 300.0)
        self.check_transit.setSingleStep(10.0)
        self.check_transit.setValue(80.0)
        self.check_transit.setSuffix(" mm")
        self.check_clearance.valueChanged.connect(self._sync_check_transit)
        grid.addWidget(QtWidgets.QLabel("Checkpoint clearance"), 1, 0)
        grid.addWidget(self.check_clearance, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Transit height"), 1, 2)
        grid.addWidget(self.check_transit, 1, 3)
        self.check_plan_button = QtWidgets.QPushButton("Plan / RViz preview")
        self.check_plan_button.clicked.connect(lambda: self.start_calibration_check(True))
        self.check_run_button = QtWidgets.QPushButton("RUN left → right")
        self.check_run_button.clicked.connect(lambda: self.start_calibration_check(False))
        self.check_cancel_button = QtWidgets.QPushButton("Cancel verification")
        self.check_cancel_button.clicked.connect(self.api.cancel_calibration_check)
        self.check_cancel_button.setEnabled(False)
        grid.addWidget(self.check_plan_button, 2, 0, 1, 2)
        grid.addWidget(self.check_run_button, 2, 2)
        grid.addWidget(self.check_cancel_button, 2, 3)
        self.check_progress = QtWidgets.QProgressBar()
        self.check_progress.setRange(0, 100)
        self.check_progress.setValue(0)
        self.check_status = QtWidgets.QLabel("No five-point verification running")
        self.check_status.setWordWrap(True)
        grid.addWidget(self.check_progress, 3, 0, 1, 4)
        grid.addWidget(self.check_status, 4, 0, 1, 4)
        safety = QtWidgets.QLabel(
            "Safety floor: 15 mm. The current calibration's 5.22 mm worst-point error "
            "makes a 3 mm automatic pass unsafe.")
        safety.setStyleSheet("color: #b00020; font-weight: bold")
        safety.setWordWrap(True)
        grid.addWidget(safety, 5, 0, 1, 4)
        self.command_widgets.extend([
            self.check_clearance, self.check_transit,
            self.check_plan_button, self.check_run_button,
        ])
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

    def _cartesian_frame(self):
        if self.cart_frame.currentText() == "Table / World":
            return "world"
        return "left_base_link" if self._cartesian_scope() == LEFT else "right_base_link"

    def _populate_cartesian_fields(self, pose):
        p, q = pose.pose.position, pose.pose.orientation
        for name, value in (("x", p.x), ("y", p.y), ("z", p.z),
                            ("qx", q.x), ("qy", q.y), ("qz", q.z), ("qw", q.w)):
            self.cart_fields[name].setValue(value)

    def _cartesian_frame_changed(self, *_):
        new_frame = self._cartesian_frame()
        self.cart_frame_resolved.setText(new_frame)
        if new_frame == self._last_cart_frame:
            return
        old_target = self._cartesian_target(self._last_cart_frame)
        transformed = self.api.transform_pose(old_target, new_frame)
        self._last_cart_frame = new_frame
        if transformed is None:
            self._cart_target_frame_valid = False
            self.cart_result.setText(
                f"TF unavailable for {new_frame}; press Copy current TCP before planning")
            return
        self._populate_cartesian_fields(transformed)
        self._cart_target_frame_valid = True
        self.cart_result.setText(f"Target converted to {new_frame}")

    def copy_current_pose(self):
        if not self.api.state:
            return
        frame = self._cartesian_frame()
        tcp = "left_flange" if self._cartesian_scope() == LEFT else "right_flange"
        pose = self.api.tcp_pose(frame, tcp)
        if pose is None and frame == "world":
            pose = (self.api.state.left_end_effector if self._cartesian_scope() == LEFT
                    else self.api.state.right_end_effector)
        if pose is None or not pose.header.frame_id:
            self.cart_result.setText("TCP transform unavailable")
            return
        self._populate_cartesian_fields(pose)
        self._cart_target_frame_valid = True
        self.cart_result.setText(f"Current {tcp} copied in {frame}")

    def _cartesian_target(self, frame_id=None):
        target = PoseStamped(); target.header.frame_id = frame_id or self._cartesian_frame()
        target.pose.position.x = self.cart_fields["x"].value()
        target.pose.position.y = self.cart_fields["y"].value()
        target.pose.position.z = self.cart_fields["z"].value()
        target.pose.orientation.x = self.cart_fields["qx"].value()
        target.pose.orientation.y = self.cart_fields["qy"].value()
        target.pose.orientation.z = self.cart_fields["qz"].value()
        target.pose.orientation.w = self.cart_fields["qw"].value()
        return target

    def _invalidate_cartesian_plan(self, *_):
        self.api.cartesian_plan_id = ""

    def _cartesian(self, operation):
        if not self._cart_target_frame_valid:
            self.cart_result.setText("Target frame is unresolved; copy the current TCP first")
            return
        scope = self._cartesian_scope()
        tcp = "left_flange" if scope == LEFT else "right_flange"
        target = self._cartesian_target()

        def send(acquired, reason):
            if acquired:
                self.api.cartesian_pose(operation, scope, tcp, target)
            else:
                self.cart_result.setText(reason)
        self.api.acquire(scope, send)

    def validate_cartesian(self): self._cartesian(CartesianPose.Goal.VALIDATE)
    def plan_cartesian(self): self._cartesian(CartesianPose.Goal.PLAN)
    def execute_cartesian(self):
        if not self.api.cartesian_plan_id:
            self.cart_result.setText("No current Cartesian plan; press Plan first")
            return
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
        frame = tcp if self.cart_jog_frame.currentText() == "Tool / TCP" \
            else self._cartesian_frame()
        self._cart_jog_request = (scope, tcp, frame, linear, angular)
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

    def _sync_check_transit(self, clearance_mm):
        self.check_transit.setMinimum(clearance_mm + 30.0)

    def _physical_preflight(self):
        if not self.api.state:
            return False, "No robot state is available"
        if self.api.state.operating_mode != "physical":
            return True, "mock mode"
        for arm, name in ((LEFT, "left"), (RIGHT, "right")):
            status = self.api.robot_status.get(arm)
            if status is None:
                return False, f"{name} robot status is unavailable"
            if not status.motion_possible:
                return False, f"Enable Motion is required for the {name} arm"
            if status.in_error or status.e_stopped or status.tp_enabled:
                return False, f"{name} arm is not ready (error/e-stop/TP state)"
        return True, "ready"

    def start_calibration_check(self, plan_only):
        if self._verification_active:
            return
        if not self.workcell_info or not self.workcell_info.loaded:
            self.check_status.setText("Workcell placement information is unavailable")
            return
        if not self.workcell_info.valid:
            self.check_status.setText("The loaded placement is not marked VALID")
            return
        if not plan_only:
            ready, reason = self._physical_preflight()
            if not ready:
                QtWidgets.QMessageBox.critical(self, "Five-point preflight failed", reason)
                return
            answer = QtWidgets.QMessageBox.warning(
                self,
                "Automatic left → right motion",
                "Before continuing:\n\n"
                "• Move BOTH arms well apart in safe starting poses.\n"
                "• Point both tools in the intended verification orientation.\n"
                "• Clear all people and objects from the work area.\n"
                "• Keep the E-stop within reach.\n"
                "• Verify the loaded calibration profile shown above.\n\n"
                "The left arm will visit all five points and return to its captured "
                "start joints. The right arm will then do the same. Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        self._verification_active = True
        self.check_progress.setValue(0)
        self.check_status.setText(
            "Acquiring both arms for plan preview" if plan_only
            else "Acquiring both arms for five-point verification")
        self._update_check_controls()
        self.api.calibration_check(
            plan_only,
            self.check_clearance.value() / 1000.0,
            self.check_transit.value() / 1000.0,
        )

    def _update_check_controls(self):
        self.check_plan_button.setEnabled(not self._verification_active)
        self.check_run_button.setEnabled(not self._verification_active)
        self.check_clearance.setEnabled(not self._verification_active)
        self.check_transit.setEnabled(not self._verification_active)
        self.check_cancel_button.setEnabled(self._verification_active)

    @QtCore.pyqtSlot(object)
    def update_calibration_feedback(self, feedback):
        self.check_progress.setValue(round(feedback.progress * 100.0))
        self.check_status.setText(
            f"{feedback.arm.upper()} {feedback.point} — {feedback.phase} "
            f"({feedback.completed}/{feedback.total})")

    @QtCore.pyqtSlot(bool, str)
    def calibration_check_finished(self, success, reason):
        self._verification_active = False
        self._update_check_controls()
        if success:
            self.check_progress.setValue(100)
        self.check_status.setText(("PASS: " if success else "STOPPED: ") + reason)

    @QtCore.pyqtSlot(object)
    def update_workcell_info(self, info):
        self.workcell_info = info
        world_text = f"{info.world_frame} (= {info.table_frame})"
        profile = info.profile_name or os.path.basename(info.source_calibration) or "unnamed"
        status = "VALID" if info.valid else "NOT VALID"
        if info.generated_at:
            status += " · " + info.generated_at
        active_file = info.placement_file or "—"
        self.workcell_labels["World frame"].setText(world_text)
        self.workcell_labels["Profile"].setText(profile)
        self.workcell_labels["Active file"].setText(active_file)
        self.workcell_labels["Active file"].setToolTip(active_file)
        self.workcell_labels["Status"].setText(status if info.loaded else info.reason)
        self.workcell_labels["EEF Z"].setText(
            f"L={info.left_eef_z_offset_m * 1000:.1f} mm · "
            f"R={info.right_eef_z_offset_m * 1000:.1f} mm")
        self.workcell_labels["SHA-256"].setText(info.sha256[:12] or "—")
        self.workcell_labels["SHA-256"].setToolTip(info.sha256)

    def stop_all(self):
        if self._verification_active:
            self.api.cancel_calibration_check()
        else:
            self.api.stop()

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
        self.motion_box.setVisible(msg.operating_mode == "physical")
        for arm, state in ((LEFT, msg.left_joints), (RIGHT, msg.right_joints)):
            for pair, pos, vel in zip(self.joint_labels[arm], state.position, state.velocity):
                pair[0].setText(f"{math.degrees(pos):.3f}"); pair[1].setText(f"{math.degrees(vel):.3f}")
        writable = msg.control_state not in (SystemState.READ_ONLY, SystemState.ESTOP)
        for widget in self.command_widgets: widget.setEnabled(writable)
        if writable:
            self._update_check_controls()

    @QtCore.pyqtSlot(int, object)
    def update_robot_status(self, arm, msg):
        parts = [
            "motion on" if msg.motion_possible else "motion off",
            "ok" if not msg.in_error else "error",
            "tp off" if not msg.tp_enabled else "tp on",
            "estop off" if not msg.e_stopped else "estop on",
        ]
        self.motion_labels[arm].setText(" | ".join(parts))

    def focusOutEvent(self, event):
        self._jog_timer.stop(); self._jog_request = None
        self._cart_jog_timer.stop(); self._cart_jog_request = None
        if self._verification_active:
            self.api.cancel_calibration_check()
        else:
            self.api.stop("GUI focus loss")
        super().focusOutEvent(event)

    def closeEvent(self, event):
        if self._verification_active:
            self.api.cancel_calibration_check()
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
