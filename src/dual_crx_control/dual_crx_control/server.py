"""ROS adapter for the dual CRX safety/control gateway."""

import uuid

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from dual_crx_interfaces.action import CalibrationCheck
from dual_crx_interfaces.action import JointPosition
from dual_crx_interfaces.action import CartesianPose
from dual_crx_interfaces.msg import SystemState
from dual_crx_interfaces.srv import (AcquireControl, CartesianJog, Heartbeat, JointJog,
                                     ReleaseControl, SoftwareStop, GetWorkcellInfo)
from fanuc_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PositionConstraint)
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .core import (BOTH, EXECUTING, FAULT, JOGGING, LEFT, PLANNING, READY,
                   RIGHT, STOPPING, ControlCore)
from .workcell import WorkcellError, load_workcell, verification_targets


class ControlServer(Node):
    def __init__(self):
        super().__init__("dual_crx_control_server")
        self.declare_parameter("operating_mode", "mock")
        self.declare_parameter("allow_physical_control", False)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("max_jog_velocity", 0.2)
        self.declare_parameter("jog_step_duration", 0.15)
        self.declare_parameter("publish_period", 0.1)
        self.declare_parameter("robot_placement_file", "")
        mode = self.get_parameter("operating_mode").value
        allow_physical_control = bool(
            self.get_parameter("allow_physical_control").value)
        self._core = ControlCore(
            physical=mode != "mock",
            allow_physical_control=allow_physical_control,
            state_timeout=float(self.get_parameter("state_timeout").value),
            max_jog_velocity=float(self.get_parameter("max_jog_velocity").value),
        )
        self._mode = mode
        self._group = ReentrantCallbackGroup()
        self._plans = {}
        self._active_handles = []
        self._target = {LEFT: [], RIGHT: []}
        self._active_command = ""
        self._progress = 0.0
        self._servo_configured = {LEFT: False, RIGHT: False}
        self._servo_active = set()
        self._robot_status = {}
        self._placement_data = {}
        self._workcell_info = {
            "loaded": False,
            "valid": False,
            "world_frame": "world",
            "table_frame": "table_frame",
            "placement_file": str(self.get_parameter("robot_placement_file").value),
            "profile_name": "",
            "generated_at": "",
            "source_calibration": "",
            "sha256": "",
            "left_eef_z_offset_m": 0.0,
            "right_eef_z_offset_m": 0.0,
            "reason": "placement file was not provided",
        }
        placement_file = str(self.get_parameter("robot_placement_file").value)
        if placement_file:
            try:
                self._placement_data, self._workcell_info = load_workcell(placement_file)
            except WorkcellError as exc:
                self._workcell_info["reason"] = str(exc)
                self.get_logger().error(f"workcell metadata unavailable: {exc}")

        self.create_subscription(JointState, "/left_arm/joint_states",
                                 lambda m: self._state_cb(LEFT, m), qos_profile_sensor_data,
                                 callback_group=self._group)
        self.create_subscription(JointState, "/right_arm/joint_states",
                                 lambda m: self._state_cb(RIGHT, m), qos_profile_sensor_data,
                                 callback_group=self._group)
        self.create_subscription(
            RobotStatus, "/left_arm/fanuc_gpio_controller/robot_status",
            lambda m: self._robot_status_cb(LEFT, m), 10, callback_group=self._group)
        self.create_subscription(
            RobotStatus, "/right_arm/fanuc_gpio_controller/robot_status",
            lambda m: self._robot_status_cb(RIGHT, m), 10, callback_group=self._group)
        self._state_pub = self.create_publisher(SystemState, "/dual_crx/state", 10)

        self._trajectory = {
            LEFT: ActionClient(self, FollowJointTrajectory,
                               "/left_arm/joint_trajectory_controller/follow_joint_trajectory",
                               callback_group=self._group),
            RIGHT: ActionClient(self, FollowJointTrajectory,
                                "/right_arm/joint_trajectory_controller/follow_joint_trajectory",
                                callback_group=self._group),
        }
        self._servo_twist = {
            LEFT: self.create_publisher(
                TwistStamped, "/dual_crx/internal/left_servo/twist", 10),
            RIGHT: self.create_publisher(
                TwistStamped, "/dual_crx/internal/right_servo/twist", 10),
        }
        self._servo_type = {
            LEFT: self.create_client(
                ServoCommandType, "/left_servo/servo_node/switch_command_type",
                callback_group=self._group),
            RIGHT: self.create_client(
                ServoCommandType, "/right_servo/servo_node/switch_command_type",
                callback_group=self._group),
        }
        self._servo_pause = {
            LEFT: self.create_client(
                SetBool, "/left_servo/servo_node/pause_servo", callback_group=self._group),
            RIGHT: self.create_client(
                SetBool, "/right_servo/servo_node/pause_servo", callback_group=self._group),
        }
        self._move_group = ActionClient(self, MoveGroup, "/move_action", callback_group=self._group)

        self.create_service(AcquireControl, "/dual_crx/acquire_control", self._acquire,
                            callback_group=self._group)
        self.create_service(ReleaseControl, "/dual_crx/release_control", self._release,
                            callback_group=self._group)
        self.create_service(Heartbeat, "/dual_crx/heartbeat", self._heartbeat,
                            callback_group=self._group)
        self.create_service(JointJog, "/dual_crx/jog", self._jog, callback_group=self._group)
        self.create_service(CartesianJog, "/dual_crx/cartesian_jog", self._cartesian_jog,
                            callback_group=self._group)
        self.create_service(GetWorkcellInfo, "/dual_crx/workcell_info", self._get_workcell_info,
                            callback_group=self._group)
        self.create_service(SoftwareStop, "/dual_crx/stop", self._stop, callback_group=self._group)
        self._position_server = ActionServer(
            self, JointPosition, "/dual_crx/joint_position", execute_callback=self._position,
            goal_callback=self._position_goal, cancel_callback=self._position_cancel,
            callback_group=self._group)
        self._cartesian_server = ActionServer(
            self, CartesianPose, "/dual_crx/cartesian_pose",
            execute_callback=self._cartesian_pose,
            goal_callback=self._cartesian_goal,
            cancel_callback=self._cartesian_cancel,
            callback_group=self._group)
        self._calibration_check_server = ActionServer(
            self, CalibrationCheck, "/dual_crx/calibration_check",
            execute_callback=self._calibration_check,
            goal_callback=self._calibration_check_goal,
            cancel_callback=self._calibration_check_cancel,
            callback_group=self._group)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(float(self.get_parameter("publish_period").value), self._tick,
                          callback_group=self._group)
        self.get_logger().info(f"control gateway started in {mode!r} mode")
        self.get_logger().info(
            "workcell placement: " + self._workcell_info.get("placement_file", "")
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _state_cb(self, arm, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # Some hardware/mock publishers omit a stamp. Reception time remains explicit and fresh.
        if stamp == 0.0:
            stamp = self._now()
        ok, reason = self._core.update_state(arm, stamp, msg.name, msg.position, msg.velocity)
        if not ok:
            self._core.reason = reason

    def _robot_status_cb(self, arm, msg):
        self._robot_status[arm] = (self._now(), msg)

    def _get_workcell_info(self, _req, res):
        for field in (
            "loaded", "valid", "world_frame", "table_frame", "placement_file",
            "profile_name", "generated_at", "source_calibration", "sha256",
            "left_eef_z_offset_m", "right_eef_z_offset_m", "reason",
        ):
            setattr(res, field, self._workcell_info[field])
        return res

    @staticmethod
    def _time_msg(seconds):
        from builtin_interfaces.msg import Time
        msg = Time()
        if seconds > 0:
            msg.sec = int(seconds)
            msg.nanosec = int((seconds - int(seconds)) * 1e9)
        return msg

    def _tick(self):
        now = self._now()
        self._core.controllers[LEFT] = self._trajectory[LEFT].server_is_ready()
        self._core.controllers[RIGHT] = self._trajectory[RIGHT].server_is_ready()
        for arm in (LEFT, RIGHT):
            if not self._servo_configured[arm] and self._servo_type[arm].service_is_ready():
                request = ServoCommandType.Request(
                    command_type=ServoCommandType.Request.TWIST)
                future = self._servo_type[arm].call_async(request)
                future.add_done_callback(lambda f, arm=arm: self._servo_ready(arm, f))
        previous_active = self._core.active_scope
        self._core.refresh(now)
        if previous_active is not None and self._core.active_scope is None:
            self._cancel_motion()
        out = SystemState()
        out.stamp = self.get_clock().now().to_msg()
        out.operating_mode = self._mode
        out.control_state = self._core.control_state
        ready, _ = self._core.readiness(now)
        out.complete = LEFT in self._core.states and RIGHT in self._core.states
        out.fresh = ready
        for arm, attr in ((LEFT, "left_joints"), (RIGHT, "right_joints")):
            if arm in self._core.states:
                names, positions, velocities = self._core.canonical(arm)
                state = getattr(out, attr)
                state.header.stamp = self._time_msg(self._core.states[arm][0])
                state.name, state.position, state.velocity = names, positions, velocities
        out.left_target = self._target[LEFT]
        out.right_target = self._target[RIGHT]
        self._populate_pose(out.left_end_effector, "left_flange")
        self._populate_pose(out.right_end_effector, "right_flange")
        out.left_controller_available = self._core.controllers[LEFT]
        out.right_controller_available = self._core.controllers[RIGHT]
        out.left_controller_state = "active" if self._core.controllers[LEFT] else "unavailable"
        out.right_controller_state = "active" if self._core.controllers[RIGHT] else "unavailable"
        for arm, owner_attr, expiry_attr in (
                (LEFT, "left_owner", "left_lease_expires"),
                (RIGHT, "right_owner", "right_lease_expires")):
            lease = self._core.leases.get(arm)
            if lease:
                setattr(out, owner_attr, lease.client_id)
                setattr(out, expiry_attr, self._time_msg(lease.expires_at))
        out.active_command = self._active_command
        out.progress = self._progress
        out.reason = self._core.reason
        self._state_pub.publish(out)

    def _servo_ready(self, arm, future):
        try:
            self._servo_configured[arm] = bool(future.result().success)
            if self._servo_configured[arm] and self._servo_pause[arm].service_is_ready():
                self._servo_pause[arm].call_async(SetBool.Request(data=False))
        except Exception as exc:
            self._core.reason = f"Servo configuration failed: {exc}"

    def _populate_pose(self, pose, link):
        try:
            transform = self._tf_buffer.lookup_transform("world", link, rclpy.time.Time())
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
        except TransformException:
            pose.header.frame_id = ""

    def _acquire(self, req, res):
        res.accepted, res.reason, expiry = self._core.acquire(
            req.client_id, req.source_type, req.arm_scope, req.requested_mode,
            req.lease_duration, self._now())
        res.expires_at = self._time_msg(expiry)
        return res

    def _release(self, req, res):
        res.released, res.reason = self._core.release(req.client_id, req.arm_scope)
        if res.released:
            self._cancel_motion()
        return res

    def _heartbeat(self, req, res):
        res.accepted, res.reason, expiry = self._core.heartbeat(
            req.client_id, req.arm_scope, req.lease_duration, self._now())
        res.expires_at = self._time_msg(expiry)
        return res

    def _jog(self, req, res):
        stamp = req.command.header.stamp.sec + req.command.header.stamp.nanosec / 1e9
        res.accepted, res.reason = self._core.validate_jog(
            req.client_id, req.arm_scope, req.command.joint_names,
            req.command.velocities, stamp, self._now())
        if not res.accepted:
            return res
        if abs(req.command.velocities[0]) < 1e-9:
            self._cancel_motion()
            self._core.control_state = READY
            self._core.active_scope = None
            return res
        arm = req.arm_scope
        names, positions, _ = self._core.canonical(arm)
        idx = names.index(req.command.joint_names[0])
        duration = float(self.get_parameter("jog_step_duration").value)
        target = list(positions)
        target[idx] += req.command.velocities[0] * duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]
        future = self._trajectory[arm].send_goal_async(goal)
        future.add_done_callback(self._track_goal)
        self._core.active_scope = arm
        self._core.control_state = JOGGING
        self._active_command = f"jog {req.command.joint_names[0]}"
        return res

    def _track_goal(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                self._active_handles.append(handle)
        except Exception as exc:
            self._core.control_state = FAULT
            self._core.reason = f"controller dispatch failed: {exc}"

    def _cartesian_jog(self, req, res):
        stamp = req.command.header.stamp.sec + req.command.header.stamp.nanosec / 1e9
        linear, angular = req.command.twist.linear, req.command.twist.angular
        res.accepted, res.reason = self._core.validate_cartesian_jog(
            req.client_id, req.arm_scope, req.command.header.frame_id, req.tcp_link,
            [linear.x, linear.y, linear.z], [angular.x, angular.y, angular.z],
            stamp, self._now())
        if not res.accepted:
            return res
        try:
            self._tf_buffer.lookup_transform(
                req.command.header.frame_id, req.tcp_link, rclpy.time.Time())
        except TransformException as exc:
            res.accepted, res.reason = False, f"Cartesian jog TF unavailable: {exc}"
            return res
        if not self._servo_configured[req.arm_scope]:
            res.accepted, res.reason = False, "MoveIt Servo is not ready"
            return res
        self._servo_twist[req.arm_scope].publish(req.command)
        moving = any(abs(x) > 1e-12 for x in
                     (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z))
        if moving:
            self._servo_active.add(req.arm_scope)
            self._core.active_scope = req.arm_scope
            self._core.control_state = JOGGING
            self._active_command = f"Cartesian jog {req.tcp_link}"
        else:
            self._servo_active.discard(req.arm_scope)
            self._core.active_scope = None
            self._core.control_state = READY
            self._active_command = ""
        return res

    def _cancel_motion(self):
        for handle in self._active_handles:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self._active_handles.clear()
        self._core.active_scope = None
        self._active_command = ""
        self._progress = 0.0
        for arm in tuple(self._servo_active):
            zero = TwistStamped()
            zero.header.stamp = self.get_clock().now().to_msg()
            zero.header.frame_id = "world"
            self._servo_twist[arm].publish(zero)
        self._servo_active.clear()

    def _stop(self, req, res):
        self._core.control_state = STOPPING
        self._cancel_motion()
        self._core.control_state = READY if not self._core._is_read_only() else self._core.control_state
        self._core.reason = f"software stop: {req.reason or 'requested'}"
        res.stopped = True
        res.detail = self._core.reason
        return res

    def _position_goal(self, goal):
        if self._core._is_read_only():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _position_cancel(self, _goal):
        self._cancel_motion()
        return CancelResponse.ACCEPT

    def _cartesian_goal(self, goal):
        if self._core._is_read_only():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cartesian_cancel(self, _goal):
        self._cancel_motion()
        return CancelResponse.ACCEPT

    def _calibration_check_goal(self, _goal):
        if self._core._is_read_only():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _calibration_check_cancel(self, _goal):
        self._cancel_motion()
        return CancelResponse.ACCEPT

    def _physical_robot_status_ready(self):
        if self._mode == "mock":
            return True, "mock mode"
        for arm, label in ((LEFT, "left"), (RIGHT, "right")):
            item = self._robot_status.get(arm)
            if item is None:
                return False, f"{label} robot status unavailable"
            received_at, status = item
            if self._now() - received_at > 1.0:
                return False, f"{label} robot status is stale"
            if not status.motion_possible:
                return False, f"{label} motion is not enabled"
            if status.in_error:
                return False, f"{label} robot reports an error"
            if status.tp_enabled:
                return False, f"{label} teach pendant is enabled"
            if status.e_stopped:
                return False, f"{label} robot is e-stopped"
        return True, "both robots ready"

    async def _plan_pose_trajectory(
        self, scope, position_xyz, orientation_xyzw, velocity_scale=0.05
    ):
        if not self._move_group.server_is_ready():
            return False, "MoveIt move_action unavailable", 0, None
        group = "left_arm" if scope == LEFT else "right_arm"
        tcp_link = "left_flange" if scope == LEFT else "right_flange"
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = tcp_link
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE)
        primitive.dimensions = [0.002]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position_xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            orientation_xyzw
        )
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [pose]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = tcp_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = group
        move_goal.request.num_planning_attempts = 5
        move_goal.request.allowed_planning_time = 5.0
        move_goal.request.max_velocity_scaling_factor = velocity_scale
        move_goal.request.max_acceleration_scaling_factor = velocity_scale
        move_goal.request.goal_constraints = [constraints]
        move_goal.planning_options = PlanningOptions(plan_only=True)
        move_handle = await self._move_group.send_goal_async(move_goal)
        if not move_handle.accepted:
            return False, "MoveIt rejected verification planning request", 0, None
        self._active_handles.append(move_handle)
        move_result = await move_handle.get_result_async()
        if move_handle in self._active_handles:
            self._active_handles.remove(move_handle)
        response = move_result.result
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return (
                False,
                f"MoveIt verification planning failed ({response.error_code.val})",
                response.error_code.val,
                None,
            )
        return True, "planned", 0, response.planned_trajectory

    async def _plan_joint_trajectory(self, scope, names, positions, velocity_scale=0.05):
        if not self._move_group.server_is_ready():
            return False, "MoveIt move_action unavailable", 0, None
        group = "left_arm" if scope == LEFT else "right_arm"
        constraints = Constraints()
        for name, value in zip(names, positions):
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=value,
                    tolerance_above=1e-4,
                    tolerance_below=1e-4,
                    weight=1.0,
                )
            )
        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = group
        move_goal.request.num_planning_attempts = 5
        move_goal.request.allowed_planning_time = 5.0
        move_goal.request.max_velocity_scaling_factor = velocity_scale
        move_goal.request.max_acceleration_scaling_factor = velocity_scale
        move_goal.request.goal_constraints = [constraints]
        move_goal.planning_options = PlanningOptions(plan_only=True)
        move_handle = await self._move_group.send_goal_async(move_goal)
        if not move_handle.accepted:
            return False, "MoveIt rejected return-pose planning request", 0, None
        self._active_handles.append(move_handle)
        move_result = await move_handle.get_result_async()
        if move_handle in self._active_handles:
            self._active_handles.remove(move_handle)
        response = move_result.result
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return (
                False,
                f"MoveIt return-pose planning failed ({response.error_code.val})",
                response.error_code.val,
                None,
            )
        return True, "planned", 0, response.planned_trajectory

    async def _dispatch_trajectory(self, scope, trajectory):
        source = trajectory.joint_trajectory
        try:
            canonical = self._core.canonical(scope)[0]
            indices = [source.joint_names.index(name) for name in canonical]
        except (KeyError, ValueError):
            return False, "planned trajectory does not contain the expected arm joints"
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header = source.header
        goal.trajectory.joint_names = canonical
        for source_point in source.points:
            point = JointTrajectoryPoint()
            point.positions = [source_point.positions[index] for index in indices]
            if source_point.velocities:
                point.velocities = [source_point.velocities[index] for index in indices]
            if source_point.accelerations:
                point.accelerations = [
                    source_point.accelerations[index] for index in indices
                ]
            point.time_from_start = source_point.time_from_start
            goal.trajectory.points.append(point)
        controller_handle = await self._trajectory[scope].send_goal_async(goal)
        if not controller_handle.accepted:
            return False, "trajectory controller rejected verification motion"
        self._active_handles.append(controller_handle)
        execution = await controller_handle.get_result_async()
        if controller_handle in self._active_handles:
            self._active_handles.remove(controller_handle)
        if execution.status == GoalStatus.STATUS_CANCELED:
            return False, "verification motion canceled"
        if execution.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, "trajectory controller execution failed"
        return True, "execution complete"

    @staticmethod
    def _flange_orientation(transform):
        q = transform.transform.rotation
        return [q.x, q.y, q.z, q.w]

    async def _calibration_check(self, handle):
        req = handle.request
        result = CalibrationCheck.Result()

        def fail(reason, code=1):
            result.success = False
            result.code = code & 0xffff
            result.reason = reason
            self._finish_command()
            handle.abort()
            return result

        ok, reason = self._core.require_owner(req.client_id, BOTH, self._now())
        if not ok:
            return fail(reason)
        if self._core.active_scope is not None:
            return fail("another motion command is already active")
        try:
            per_arm_targets = {
                LEFT: verification_targets(
                    self._placement_data, "left_arm", req.clearance_m,
                    req.transit_height_m
                ),
                RIGHT: verification_targets(
                    self._placement_data, "right_arm", req.clearance_m,
                    req.transit_height_m
                ),
            }
        except WorkcellError as exc:
            return fail(str(exc))
        if not req.plan_only:
            ready, reason = self._physical_robot_status_ready()
            if not ready:
                return fail(reason)

        try:
            flange_transforms = {
                LEFT: self._tf_buffer.lookup_transform(
                    "world", "left_flange", rclpy.time.Time()
                ),
                RIGHT: self._tf_buffer.lookup_transform(
                    "world", "right_flange", rclpy.time.Time()
                ),
            }
        except TransformException as exc:
            return fail(f"flange TF unavailable: {exc}")
        start_joints = {
            arm: self._core.canonical(arm)[:2] for arm in (LEFT, RIGHT)
        }
        orientations = {
            arm: self._flange_orientation(flange_transforms[arm])
            for arm in (LEFT, RIGHT)
        }

        stages = []
        for arm, arm_name in ((LEFT, "left"), (RIGHT, "right")):
            for target in per_arm_targets[arm]:
                stages.append((arm, arm_name, target["point"], "transit", target["transit"]))
                stages.append((arm, arm_name, target["point"], "checkpoint", target["checkpoint"]))
                if not req.plan_only:
                    stages.append((arm, arm_name, target["point"], "retract", target["transit"]))
            if not req.plan_only:
                stages.append((arm, arm_name, "START", "return", None))

        total = len(stages)
        completed = 0
        self._core.active_scope = BOTH
        for arm, arm_name, point_name, phase, position in stages:
            if handle.is_cancel_requested:
                self._cancel_motion()
                self._finish_command()
                result.reason = "five-point verification canceled"
                handle.canceled()
                return result
            owned, reason = self._core.require_owner(req.client_id, BOTH, self._now())
            if not owned:
                return fail(reason)
            if not req.plan_only:
                ready, reason = self._physical_robot_status_ready()
                if not ready:
                    return fail(reason)

            self._core.control_state = PLANNING
            self._core.active_scope = BOTH
            self._active_command = f"five-point {arm_name} {point_name} {phase} planning"
            feedback = CalibrationCheck.Feedback(
                arm=arm_name,
                point=point_name,
                phase="planning " + phase,
                completed=completed,
                total=total,
                progress=float(completed) / total,
            )
            handle.publish_feedback(feedback)
            if phase == "return":
                names, positions = start_joints[arm]
                planned, reason, code, trajectory = await self._plan_joint_trajectory(
                    arm, names, positions
                )
            else:
                planned, reason, code, trajectory = await self._plan_pose_trajectory(
                    arm, position, orientations[arm]
                )
            if handle.is_cancel_requested:
                self._cancel_motion()
                self._finish_command()
                result.reason = "five-point verification canceled"
                handle.canceled()
                return result
            if not planned:
                return fail(f"{arm_name} {point_name} {phase}: {reason}", code or 1)
            if not req.plan_only:
                self._core.control_state = EXECUTING
                self._core.active_scope = BOTH
                self._active_command = f"five-point {arm_name} {point_name} {phase} executing"
                feedback.phase = "executing " + phase
                handle.publish_feedback(feedback)
                executed, reason = await self._dispatch_trajectory(arm, trajectory)
                if not executed:
                    if handle.is_cancel_requested:
                        self._finish_command()
                        result.reason = "five-point verification canceled"
                        handle.canceled()
                        return result
                    return fail(f"{arm_name} {point_name} {phase}: {reason}")
            completed += 1
            self._progress = float(completed) / total
            handle.publish_feedback(
                CalibrationCheck.Feedback(
                    arm=arm_name,
                    point=point_name,
                    phase="planned" if req.plan_only else "complete " + phase,
                    completed=completed,
                    total=total,
                    progress=self._progress,
                )
            )

        result.success = True
        result.reason = (
            "all five-point verification targets planned successfully"
            if req.plan_only
            else "left then right five-point verification complete; both arms returned"
        )
        self._finish_command()
        handle.succeed()
        return result

    async def _cartesian_pose(self, handle):
        req = handle.request
        result = CartesianPose.Result()
        now = self._now()
        if req.operation == CartesianPose.Goal.EXECUTE:
            stored = self._plans.get(req.plan_id)
            if not stored:
                result.reason = "unknown or expired plan_id"
                handle.abort()
                return result
            owner, scope, trajectory = stored
            ok, reason = self._core.require_owner(req.client_id, scope, now)
            if not ok or owner != req.client_id:
                result.reason = reason if not ok else "plan belongs to another client"
                handle.abort()
                return result
            return await self._execute_plan(
                handle, req.plan_id, scope, trajectory, result=result,
                feedback_type=CartesianPose.Feedback)

        stamp = req.target.header.stamp.sec + req.target.header.stamp.nanosec / 1e9
        p, q = req.target.pose.position, req.target.pose.orientation
        ok, reason = self._core.validate_cartesian_pose(
            req.client_id, req.arm_scope, req.target.header.frame_id, req.tcp_link,
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], stamp, now)
        if not ok:
            result.reason = reason
            handle.abort()
            return result
        try:
            # A lookup both verifies target-frame availability and rejects stale/missing TF.
            self._tf_buffer.lookup_transform("world", req.target.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            result.reason = f"target frame unavailable: {exc}"
            handle.abort()
            return result
        if req.operation == CartesianPose.Goal.VALIDATE:
            result.success, result.reason = True, "validated"
            handle.succeed()
            return result
        if not self._move_group.server_is_ready():
            result.reason = "MoveIt move_action unavailable"
            handle.abort()
            return result
        group = "left_arm" if req.arm_scope == LEFT else "right_arm"
        self._core.control_state, self._core.active_scope = PLANNING, req.arm_scope
        self._active_command = f"Cartesian plan {group}"
        handle.publish_feedback(CartesianPose.Feedback(
            phase="planning", progress=0.1, detail="submitted to MoveIt"))
        constraints = Constraints()
        position = PositionConstraint()
        position.header = req.target.header
        position.link_name = req.tcp_link
        primitive = SolidPrimitive(type=SolidPrimitive.SPHERE)
        primitive.dimensions = [max(req.position_tolerance, 0.001)]
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [req.target.pose]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header = req.target.header
        orientation.link_name = req.tcp_link
        orientation.orientation = req.target.pose.orientation
        tolerance = max(req.orientation_tolerance, 0.001)
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = group
        move_goal.request.num_planning_attempts = 5
        move_goal.request.allowed_planning_time = 5.0
        move_goal.request.max_velocity_scaling_factor = 0.1
        move_goal.request.max_acceleration_scaling_factor = 0.1
        move_goal.request.goal_constraints = [constraints]
        move_goal.planning_options = PlanningOptions(plan_only=True)
        move_handle = await self._move_group.send_goal_async(move_goal)
        if not move_handle.accepted:
            result.reason = "MoveIt rejected Cartesian planning request"
            self._finish_command(); handle.abort(); return result
        self._active_handles.append(move_handle)
        move_result = await move_handle.get_result_async()
        if handle.is_cancel_requested:
            result.reason = "Cartesian planning canceled"
            self._finish_command(); handle.canceled(); return result
        response = move_result.result
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            result.code = response.error_code.val & 0xffff
            result.reason = f"MoveIt Cartesian planning failed ({response.error_code.val})"
            self._finish_command(); handle.abort(); return result
        plan_id = str(uuid.uuid4())
        self._plans[plan_id] = (req.client_id, req.arm_scope, response.planned_trajectory)
        result.success, result.reason, result.plan_id = True, "Cartesian plan ready", plan_id
        handle.publish_feedback(CartesianPose.Feedback(
            phase="planned", progress=1.0, detail=plan_id))
        self._finish_command(); handle.succeed(); return result

    async def _position(self, handle):
        req = handle.request
        result = JointPosition.Result()
        now = self._now()
        if req.operation == JointPosition.Goal.EXECUTE:
            stored = self._plans.get(req.plan_id)
            if not stored:
                result.reason = "unknown or expired plan_id"
                handle.abort()
                return result
            owner, scope, trajectory = stored
            ok, reason = self._core.require_owner(req.client_id, scope, now)
            if not ok or owner != req.client_id:
                result.reason = reason if not ok else "plan belongs to another client"
                handle.abort()
                return result
            return await self._execute_plan(handle, req.plan_id, scope, trajectory)

        ok, reason, scope = self._core.validate_goal(
            req.client_id, req.planning_group, req.joint_names, req.positions, now)
        if not ok:
            result.reason = reason
            handle.abort()
            return result
        ordered = dict(zip(req.joint_names, req.positions))
        target = [ordered[n] for n in __import__("dual_crx_control.core", fromlist=["GROUP_JOINTS"]).GROUP_JOINTS[req.planning_group]]
        if scope in (LEFT, BOTH): self._target[LEFT] = target[:6]
        if scope == RIGHT: self._target[RIGHT] = target
        if scope == BOTH: self._target[RIGHT] = target[6:]
        if req.operation == JointPosition.Goal.VALIDATE:
            result.success, result.reason = True, "validated"
            handle.succeed()
            return result
        if not self._move_group.server_is_ready():
            result.reason = "MoveIt move_action unavailable"
            handle.abort()
            return result
        feedback = JointPosition.Feedback(phase="planning", progress=0.1, detail="submitted to MoveIt")
        handle.publish_feedback(feedback)
        self._core.control_state, self._core.active_scope = PLANNING, scope
        self._active_command = f"plan {req.planning_group}"
        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = req.planning_group
        move_goal.request.num_planning_attempts = 5
        move_goal.request.allowed_planning_time = 5.0
        constraint = Constraints()
        for name, value in zip(req.joint_names, req.positions):
            constraint.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=1e-4,
                tolerance_below=1e-4, weight=1.0))
        move_goal.request.goal_constraints = [constraint]
        move_goal.planning_options = PlanningOptions(plan_only=True)
        move_handle = await self._move_group.send_goal_async(move_goal)
        if not move_handle.accepted:
            result.reason = "MoveIt rejected planning request"
            self._finish_command()
            handle.abort()
            return result
        self._active_handles.append(move_handle)
        move_result = await move_handle.get_result_async()
        if handle.is_cancel_requested:
            result.reason = "planning canceled"
            self._finish_command()
            handle.canceled()
            return result
        response = move_result.result
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            result.code = response.error_code.val & 0xffff
            result.reason = f"MoveIt planning failed ({response.error_code.val})"
            self._finish_command()
            handle.abort()
            return result
        plan_id = str(uuid.uuid4())
        self._plans[plan_id] = (req.client_id, scope, response.planned_trajectory)
        result.success, result.reason, result.plan_id = True, "plan ready", plan_id
        handle.publish_feedback(JointPosition.Feedback(phase="planned", progress=1.0, detail=plan_id))
        self._finish_command()
        handle.succeed()
        return result

    async def _execute_plan(self, handle, plan_id, scope, trajectory, result=None,
                            feedback_type=JointPosition.Feedback):
        result = result or JointPosition.Result()
        result.plan_id = plan_id
        self._core.control_state, self._core.active_scope = EXECUTING, scope
        self._active_command = f"execute {plan_id}"
        handle.publish_feedback(feedback_type(phase="executing", progress=0.1,
                                              detail="dispatching approved plan"))
        source = trajectory.joint_trajectory
        sends = []
        for arm in self._core.arms(scope):
            canonical = self._core.canonical(arm)[0]
            indices = [source.joint_names.index(name) for name in canonical]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.header = source.header
            goal.trajectory.joint_names = canonical
            for source_point in source.points:
                point = JointTrajectoryPoint()
                point.positions = [source_point.positions[i] for i in indices]
                if source_point.velocities:
                    point.velocities = [source_point.velocities[i] for i in indices]
                if source_point.accelerations:
                    point.accelerations = [source_point.accelerations[i] for i in indices]
                point.time_from_start = source_point.time_from_start
                goal.trajectory.points.append(point)
            sends.append(await self._trajectory[arm].send_goal_async(goal))
        if not all(item.accepted for item in sends):
            result.reason = "one or more trajectory controllers rejected approved plan"
            self._finish_command()
            handle.abort()
            return result
        self._active_handles.extend(sends)
        executions = [await item.get_result_async() for item in sends]
        if handle.is_cancel_requested or any(x.status == GoalStatus.STATUS_CANCELED for x in executions):
            result.reason = "execution canceled/stopped"
            self._finish_command()
            handle.canceled()
            return result
        result.success = all(x.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL for x in executions)
        result.code = 0 if result.success else 1
        result.reason = "execution complete" if result.success else "trajectory controller execution failed"
        self._finish_command()
        if result.success: handle.succeed()
        else: handle.abort()
        return result

    def _finish_command(self):
        self._active_handles.clear()
        self._core.active_scope = None
        self._core.control_state = READY
        self._active_command = ""
        self._progress = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ControlServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
