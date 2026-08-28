#!/usr/bin/env python3
"""Non-interactive live public-API acceptance client (mock only)."""

import math
import sys
import time

import rclpy
from control_msgs.msg import JointJog
from dual_crx_interfaces.action import JointPosition
from dual_crx_interfaces.action import CartesianPose
from dual_crx_interfaces.msg import SystemState
from dual_crx_interfaces.srv import AcquireControl, CartesianJog, JointJog as JointJogSrv
from dual_crx_interfaces.srv import ReleaseControl, SoftwareStop
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

LEFT, RIGHT, BOTH = 1, 2, 3


class Client(Node):
    def __init__(self):
        super().__init__("phase1_mock_acceptance_client")
        self.state = None
        self.create_subscription(SystemState, "/dual_crx/state", self._state, 10)
        self.acquire = self.create_client(AcquireControl, "/dual_crx/acquire_control")
        self.release = self.create_client(ReleaseControl, "/dual_crx/release_control")
        self.jog = self.create_client(JointJogSrv, "/dual_crx/jog")
        self.cartesian_jog = self.create_client(CartesianJog, "/dual_crx/cartesian_jog")
        self.stop = self.create_client(SoftwareStop, "/dual_crx/stop")
        self.position = ActionClient(self, JointPosition, "/dual_crx/joint_position")
        self.cartesian = ActionClient(self, CartesianPose, "/dual_crx/cartesian_pose")

    def _state(self, msg): self.state = msg

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate(): return
        raise RuntimeError(f"timeout waiting for {label}")

    def service(self, client, request, timeout=5.0):
        self.wait(client.service_is_ready, timeout, client.srv_name)
        future = client.call_async(request)
        self.wait(future.done, timeout, client.srv_name + " response")
        return future.result()

    def action(self, goal, timeout=20.0):
        self.wait(self.position.server_is_ready, 10.0, "joint-position action")
        sent = self.position.send_goal_async(goal)
        self.wait(sent.done, 5.0, "goal acceptance")
        handle = sent.result()
        if not handle.accepted: raise RuntimeError("joint-position goal rejected at transport boundary")
        result = handle.get_result_async()
        self.wait(result.done, timeout, "joint-position result")
        return result.result().result

    def cartesian_action(self, goal, timeout=20.0):
        self.wait(self.cartesian.server_is_ready, 10.0, "Cartesian pose action")
        sent = self.cartesian.send_goal_async(goal)
        self.wait(sent.done, 5.0, "Cartesian goal acceptance")
        handle = sent.result()
        if not handle.accepted: raise RuntimeError("Cartesian goal rejected at transport boundary")
        result = handle.get_result_async()
        self.wait(result.done, timeout, "Cartesian pose result")
        return result.result().result


def acquire(client, scope):
    result = client.service(client.acquire, AcquireControl.Request(
        client_id="phase1-test", source_type="TEST", arm_scope=scope,
        requested_mode="JOINT", lease_duration=5.0))
    if not result.accepted: raise RuntimeError("acquire rejected: " + result.reason)


def release(client, scope):
    result = client.service(client.release, ReleaseControl.Request(
        client_id="phase1-test", arm_scope=scope))
    if not result.released: raise RuntimeError("release rejected: " + result.reason)


def stop(client, reason):
    result = client.service(client.stop, SoftwareStop.Request(
        client_id="phase1-test", arm_scope=BOTH, reason=reason))
    if not result.stopped: raise RuntimeError("stop rejected")


def jog(client, arm, joint, velocity):
    before = list(client.state.left_joints.position if arm == LEFT else client.state.right_joints.position)
    command = JointJog(joint_names=[joint], velocities=[velocity])
    command.header.stamp = client.get_clock().now().to_msg()
    result = client.service(client.jog, JointJogSrv.Request(
        client_id="phase1-test", arm_scope=arm, command=command))
    if not result.accepted: raise RuntimeError("jog rejected: " + result.reason)
    time.sleep(0.3)
    stop(client, "integration jog release")
    client.wait(lambda: client.state is not None, 2.0, "post-jog state")
    rclpy.spin_once(client, timeout_sec=0.3)
    after = list(client.state.left_joints.position if arm == LEFT else client.state.right_joints.position)
    index = int(joint[-1]) - 1
    if abs(after[index] - before[index]) < 1e-4:
        raise RuntimeError(f"{joint} did not move: {before[index]} -> {after[index]}")
    print(f"EVIDENCE jog {joint}: {before[index]:.6f} -> {after[index]:.6f}")


def plan_execute(client, group, scope, positions):
    if group == "left_arm": names = [f"left_J{i}" for i in range(1, 7)]
    elif group == "right_arm": names = [f"right_J{i}" for i in range(1, 7)]
    else: names = [f"left_J{i}" for i in range(1, 7)] + [f"right_J{i}" for i in range(1, 7)]
    plan = client.action(JointPosition.Goal(
        client_id="phase1-test", planning_group=group,
        operation=JointPosition.Goal.PLAN, joint_names=names, positions=positions), 30.0)
    if not plan.success: raise RuntimeError(f"{group} plan failed: {plan.reason}")
    execution = client.action(JointPosition.Goal(
        client_id="phase1-test", planning_group=group,
        operation=JointPosition.Goal.EXECUTE, plan_id=plan.plan_id), 40.0)
    if not execution.success: raise RuntimeError(f"{group} execute failed: {execution.reason}")
    print(f"EVIDENCE {group}: plan={plan.plan_id} execute={execution.reason}")


def servo_jog(client, arm, velocity):
    pose = client.state.left_end_effector if arm == LEFT else client.state.right_end_effector
    before = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    tcp = "left_flange" if arm == LEFT else "right_flange"
    accepted = False
    for _ in range(15):
        command = TwistStamped(); command.header.frame_id = "world"
        command.header.stamp = client.get_clock().now().to_msg()
        command.twist.linear.z = velocity
        response = client.service(client.cartesian_jog, CartesianJog.Request(
            client_id="phase1-test", arm_scope=arm, tcp_link=tcp, command=command))
        if response.accepted:
            accepted = True
        elif "not ready" not in response.reason:
            raise RuntimeError("Cartesian jog rejected: " + response.reason)
        time.sleep(0.05)
        rclpy.spin_once(client, timeout_sec=0.01)
    if not accepted: raise RuntimeError("MoveIt Servo never became ready")
    stop(client, "Cartesian jog release")
    client.wait(lambda: client.state is not None, 2.0, "post-Servo state")
    rclpy.spin_once(client, timeout_sec=0.3)
    pose = client.state.left_end_effector if arm == LEFT else client.state.right_end_effector
    after = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(after, before)))
    if distance < 1e-4:
        raise RuntimeError(f"Cartesian Servo arm {arm} did not move: {before} -> {after}")
    print(f"EVIDENCE Cartesian Servo arm={arm}: {before} -> {after}")


def main():
    rclpy.init(); client = Client()
    try:
        client.wait(lambda: client.state is not None and client.state.complete and client.state.fresh
                    and client.state.control_state == SystemState.READY, 90.0,
                    "complete fresh READY state")
        start_left = list(client.state.left_joints.position)
        start_right = list(client.state.right_joints.position)
        print("EVIDENCE controllers:", client.state.left_controller_state,
              client.state.right_controller_state)
        print("EVIDENCE start left:", start_left)
        print("EVIDENCE start right:", start_right)

        # Ownership conflict and atomic BOTH rejection.
        acquire(client, LEFT)
        conflict = client.service(client.acquire, AcquireControl.Request(
            client_id="conflict", source_type="TEST", arm_scope=BOTH,
            requested_mode="JOINT", lease_duration=1.0))
        if conflict.accepted: raise RuntimeError("conflicting BOTH lease was accepted")
        print("EVIDENCE conflict rejection:", conflict.reason)
        jog(client, LEFT, "left_J6", math.radians(2.0))
        jog(client, LEFT, "left_J6", -math.radians(2.0))
        release(client, LEFT)

        acquire(client, RIGHT)
        jog(client, RIGHT, "right_J6", math.radians(2.0))
        jog(client, RIGHT, "right_J6", -math.radians(2.0))
        release(client, RIGHT)

        # Current-state targets exercise no-op/minimal collision-aware plans.
        for group, scope, target in (("left_arm", LEFT, list(client.state.left_joints.position)),
                                     ("right_arm", RIGHT, list(client.state.right_joints.position)),
                                     ("both_arms", BOTH, list(client.state.left_joints.position) +
                                      list(client.state.right_joints.position))):
            acquire(client, scope); plan_execute(client, group, scope, target); release(client, scope)

        # Structured invalid target rejection before MoveIt execution.
        acquire(client, LEFT)
        invalid = client.action(JointPosition.Goal(
            client_id="phase1-test", planning_group="left_arm",
            operation=JointPosition.Goal.PLAN,
            joint_names=["left_J1"] * 6, positions=[0.0] * 6), 5.0)
        if invalid.success or "exactly once" not in invalid.reason:
            raise RuntimeError("duplicate target was not deterministically rejected")
        print("EVIDENCE invalid rejection:", invalid.reason)
        release(client, LEFT)

        # Cartesian current-pose plan validates TF/TCP/IK and executes a no-op plan.
        client.wait(lambda: bool(client.state.left_end_effector.header.frame_id), 10.0,
                    "left end-effector TF")
        acquire(client, LEFT)
        target = client.state.left_end_effector
        target.header.stamp = client.get_clock().now().to_msg()
        cart_plan = client.cartesian_action(CartesianPose.Goal(
            client_id="phase1-test", arm_scope=LEFT,
            operation=CartesianPose.Goal.PLAN, tcp_link="left_flange", target=target,
            position_tolerance=0.002, orientation_tolerance=0.01), 30.0)
        if not cart_plan.success:
            raise RuntimeError("Cartesian plan failed: " + cart_plan.reason)
        cart_execute = client.cartesian_action(CartesianPose.Goal(
            client_id="phase1-test", arm_scope=LEFT,
            operation=CartesianPose.Goal.EXECUTE, plan_id=cart_plan.plan_id), 40.0)
        if not cart_execute.success:
            raise RuntimeError("Cartesian execute failed: " + cart_execute.reason)
        print("EVIDENCE Cartesian left current-pose:", cart_plan.plan_id,
              cart_execute.reason)
        release(client, LEFT)

        for arm in (LEFT, RIGHT):
            acquire(client, arm)
            servo_jog(client, arm, 0.005)
            servo_jog(client, arm, -0.005)
            release(client, arm)
        print("PASS live public API mock integration")
    finally:
        stop(client, "integration cleanup")
        client.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    try: main()
    except Exception as exc:
        print("FAIL live integration:", exc, file=sys.stderr)
        raise
