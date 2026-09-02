#!/usr/bin/env python3
"""Plan every reviewed five-point target through the public API without motion."""

import sys
import time
import uuid

import rclpy
from dual_crx_interfaces.action import CalibrationCheck
from dual_crx_interfaces.msg import SystemState
from dual_crx_interfaces.srv import AcquireControl, GetWorkcellInfo, Heartbeat, ReleaseControl
from rclpy.action import ActionClient
from rclpy.node import Node


BOTH = 3


class SmokeClient(Node):
    def __init__(self):
        super().__init__("five_point_plan_smoke")
        self.client_id = "five-point-smoke-" + str(uuid.uuid4())[:8]
        self.state = None
        self.feedback_count = 0
        self.create_subscription(SystemState, "/dual_crx/state", self._state, 10)
        self.acquire = self.create_client(AcquireControl, "/dual_crx/acquire_control")
        self.heartbeat = self.create_client(Heartbeat, "/dual_crx/heartbeat")
        self.release = self.create_client(ReleaseControl, "/dual_crx/release_control")
        self.info = self.create_client(GetWorkcellInfo, "/dual_crx/workcell_info")
        self.check = ActionClient(self, CalibrationCheck, "/dual_crx/calibration_check")

    def _state(self, msg):
        self.state = msg

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def service(self, client, request, timeout=5.0):
        self.wait(client.service_is_ready, timeout, client.srv_name)
        future = client.call_async(request)
        self.wait(future.done, timeout, client.srv_name + " response")
        return future.result()

    def feedback(self, message):
        self.feedback_count += 1
        feedback = message.feedback
        print(
            f"PLAN {feedback.completed}/{feedback.total} "
            f"{feedback.arm} {feedback.point} {feedback.phase}"
        )

    def run(self):
        self.wait(
            lambda: self.state is not None and self.state.complete and self.state.fresh,
            90.0,
            "complete fresh state",
        )
        info = self.service(self.info, GetWorkcellInfo.Request())
        if not info.loaded or not info.valid:
            raise RuntimeError("launch did not load a valid reviewed profile")
        print("PROFILE", info.profile_name, info.sha256[:12], info.placement_file)
        acquired = self.service(
            self.acquire,
            AcquireControl.Request(
                client_id=self.client_id,
                source_type="TEST",
                arm_scope=BOTH,
                requested_mode="CALIBRATION_CHECK",
                lease_duration=5.0,
            ),
        )
        if not acquired.accepted:
            raise RuntimeError("BOTH acquisition failed: " + acquired.reason)
        self.wait(self.check.server_is_ready, 10.0, "calibration-check action")
        sent = self.check.send_goal_async(
            CalibrationCheck.Goal(
                client_id=self.client_id,
                plan_only=True,
                clearance_m=0.020,
                transit_height_m=0.080,
            ),
            feedback_callback=self.feedback,
        )
        self.wait(sent.done, 5.0, "goal acceptance")
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError("calibration-check goal rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 180.0
        next_heartbeat = 0.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= next_heartbeat:
                self.heartbeat.call_async(
                    Heartbeat.Request(
                        client_id=self.client_id,
                        arm_scope=BOTH,
                        lease_duration=5.0,
                    )
                )
                next_heartbeat = time.monotonic() + 1.0
        if not result_future.done():
            handle.cancel_goal_async()
            raise RuntimeError("five-point plan-only timed out")
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(result.reason)
        if self.feedback_count < 20:
            raise RuntimeError("incomplete five-point planning feedback")
        print("PASS", result.reason)
        released = self.service(
            self.release,
            ReleaseControl.Request(client_id=self.client_id, arm_scope=BOTH),
        )
        if not released.released:
            raise RuntimeError("BOTH release failed: " + released.reason)


def main():
    rclpy.init()
    client = SmokeClient()
    try:
        client.run()
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("FAIL five-point plan smoke:", exc, file=sys.stderr)
        raise
