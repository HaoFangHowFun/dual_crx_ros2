#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__("dual_crx_joint_state_merger")

        self.declare_parameter("left_topic", "/left_arm/joint_states")
        self.declare_parameter("right_topic", "/right_arm/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("state_timeout", 0.5)

        self._joint_states = {}

        left_topic = self.get_parameter("left_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_period = (
            self.get_parameter("publish_period").get_parameter_value().double_value
        )
        self._state_timeout = self.get_parameter("state_timeout").value

        self.create_subscription(JointState, left_topic, self._left_callback, 10)
        self.create_subscription(JointState, right_topic, self._right_callback, 10)
        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_timer(publish_period, self._publish_merged_joint_states)

    def _left_callback(self, msg: JointState) -> None:
        self._joint_states["left"] = msg

    def _right_callback(self, msg: JointState) -> None:
        self._joint_states["right"] = msg

    def _publish_merged_joint_states(self) -> None:
        # Never expose a partial dual-arm state during startup. Consumers which need
        # per-arm age still subscribe to the original namespaced topics.
        if set(self._joint_states) != {"left", "right"}:
            return
        now = self.get_clock().now()
        for joint_state in self._joint_states.values():
            stamp = rclpy.time.Time.from_msg(joint_state.header.stamp)
            if stamp.nanoseconds and (now - stamp).nanoseconds / 1e9 > self._state_timeout:
                return
        merged = JointState()
        merged.header.stamp = now.to_msg()

        for key in ("left", "right"):
            joint_state = self._joint_states.get(key)
            merged.name.extend(joint_state.name)
            merged.position.extend(joint_state.position)
            merged.velocity.extend(joint_state.velocity)
            merged.effort.extend(joint_state.effort)

        self._publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
