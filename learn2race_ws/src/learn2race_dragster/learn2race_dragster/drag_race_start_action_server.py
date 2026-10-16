"""Action server that starts a dragster run from a light threshold."""

# References (full entries: docs/references.md at the repository root):
#   [R2]  Dragster rules: light-triggered start
#   [R10] ROS 2 Jazzy documentation: action server
# More details: README.md of this package

import time
from statistics import fmean
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from learn2race_dragster_interfaces.action import DragRaceStart
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32


class DragRaceStartActionServer(Node):
    """Wait for a light change, then optionally publish a launch command."""

    def __init__(self) -> None:
        super().__init__("drag_race_start_action_server")

        self.declare_parameter("action_name", "/dragster/start_race")
        self.declare_parameter("light_topic", "/dragster/light_ratio")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("default_threshold_ratio", 0.65)
        self.declare_parameter("default_min_delta_ratio", 0.15)
        self.declare_parameter("default_stable_baseline_s", 0.5)
        self.declare_parameter("default_timeout_s", 10.0)
        self.declare_parameter("default_rising_edge", True)
        self.declare_parameter("trigger_hold_s", 0.03)
        self.declare_parameter("max_sample_age_s", 0.25)
        self.declare_parameter("command_rate_hz", 50.0)

        self.latest_light: Optional[float] = None
        self.latest_light_time = self.get_clock().now()
        self.callback_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("light_topic").value),
            self._light_callback,
            20,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            DragRaceStart,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _light_callback(self, msg: Float32) -> None:
        self.latest_light = max(0.0, min(1.0, float(msg.data)))
        self.latest_light_time = self.get_clock().now()

    def _execute_callback(self, goal_handle) -> DragRaceStart.Result:
        goal = goal_handle.request
        result = DragRaceStart.Result()
        feedback = DragRaceStart.Feedback()

        threshold = self._threshold_or_default(goal.threshold_ratio)
        min_delta = self._ratio_goal_or_default(
            goal.min_delta_ratio, "default_min_delta_ratio"
        )
        baseline_s = self._goal_or_default(
            goal.stable_baseline_s, "default_stable_baseline_s"
        )
        timeout_s = self._goal_or_default(goal.timeout_s, "default_timeout_s")
        rising_edge = self._edge_mode_to_bool(goal.edge_mode)
        result.rising_edge = bool(rising_edge)
        max_sample_age_s = max(0.0, float(self.get_parameter("max_sample_age_s").value))

        start_clock = self.get_clock().now()
        samples = []
        while (self.get_clock().now() - start_clock).nanoseconds / 1e9 < baseline_s:
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result, "Cancelled during baseline.")
            if self.latest_light is not None and self._light_sample_fresh(max_sample_age_s):
                samples.append(self.latest_light)
                feedback.light_ratio = self.latest_light
            feedback.armed = False
            feedback.state = "baseline"
            feedback.elapsed_s = (self.get_clock().now() - start_clock).nanoseconds / 1e9
            goal_handle.publish_feedback(feedback)
            time.sleep(0.02)

        if not samples:
            goal_handle.abort()
            result.message = "No fresh light samples received before baseline timeout."
            return result

        baseline = fmean(samples)
        result.baseline_ratio = float(baseline)
        armed_clock = self.get_clock().now()
        trigger_started_at = None

        while (self.get_clock().now() - armed_clock).nanoseconds / 1e9 < timeout_s:
            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result, "Cancelled while armed.")

            light = self.latest_light
            elapsed_s = (self.get_clock().now() - armed_clock).nanoseconds / 1e9
            if light is None or not self._light_sample_fresh(max_sample_age_s):
                feedback.armed = False
                feedback.state = "waiting_for_light"
                feedback.elapsed_s = float(elapsed_s)
                goal_handle.publish_feedback(feedback)
                time.sleep(0.02)
                continue

            triggered = self._is_triggered(
                light, baseline, threshold, min_delta, rising_edge
            )
            if triggered and trigger_started_at is None:
                trigger_started_at = self.get_clock().now()
            elif not triggered:
                trigger_started_at = None

            feedback.light_ratio = float(light)
            feedback.baseline_ratio = float(baseline)
            feedback.elapsed_s = float(elapsed_s)
            feedback.armed = True
            feedback.state = "triggered" if triggered else "armed"
            goal_handle.publish_feedback(feedback)

            if trigger_started_at is not None:
                hold_s = (self.get_clock().now() - trigger_started_at).nanoseconds / 1e9
                if hold_s >= float(self.get_parameter("trigger_hold_s").value):
                    result.started = True
                    result.trigger_ratio = float(light)
                    result.start_time = self.get_clock().now().to_msg()
                    if goal.publish_launch_command:
                        self._publish_launch(goal, goal_handle, result)
                    goal_handle.succeed()
                    result.message = "Race start detected."
                    return result

            time.sleep(0.01)

        result.timed_out = True
        result.message = "Timed out waiting for dragster start light."
        goal_handle.abort()
        return result

    def _goal_or_default(self, value: float, parameter_name: str) -> float:
        if float(value) > 0.0:
            return float(value)
        return float(self.get_parameter(parameter_name).value)

    def _ratio_goal_or_default(self, value: float, parameter_name: str) -> float:
        return max(0.0, min(1.0, self._goal_or_default(value, parameter_name)))

    def _threshold_or_default(self, value: float) -> float:
        value = float(value)
        if value < 0.0:
            value = float(self.get_parameter("default_threshold_ratio").value)
        return max(0.0, min(1.0, value))

    def _edge_mode_to_bool(self, edge_mode: int) -> bool:
        edge_mode = int(edge_mode)
        if edge_mode > 0:
            return True
        if edge_mode < 0:
            return False
        return bool(self.get_parameter("default_rising_edge").value)

    def _light_sample_fresh(self, max_sample_age_s: float) -> bool:
        age_s = (self.get_clock().now() - self.latest_light_time).nanoseconds / 1e9
        return age_s <= max_sample_age_s

    @staticmethod
    def _is_triggered(
        light: float,
        baseline: float,
        threshold: float,
        min_delta: float,
        rising_edge: bool,
    ) -> bool:
        absolute_ok = threshold <= 0.0
        if rising_edge:
            absolute_ok = absolute_ok or light >= threshold
            return absolute_ok and light >= baseline + min_delta
        absolute_ok = absolute_ok or light <= threshold
        return absolute_ok and light <= baseline - min_delta

    def _publish_launch(self, goal, goal_handle, result: DragRaceStart.Result) -> None:
        del result
        duration_s = max(0.0, float(goal.launch_duration_s))
        if duration_s <= 0.0:
            return

        rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        command = Twist()
        command.linear.x = float(goal.launch_linear_mps)
        command.angular.z = float(goal.launch_angular_radps)
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < duration_s:
            if goal_handle.is_cancel_requested:
                break
            self.cmd_pub.publish(command)
            time.sleep(1.0 / rate_hz)
        self.cmd_pub.publish(Twist())

    def _cancel(self, goal_handle, result: DragRaceStart.Result, message: str):
        self.cmd_pub.publish(Twist())
        result.cancelled = True
        result.message = message
        goal_handle.canceled()
        return result

    def destroy_node(self) -> bool:
        self.action_server.destroy()
        self.cmd_pub.publish(Twist())
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DragRaceStartActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
