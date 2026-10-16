"""Command-line client for the DragRaceStart action."""

# References (full entries: docs/references.md at the repository root):
#   [R10] ROS 2 Jazzy documentation: action client
# More details: README.md of this package

import argparse

import rclpy
from learn2race_dragster_interfaces.action import DragRaceStart
from rclpy.action import ActionClient
from rclpy.node import Node


class DragRaceStartClient(Node):
    """Send one drag race start goal and print feedback/result."""

    def __init__(self, args) -> None:
        super().__init__("drag_race_start_client")
        self.args = args
        self.client = ActionClient(self, DragRaceStart, args.action_name)

    def send_goal(self) -> None:
        goal = DragRaceStart.Goal()
        goal.threshold_ratio = self.args.threshold
        goal.min_delta_ratio = self.args.min_delta
        goal.stable_baseline_s = self.args.baseline_s
        goal.timeout_s = self.args.timeout_s
        goal.edge_mode = -1 if self.args.falling_edge else 1
        goal.publish_launch_command = self.args.launch
        goal.launch_linear_mps = self.args.launch_linear
        goal.launch_angular_radps = self.args.launch_angular
        goal.launch_duration_s = self.args.launch_duration

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"{feedback.state}: light={feedback.light_ratio:.3f}, "
            f"baseline={feedback.baseline_ratio:.3f}, elapsed={feedback.elapsed_s:.2f}s"
        )

    def _goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Drag race start goal rejected.")
            rclpy.try_shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future) -> None:
        result = future.result().result
        self.get_logger().info(
            f"started={result.started}, timed_out={result.timed_out}, "
            f"cancelled={result.cancelled}, rising_edge={result.rising_edge}, "
            f"trigger={result.trigger_ratio:.3f}, "
            f"message='{result.message}'"
        )
        rclpy.try_shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="Trigger a drag race start action.")
    parser.add_argument("--action-name", default="/dragster/start_race")
    parser.add_argument("--threshold", type=float, default=0.65)
    parser.add_argument("--min-delta", type=float, default=0.15)
    parser.add_argument("--baseline-s", type=float, default=0.5)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--falling-edge", action="store_true")
    parser.add_argument("--launch", action="store_true")
    parser.add_argument("--launch-linear", type=float, default=0.6)
    parser.add_argument("--launch-angular", type=float, default=0.0)
    parser.add_argument("--launch-duration", type=float, default=0.75)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = DragRaceStartClient(args)
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
