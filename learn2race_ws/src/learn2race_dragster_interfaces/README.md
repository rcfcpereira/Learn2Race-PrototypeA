# learn2race_dragster_interfaces

ROS 2 action definition for starting a dragster run from the start light.

## `action/DragRaceStart.action`

Goal: `threshold_ratio`, `min_delta_ratio`, `stable_baseline_s`, `timeout_s`,
`edge_mode`, and optionally a launch command (`publish_launch_command`,
`launch_linear_mps`, `launch_angular_radps`, `launch_duration_s`). Result and feedback
fields are defined in the file. Served by `learn2race_dragster`'s
`drag_race_start_action_server`.

## References and more details

- [R2] Dragster rules (start light); [R10] ROS 2 actions.
- Full entries: [`docs/references.md`](../../../docs/references.md);
  `learn2race_dragster/README.md`.
