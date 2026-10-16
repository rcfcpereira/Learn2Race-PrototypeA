# learn2race_competition

Competition logic for Prototype A: the TrackBotGP race referee (start light, laps,
pit stop), the competition state machine, the rule LED colour code, and the dragster
referee.

## Nodes

| Executable | Module | Role |
|---|---|---|
| `gp_referee` | `gp_referee_node.py` + `gp_policy.py` | TrackBotGP: start light → `start_event`; finish line → `lap_event`; pit split → `pit_lane`; stop point → `pit_entry_event`; publishes `line/branch_preference` and `line/speed_limit` for the follower |
| `competition_fsm` | `competition_fsm_node.py` | `READY → LAP1 → LAP2 → PIT → LAP3 → FINISH`, `SAFE_STOP` from any state; arms the drive mux; lap timeout (120 s) and pit hold (≥ 5 s) |
| `led_signalling` | `led_signalling_node.py` | Race state → `leds/set_color` using the rule colour code (green, blue, 3 flashes, yellow, orange, white flashing) |
| `dragster_referee` | `dragster_referee_node.py` + `dragster_policy.py` | Dragster start light and finish band |
| `lap_counter` | `lap_counter_node.py` | Earlier finish-line detector; superseded by `gp_referee` in TrackBotGP mode |

`gp_policy.py` and `dragster_policy.py` contain no ROS code, so the race decisions are
unit-tested on their own (`test/test_gp_policy.py`, `test/test_dragster_policy.py`):

    python3 -m pytest -q test

## How a TrackBotGP race is refereed (summary)

- **Start:** LTR303 light ≥ 1.5 × ambient and ≥ 50 counts above it.
- **Laps:** odometry distance + camera events. A band is a finish line only with the
  pit split or the chequered marker within `pair_window_m`, after most of a lap, and
  not while the robot spins on the spot. With `expected_lap_m` set, a missed finish
  line is counted after 1.35 laps.
- **Pit:** armed `pit_arm_before_finish_m` before the end of lap 2; slow approach;
  the next split is the pit lane; stop 0.5 m in; the FSM holds 5.3 s; lap 2 ends at
  the pit exit.
- **Per-track settings:** `learn2race_bringup/config/track_<name>.yaml`.

Every guard in `gp_policy.py` is commented with the run that motivated it.

## References and more details

- [R1] TrackBotGP rules 2026 (repository root PDF): race, pit stop, lap limit, LED
  code (V.3).
- [R2] Dragster rules; [R50] Harel, statecharts (state machine design);
  [R67] LTR-303ALS-01 datasheet (start light).
- Full entries: [`docs/references.md`](../../../docs/references.md).
