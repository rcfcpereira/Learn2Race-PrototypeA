#!/bin/bash

# References (full entries: docs/references.md at the repository root):
#   [R91] tmux: tmux sessions
#   [R10] ROS 2 Jazzy documentation: ros2 launch, colcon
#   [R75] WS2813 datasheet, Linux spidev, rpi_ws281x: blanking the WS2813 stick
# More details: learn2race_ws/README.md

# Prototype A run helper. Everything runs inside tmux, so a dropped SSH
# session no longer kills a run or leaves orphan nodes holding the camera.
#
#   ~/robot.sh practice [speed] [follower args...]   line following
#   ~/robot.sh race [trackbotgp|dragster] [speed] [follower args...]
#     follower args are launch arguments, e.g. curve_gain:=0.7
#     light:=N sets the track light (0 = off, default 200)
#     track:=monza|portimao|suzuka picks the layout settings (default portimao)
#     lidar:=true|false, display:=true|false   bringup subsystems (default: off in a race)
#     sensor_host:=esp32|pi   who reads the board sensors (default pi; must match the firmware)
#     preview:=on   keep the web preview encoding all run (a local viewer)
#     target_line_distance_mm:=100 roi_top_ratio:=0.45
#   ~/robot.sh attach                      watch the logs (leave: Ctrl+b then d)
#   ~/robot.sh stop                        stop everything, including orphans
#   ~/robot.sh status                      what runs, load, memory, power flags
#   ~/robot.sh build [packages...]         colcon build, refused while running
#   ~/robot.sh graph LABEL                 node/topic graph of the running system
#   ~/robot.sh topics [SECONDS]            rate, timing and size of the main topics
#   Every run also logs CPU and memory per process (pidstat.txt) and temperature,
#   load and throttling (thermal.csv) in its run folder.
#
# After an SSH drop, just reconnect: the run is still going; `attach` to see it.

SESSION=robot
WS=~/learn2race_ws
ROS="source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash"
TOOLS=~/learn2race_tools      # export_ros_graph.py, topic_stats.py (from tools)

running() { pgrep -f "[r]os2 launch" >/dev/null; }

health() {
  # One line per second, synced, so the tail survives a hard freeze:
  # throttled != 0x0 means the 5 V supply dipped; low mem/high swap means
  # the Pi was swapping to the SD card.
  pgrep -f "[h]ealthlog.sh" >/dev/null && return
  cat > ~/healthlog.sh <<'EOF'
#!/bin/bash
f=~/health.log
echo "=== boot $(uptime -s)" >> $f
while true; do
  echo "$(date +%T) $(vcgencmd get_throttled) $(vcgencmd measure_temp)" \
       "load=$(cut -d' ' -f1 /proc/loadavg)" \
       "mem=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)MB" \
       "swap=$(awk '/SwapTotal/{t=$2}/SwapFree/{f=$2}END{print int((t-f)/1024)}' /proc/meminfo)MB" \
       "mempsi=$(awk -F'[= ]' '/^some/{print $3}' /proc/pressure/memory 2>/dev/null)" >> $f
  sync $f
  sleep 1
done
EOF
  chmod +x ~/healthlog.sh
  (setsid nohup ~/healthlog.sh >/dev/null 2>&1 < /dev/null &)
}

stop_all() {
  tmux kill-session -t $SESSION 2>/dev/null
  pkill -INT -f "[r]os2 launch"; sleep 3
  pkill -KILL -f "[r]os2 launch"
  pkill -KILL -f "[l]earn2race_ws/install"
  pkill -KILL -x micro_ros_agent
  pkill -KILL -f "[c]amera_node"
  pkill -f "[r]os2cli.daemon"
  pkill -f "[p]idstat -h -u -r -l"
  pkill -f "[e]cho epoch,temp_c,load1"
  pkill -f "[c]url -s -o /dev/null http://localhost:8080/stream.mjpg"
  sleep 1
  # The killed light node never switched the stick off, and WS2813 LEDs keep
  # their last colour: blank it explicitly (harmless if no stick is fitted).
  bash -c "$ROS; python3 -c 'from learn2race_hardware.grove_ws2813_rgb_stick import SpiWs2813RgbStick as S; s = S(20); s.begin(); s.clear(); s.close()'" >/dev/null 2>&1
  left=$(pgrep -af "[l]earn2race_ws/install|[m]icro_ros_agent|[c]amera_node" | wc -l)
  echo "stopped ($left processes left), track light off"
}

start() {   # start <bringup args> -- <follower args> [env]
  if running; then echo "Already running: ~/robot.sh stop first"; exit 1; fi
  if ! rpicam-hello --list-cameras 2>/dev/null | grep -q imx219; then
    echo "Camera not detected: power off and reseat the ribbon cable."; exit 1
  fi
  health
  local env="$3"
  # light:=N goes to the bringup (the track light), not to the follower.
  local light=255 track=portimao fargs="" bargs="" preview=off a
  for a in $2; do
    case "$a" in
      light:=*) light=${a#light:=} ;;
      track:=*) track=${a#track:=} ;;
      lidar:=*|display:=*|sensor_host:=*) bargs="$bargs $a" ;;
      preview:=*) preview=${a#preview:=} ;;
      *) fargs="$fargs $a" ;;
    esac
  done
  if [ ! -f $WS/install/learn2race_bringup/share/learn2race_bringup/config/track_$track.yaml ]; then
    echo "Unknown track '$track' (no config/track_$track.yaml)"; exit 1
  fi
  # Per-track follower arguments: "# robot.sh follower: ..." lines in the track file.
  fargs="$(sed -n 's/^# robot.sh follower: //p' $WS/install/learn2race_bringup/share/learn2race_bringup/config/track_$track.yaml | tr '\n' ' ')$fargs"
  echo "Track: $track, track light: $light${bargs:+, bringup:$bargs}, preview: $preview"
  set -- "$1 track_light_level:=$light track:=$track$bargs" "$fargs" "$3"
  # Logs also go to files, so a run can be analysed afterwards.
  local logs=~/run_logs/$(date +%Y%m%d-%H%M%S)
  mkdir -p $logs; ln -sfn $logs ~/run_logs/latest
  echo "bringup: $1" > $logs/args.txt; echo "follower: $2" >> $logs/args.txt
  echo "preview: $preview" >> $logs/args.txt
  # CPU and memory per process, every 2 s (pidstat costs ~5 % of a core at 1 s),
  # for the resource figures
  # (docs/figures/make_resource_figures.py).
  if command -v pidstat >/dev/null; then
    (setsid nohup env LC_ALL=C pidstat -h -u -r -l 2 > $logs/pidstat.txt 2>&1 < /dev/null &)
  else
    echo "pidstat missing (sudo apt install sysstat): no per-process CPU log"
  fi
  # Temperature, load and throttling with full timestamps (health.log has only
  # the time of day and spans several days).
  (setsid nohup bash -c "echo epoch,temp_c,load1,throttled,arm_mhz; while :; do      echo \$(date +%s.%N | cut -c1-14),\$(vcgencmd measure_temp | tr -dc '0-9.'),\$(cut -d' ' -f1 /proc/loadavg),\$(vcgencmd get_throttled | cut -d= -f2),\$(( \$(vcgencmd measure_clock arm | cut -d= -f2) / 1000000 ));      sleep 1; done" > $logs/thermal.csv 2>/dev/null < /dev/null &)
  tmux new-session -d -s $SESSION -n base \
    "$env $ROS; ros2 launch learn2race_bringup prototype_a.launch.py $1 2>&1 | tee $logs/base.log; bash"
  sleep 8   # let the agent and safety come up before the follower
  tmux new-window -t $SESSION -n follower \
    "$env $ROS; ros2 launch learn2race_camera_follower camera_line_follower.launch.py $2 snapshot_dir:=$logs/snaps 2>&1 | tee $logs/follower.log; bash"
  if [ "$preview" = on ]; then
    # A local viewer keeps the preview encoding, as if someone watched the page.
    (sleep 10; setsid nohup curl -s -o /dev/null http://localhost:8080/stream.mjpg >/dev/null 2>&1 < /dev/null &)
  fi
  echo "Running in tmux session '$SESSION'. ~/robot.sh attach to watch, ~/robot.sh stop to end."
  echo "Logs: $logs"
}

command -v tmux >/dev/null || { echo "tmux missing: sudo apt install tmux"; exit 1; }

case "$1" in
  practice)
    speed=${2:-0.3}; shift $(( $# < 2 ? $# : 2 ))
    start "competition:=false autonomy_enabled:=true sensor_host:=pi rgb_stick:=true" \
          "max_linear_speed:=$speed $*" "" ;;
  race)
    mode=${2:-trackbotgp}; speed=${3:-0.3}; shift $(( $# < 3 ? $# : 3 ))
    # Default ROS networking. Tried 2026-10-01: CycloneDDS on loopback plus
    # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST cut the micro-ROS agent (Fast
    # DDS) off from the nodes: "odom never received", no race. A cable drop
    # is handled instead by keeping eth0's address through link loss.
    start "competition:=true mode:=$mode sensor_host:=pi rgb_stick:=true" \
          "max_linear_speed:=$speed race_mode:=$mode $*" "" ;;
  attach) tmux attach -t $SESSION ;;
  stop) stop_all ;;
  status)
    running && echo "RUNNING: $(pgrep -af '[r]os2 launch' | sed 's/.*launch //' | tr '\n' ';')" || echo "stopped"
    uptime; tail -1 ~/health.log 2>/dev/null ;;
  build)
    if running; then echo "Stop the robot first (a build while running can make the Pi swap and freeze)."; exit 1; fi
    shift
    # micro_ros_agent is skipped unless named: it is built from source once, and a
    # ROS system update (2026-10-07: Fast DDS 2.14.6 -> 2.14.7) breaks its cached
    # build while the installed agent keeps working.
    bash -c "source /opt/ros/jazzy/setup.bash; cd $WS && colcon build ${*:+--packages-select $*} ${*:---packages-skip micro_ros_agent}" ;;
  graph)
    running || { echo "Start the robot first: the graph is read from the running system."; exit 1; }
    label=${2:-graph}
    bash -c "$ROS; python3 $TOOLS/export_ros_graph.py ~/run_logs/latest/graph_$label" ;;
  topics)
    running || { echo "Start the robot first."; exit 1; }
    bash -c "$ROS; python3 $TOOLS/topic_stats.py ~/run_logs/latest/topics_$(date +%H%M%S).csv ${2:-30}" ;;
  *) sed -n '/^# Prototype A run helper/,/^# After an SSH/p' "$0" ;;
esac
