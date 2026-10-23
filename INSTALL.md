# Installation guide – Prototype A

This guide takes a blank Raspberry Pi 4 and a new ESP32 board to a robot that runs `~/robot.sh race`. Allow about two hours, most of it compiling.

## 1. What you need

- Raspberry Pi 4 (4 GB or more) with a 32 GB microSD card.
- Prototype A assembled: Waveshare General Driver for Robots (ESP32), two CHP-20GP-180ABHL motors, IMX219 camera on the CSI port, WS2813 LED ring on GPIO10 (SPI MOSI), LTR-303 and the board's I2C sensors wired to the Pi's I2C bus 1, LD06 lidar on USB.
- A PC with [PlatformIO](https://platformio.org/) (VS Code extension or `pip install platformio`) to build and flash the ESP32.
- A network shared by the robot and the PC. The project uses a TP-Link TL-WR802N mini router, with the Pi on Ethernet and a fixed address (Prototype A: `192.168.8.101`).

## 2. Operating system

1. Write **Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi** with Raspberry Pi Imager. In the imager settings, create the user `learn2race` and enable SSH.
2. Boot, log in and update:

   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo apt install -y git tmux sysstat i2c-tools python3-pip python3-smbus2 python3-spidev
   ```

3. Edit `/boot/firmware/config.txt` and add, under `[all]`:

   ```ini
   dtparam=i2c_arm=on
   dtparam=spi=on
   camera_auto_detect=0
   dtoverlay=imx219
   # The SPI clock follows the core clock; a fixed minimum keeps the LED timing right
   core_freq_min=500
   # Wi-Fi off: the robot uses Ethernet to the router (fewer stalls and faster boot)
   dtoverlay=disable-wifi
   ```

4. Give the user access to the hardware, then reboot:

   ```bash
   sudo usermod -aG dialout,i2c,spi,video learn2race
   sudo reboot
   ```

5. **Kernel.** Use the standard `raspi` kernel, not the real-time one. If an upgrade installs `linux-image-*-realtime`, put the standard kernel back and hold the real-time packages:

   ```bash
   uname -r                               # expect 6.8.0-xxxx-raspi
   sudo flash-kernel --force <standard kernel version>
   sudo apt-mark hold linux-image-raspi-realtime linux-raspi-realtime 2>/dev/null
   ```

6. Optional, recommended: turn off unattended upgrades (`sudo systemctl disable --now unattended-upgrades`). An upgrade during an event day can change the kernel or the DDS libraries.
7. If Wi-Fi is disabled, remove any `wlan0` entry from the netplan files in `/etc/netplan/`. Otherwise systemd waits 90 s at boot for a device that never appears.

## 3. ROS 2 Jazzy

Follow the official [ROS 2 Jazzy installation for Ubuntu (deb packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), installing `ros-jazzy-ros-base` and `ros-dev-tools`. Then add the packages the workspace uses:

```bash
sudo apt install -y \
  ros-jazzy-camera-ros ros-jazzy-cv-bridge ros-jazzy-image-proc \
  ros-jazzy-imu-filter-madgwick ros-jazzy-robot-localization \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-tf2-ros \
  ros-jazzy-camera-calibration ros-jazzy-stereo-image-proc \
  ros-jazzy-diagnostic-msgs ros-jazzy-rosbag2-py \
  python3-opencv python3-numpy python3-pil
sudo rosdep init 2>/dev/null; rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 4. Camera (Raspberry Pi libcamera)

The `libcamera` shipped with ROS does not run the Raspberry Pi camera pipeline correctly on Ubuntu: the camera node starts but no frames arrive. Build the Raspberry Pi fork of libcamera and `rpicam-apps` into `/usr/local`, following the Raspberry Pi documentation ("Building libcamera and rpicam-apps"). The line-follower launch file loads `/usr/local/lib/aarch64-linux-gnu/libcamera.so.0.7` first when it is present.

Check the camera before going further:

```bash
rpicam-hello --list-cameras     # must list imx219
```

## 5. Workspace

The run helper expects the workspace at `~/learn2race_ws` and the tools at `~/learn2race_tools`:

```bash
cd ~
git clone https://github.com/rcfcpereira/Learn2Race-PrototypeA.git
ln -s ~/Learn2Race-PrototypeA/learn2race_ws ~/learn2race_ws
ln -s ~/Learn2Race-PrototypeA/tools ~/learn2race_tools
cp ~/Learn2Race-PrototypeA/tools/robot.sh ~/robot.sh && chmod +x ~/robot.sh

# Third-party sources: the micro-ROS agent and the LD06 lidar driver
cd ~/learn2race_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

cd ~/learn2race_ws
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install       # about 40 min on a Pi 4 the first time
```

The third-party folders are ignored by git (see `.gitignore`), so updating the repository with `git pull` does not touch them. After the first build, rebuild only the project packages with `~/robot.sh build`. That command skips the micro-ROS agent unless it is named, because a ROS update can break the agent's cached build while the installed agent keeps working.

## 6. Stable device names

The ESP32 board and the lidar are both USB serial devices, and their `ttyUSB` numbers swap between boots. Create fixed names (`/dev/mcu` and `/dev/lidar`):

```bash
cd ~/learn2race_ws/src/learn2race_bringup/scripts
sudo ./setup_udev.sh        # asks you to unplug each device in turn
ls -l /dev/mcu /dev/lidar
```

## 7. ESP32 firmware (from the PC)

The Pi reads the board's I2C sensors, so the ESP32 must run the **`esp32dev_pi_sensors`** profile. The I2C bus is wired to both the ESP32 and the Pi, and only one of them may drive it.

```bash
cd firmware
pio run -e esp32dev_pi_sensors -t upload
```

The firmware profile and the launch argument must always match:

| Firmware profile | Launch argument | Who reads IMU, light sensor and INA219 |
|---|---|---|
| `esp32dev_pi_sensors` (used) | `sensor_host:=pi` (the default in `robot.sh`) | the Pi |
| `esp32dev` | `sensor_host:=esp32`, with the OLED off | the ESP32 |

Other profiles: `diag` (prints the I2C sensors over serial, no micro-ROS) and `wheels` (raw PWM and encoder test from the serial monitor). See [firmware/README.md](firmware/README.md).

## 8. First run and checks

```bash
~/robot.sh practice 0.2
~/robot.sh status
```

Leave the wheels off the floor for the first run. Check:

1. The status LED turns green when the stack is up. If it stays off, the ESP32 is not connected: check `/dev/mcu`.
2. With a line under the camera, the wheels turn. The run page at `http://<robot address>:8080` shows the camera view and the detected line.
3. `~/robot.sh stop` stops everything and switches off the LED ring.

Then run the hardware check with the robot on a stand: `~/robot.sh check wheels_lifted:=true` (see [USAGE.md](USAGE.md#hardware-check)).

Calibration (wheel radius, track width, IMU bias, camera) is described in [USAGE.md](USAGE.md#calibration).

## 9. Automatic updates

Each robot updates itself from this repository, so a change reaches every unit with one `git push` and a power cycle instead of an SD-card image or an SSH session per robot. Install the service once:

```bash
~/Learn2Race-PrototypeA/tools/install_update_service.sh          # follow main
~/Learn2Race-PrototypeA/tools/install_update_service.sh v1.0     # or pin a release tag
```

At every boot, once the network is up, the `learn2race-update` service runs `tools/update_robot.sh`, which:

1. does nothing if the robot is running or the network is down;
2. refuses to update a checkout with uncommitted changes (commit calibration changes first);
3. fetches and moves to the pinned branch or tag (`L2R_REF` in `/etc/default/learn2race-update`);
4. runs `tools/check_robot_config.py`, which checks that `robot.yaml` agrees with `firmware/platformio.ini` and the ROS configuration, and rolls back if it does not;
5. installs missing dependencies with `rosdep` (when `sudo` needs no password) and rebuilds the workspace; if the build fails, it rolls back to the previous revision and rebuilds that;
6. refreshes `~/robot.sh` and records the version in `~/.learn2race_version`.

A revision that failed is not retried at every boot; push a fix, or force a retry with `L2R_FORCE=1 ~/robot.sh update`. The robot is never started by the update: start it with `~/robot.sh` as usual, which refuses to start while an update is building. The ESP32 is not flashed automatically: when `firmware/` changed, the log says so, and the firmware is flashed from a PC (section 7).

```bash
~/robot.sh update               # update now (the robot must be stopped)
~/robot.sh status               # shows the installed version
cat ~/update_logs/latest.log    # what the last update did
```

`robot.yaml`, at the root of this repository, records the robot's identity and calibration constants. Two robots built from the same release differ only in that file. When a constant changes, change it in `robot.yaml` and in the file it is compiled from, together; `python3 tools/check_robot_config.py` confirms they agree.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Camera not detected` from `robot.sh` | Power off and reseat the camera ribbon |
| Agent never connects (LED not green) | `/dev/mcu` missing (USB cable or board power), or the baud rate differs from `ROS_SERIAL_BAUD` in `platformio.ini` (921 600) |
| `Errno 121` / I/O errors on I2C | Wrong firmware profile: the ESP32 and the Pi are both driving the bus (see section 7) |
| LED ring shows wrong colours | `core_freq_min=500` missing from `config.txt` |
| Pi freezes under load | Real-time kernel in use, or a build running at the same time as a race; `robot.sh build` refuses to build while the robot runs |
| Robot stops after 0.5 s in a race, or moves on its own | Another robot is running a ROS stack on the same network and ROS domain. Run one robot at a time, or give each a different `ROS_DOMAIN_ID` |
