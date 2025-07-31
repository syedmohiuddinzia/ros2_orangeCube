# ROS2 MavRos Cube Orange

> Full setup guide to connect and use the **Cube Orange** flight controller with **ROS 2 (Humble)** via **MAVROS** over a USB serial interface.

### Introduction
MAVROS is a ROS (Robot Operating System) package that serves as a communication bridge between ROS and MAVLink-based autopilots such as PX4 and ArduPilot. It allows developers to control drones, monitor telemetry, and integrate autonomous flight behaviors using ROS tools.
This guide focuses on setting up MAVROS in ROS 2 Humble on Ubuntu 22.04 with the Orange Cube (CubePilot) flight controller. The Orange Cube is a powerful flight controller based on the Pixhawk family, commonly used in research and commercial drones for its modular design, reliable performance, and compatibility with ArduPilot/PX4 firmware.

---
## Cube Orange
![Cube Orange](https://github.com/syedmohiuddinzia/ros2_orangeCube/blob/main/pic/cube_orange.jpg)

The Cube Orange (CubePilot) is an advanced open-source autopilot. Key features include:
- Compatible with ArduPilot and PX4 firmware
- STM32H7 microcontroller with more processing power than older Pixhawk models
- IMU redundancy for improved reliability
- Integrated vibration isolation
- Standard interfaces (GPS, CAN, UART, I2C, etc.)
- Modular carrier boards (Cube+ Carrier, ADS-B, etc.)
For ROS/MAVROS use, it typically runs ArduPilot (APM) and communicates using MAVLink over USB or telemetry radios.

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** (or newer)
- `colcon`, `rosdep`, and basic ROS build tools
- Cube Orange Flight Controller
- USB-C data cable
- MAVROS plugin repo (`ros2` branch)

---
### Install ROS 2

If ROS 2 Humble isn't already installed, follow the official installation guide:

[ROS 2 Installation Instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Or run:

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo apt install ros-humble-desktop
```
Source the ROS2 environment
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Set Up ROS 2 Workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
Clone MAVROS (ROS 2 version):
```
cd ~/ros2_ws/src
git clone https://github.com/mavlink/mavros.git --branch ros2
```

### Install Dependencies
```
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip
```
```
sudo apt install \
  ros-humble-mavlink \
  ros-humble-geographic-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  geographiclib-tools \
  libgeographic-dev
```
Install geographiclib datasets (needed for GPS transformations):
```
sudo geographiclib-get-geoids egm96-5
```

### Serial Access Setup
Ensure you have permission to access `/dev/ttyACM0`:
```
sudo usermod -a -G dialout $USER
newgrp dialout
```
Verify:
```
ls -l /dev/ttyACM0
```
You should see group `dialout`.

### Build the Workspace
```
cd ~/ros2_ws
colcon build --symlink-install
```
Source the workspace:
```
source ~/ros2_ws/install/setup.bash
```
Add to .bashrc:
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---
### Launch MAVROS with Cube Orange
Following is the procedure to launch mavros with orange cube using serial port

#### Identify the Serial Port
Plug in Cube Orange via USB and run:
```
dmesg | grep tty
```
You’ll likely find `/dev/ttyACM0`.

#### Run MAVROS
```
ros2 launch mavros node.launch \
  fcu_url:=/dev/ttyACM0:115200 \
  gcs_url:="" \
  tgt_system:=1 \
  tgt_component:=1 \
  pluginlists_yaml:=$(ros2 pkg prefix mavros)/share/mavros/launch/px4_pluginlists.yaml \
  config_yaml:=$(ros2 pkg prefix mavros)/share/mavros/launch/px4_config.yaml
```
| Arg                | Description                          |
| ------------------ | ------------------------------------ |
| `fcu_url`          | Serial port connected to Cube        |
| `gcs_url`          | Optional GCS connection (empty here) |
| `tgt_system`       | MAVLink system ID                    |
| `tgt_component`    | MAVLink component ID                 |
| `pluginlists_yaml` | MAVROS plugin list                   |
| `config_yaml`      | Config for frame\_id, rate, etc.     |

### Check ROS 2 Topics
```
ros2 topic list
```
Expected topics:
- /mavros/state
- /mavros/imu/data
- /mavros/global_position/global
- /mavros/local_position/pose
- /mavros/rc/in
- /mavros/battery

### View Data
#### View heartbeat
```
ros2 topic echo /mavros/heartbeat
```
#### View IMU
```
ros2 topic echo /mavros/imu/data
```
### Visualize in Rviz2
```
rviz2
```
- Add displays for:
  - TF
  - IMU
  - Pose
  - RobotMode

### Plot Data in rqt
```
rqt
```
Plot topics like:
```
/mavros/imu/data
```
```
/mavros/battery
```
```
/mavros/rc/in
```

### Troubleshooting
| Problem                      | Solution                                                      |
| ---------------------------- | ------------------------------------------------------------- |
| No serial data               | Check `dmesg`, verify `/dev/ttyACM0`, user in `dialout` group |
| Topics not showing           | Recheck launch args, especially plugin YAML paths             |
| Can't echo topic             | Try sourcing workspace again, verify `ros2 topic list`        |
| Slow data rate or delay      | Tune `config_yaml` parameters                                 |
| Conflicting MAVLink versions | Ensure correct PX4/APM firmware compatibility                 |

### Additional Resources
- MAVROS Wiki
- PX4 Docs
- ArduPilot Docs
- GeographicLib
