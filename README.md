# Hierarchical Decision-Driven Adaptive Video Streaming over Dynamic Wireless Links

Source repository accompanying the competition technical report. This ROS 2 package adapts camera-image resolution, target frame rate, and JPEG quality using local wireless-link measurements. It runs on the robot; a laptop connected to the same Wi-Fi network views the camera through `web_video_server`.

The demonstrated platform is a remotely operated tracked mowing robot with an NVIDIA Jetson Orin Nano, a ZED 2i camera, and an Intel Wireless-AC 8265 adapter. Autonomous navigation is outside this streaming package.

## 1. Contents

```text
adaptive_stream/
├── adaptive_stream/
│   ├── __init__.py
│   ├── adaptive_stream_node.py
│   └── network_monitor_node.py
├── resource/
│   └── adaptive_stream
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

`setup.py` registers both node executables. `setup.cfg` places them in the ROS 2 package executable directory. The resource marker registers the package with the ament index. No separate launch or YAML file is required for the four-terminal procedure below: the profiles and confirmation thresholds are defined in the Python nodes.

The repository includes the two application nodes and their installation metadata. ROS 2, the ZED SDK, the ZED ROS 2 wrapper, and `web_video_server` are external dependencies; the complete robot workspace is not required as part of this repository.

## 2. Environment and dependencies

The deployment uses Ubuntu 22.04 and ROS 2 Humble on the Jetson. Use the system Python environment associated with ROS 2 Humble. The package imports `rclpy`, `sensor_msgs`, `std_msgs`, `rcl_interfaces`, `cv_bridge`, and OpenCV. The network monitor invokes the Linux `iw` utility.

Before installation, configure the ROS 2 Humble package repository and source `/opt/ros/humble/setup.bash`. Install the common build tools and non-ROS utilities on the Jetson:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
    python3-opencv iw git
```

If rosdep has never been initialized on this machine, run `sudo rosdep init` once. Then run:

```bash
rosdep update
```

For camera acquisition, install a ZED SDK and ZED ROS 2 wrapper compatible with the Jetson's installed JetPack/CUDA and ROS 2 environment. An existing working camera installation can be reused. The streaming package consumes a standard `sensor_msgs/msg/Image` topic and does not import the ZED SDK directly.

For browser viewing, install `web_video_server` from the configured ROS repository, or reuse the version already installed in the robot workspace:

```bash
sudo apt install ros-humble-web-video-server
```

Upstream installation references:

- [ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ZED ROS 2 integration](https://docs.stereolabs.com/docs/integrations/ros-2)
- [RobotWebTools web_video_server](https://github.com/RobotWebTools/web_video_server)

## 3. Install and build the package

Clone this repository into `~/wheeltec_ros2/src/adaptive_stream` so that `package.xml` is at `~/wheeltec_ros2/src/adaptive_stream/package.xml`. If a copy of this package already exists in that workspace, use that copy or move it aside before cloning; keep only one copy in the workspace.

```bash
mkdir -p ~/wheeltec_ros2/src
git clone https://github.com/mengshaojun038-code/wireless-link-aware-adaptive-streaming.git \
    ~/wheeltec_ros2/src/adaptive_stream

cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/adaptive_stream --ignore-src \
    --rosdistro humble -r -y
colcon build --packages-select adaptive_stream
source install/setup.bash

ros2 pkg executables adaptive_stream
```

The final command should list `adaptive_stream_node` and `network_monitor_node` under the `adaptive_stream` package. ROS commands use the package name declared in `package.xml`: `adaptive_stream`.

## 4. Architecture and interfaces

```text
ZED camera driver -- raw Image --> adaptive_stream_node
                                      |       |
                                      |       +--> /adaptive/image/compressed (JPEG)
                                      |
                                      +--> /adaptive/image (resized bgr8 Image)
                                                |
                                         web_video_server
                                                |
                                          Wi-Fi / HTTP
                                                |
                                         Laptop browser

iw wireless statistics --> network_monitor_node
                                      |
                            /adaptive/network_state
                                      |
                              adaptive_stream_node
```

| Direction | Topic | ROS message type |
|---|---|---|
| Camera input | `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` |
| Network monitor output / streaming node input | `/adaptive/network_state` | `std_msgs/msg/String` |
| JPEG output | `/adaptive/image/compressed` | `sensor_msgs/msg/CompressedImage` |
| Browser image input | `/adaptive/image` | `sensor_msgs/msg/Image` |

Both image outputs preserve the camera message header. Resizing and frame scheduling apply to both outputs. JPEG quality controls the node's compressed-image output. The browser service consumes the raw-image topic and applies its own encoding settings.

## 5. Adaptation policy implemented in the supplied code

### Video profiles

| Profile | Resolution | Target frame rate | JPEG quality |
|---|---:|---:|---:|
| HIGH | 1280 x 720 | 20 FPS | 90 |
| MEDIUM | 640 x 480 | 15 FPS | 70 |
| LOW | 320 x 240 | 10 FPS | 40 |

The startup profile is `medium`. Target FPS is a scheduling setting; actual output depends on camera input and processing time. Profile changes reset the frame-scheduling reference and interval statistics.

### Wireless classification and first confirmation stage

The network monitor queries the selected Wi-Fi interface at a nominal one-second interval. The following ordered rules use strict inequalities:

| State | Rule | Requested profile |
|---|---|---|
| GOOD | RSSI > -60 dBm AND PHY TX rate > 50 Mbit/s | HIGH |
| MEDIUM | GOOD is false, RSSI > -70 dBm AND PHY TX rate > 25 Mbit/s | MEDIUM |
| BAD | All remaining cases | LOW |

Retry and failed-transmission counters are recorded for diagnostics; they do not affect the classifier. The first statistics sample initializes the packet counters. The first valid classification is accepted immediately. Thereafter, a lower candidate state requires three identical consecutive classifications, and a higher candidate state requires five. A changed candidate restarts its count; a return to the current state cancels it. The confirmed state is published after each successful evaluation.

### Video-node confirmation stage

The streaming node separately counts consecutive received network-state messages:

- GOOD: 10 messages before applying HIGH.
- MEDIUM: 5 messages before applying MEDIUM.
- BAD: 5 messages before applying LOW.

A changed received state restarts this count at one. At the threshold, the node applies the requested profile if it differs from the current profile, then resets the count to zero. The count also resets at the threshold when the profile is already active.

The log text `stable=10s` or `stable=5s` is produced from this message counter. It is not an independently measured elapsed time. With regularly received one-second messages, the span from the first counted message to the threshold message is approximately 9 s or 4 s, respectively. Monitor confirmation and video-profile application are distinct stages.

### Wireless deployment

In managed mode, the monitor reads the MAC address of the connected access point. In AP mode, it uses the first associated station returned by `iw station dump`; the intended outdoor arrangement has one monitoring laptop. Connect the Wi-Fi link before starting the monitor. The package observes an existing connection; it does not configure the hotspot or select a radio channel.

## 6. Start the demonstration

Run the following terminals on the Jetson. Each terminal must source ROS 2 and the workspace. If the ZED wrapper is installed in a separate workspace, source that installation as well.

### Terminal 1: camera

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

If the robot's usual bring-up already starts the camera, reuse it instead of launching a second camera node. Confirm the RGB topic name with `ros2 topic list -t`; wrapper versions and namespaces can differ.

### Terminal 2: wireless monitor

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
iw dev
ros2 run adaptive_stream network_monitor_node
```

The default interface is `wlP1p1s0`. If `iw dev` reports a different interface, pass its actual name. For example, use this command only when the interface is `wlan0`:

```bash
ros2 run adaptive_stream network_monitor_node --ros-args -p interface:=wlan0
```

### Terminal 3: adaptive streaming

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run adaptive_stream adaptive_stream_node
```

If the camera publishes a different RGB image topic, remap the input. For example, for a camera publishing `/camera/image_raw`:

```bash
ros2 run adaptive_stream adaptive_stream_node --ros-args \
    -r /zed/zed_node/rgb/color/rect/image:=/camera/image_raw
```

### Terminal 4: browser service

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run web_video_server web_video_server
```

### Laptop browser

Connect the laptop to the robot's Wi-Fi network. On the Jetson, use `ip -4 addr` to identify the IPv4 address on the interface used by the laptop. Replace `ROBOT_IP` below with that address:

```text
http://ROBOT_IP:8080/
http://ROBOT_IP:8080/stream_viewer?topic=/adaptive/image
```

The first URL lists available streams; the second opens the adaptive camera preview. Keep port 8080 reachable on the local network. Use Ctrl+C in each running terminal to stop the demonstration.

## 7. Output statistics and verification

The streaming node reports its current profile, output dimensions, actual local FPS, JPEG quality, mean JPEG frame size, and generated JPEG payload rate. The network monitor reports link metrics and Raw/Stable states; profile-switch messages include the triggering state and message count.

For an interval of duration `dt`, successful output frame count `N`, and total JPEG payload bytes `B`:

```text
FPS                 = N / dt
JPEG rate (Mbit/s)   = 8 * B / dt / 1,000,000
Mean frame size     = B / N / 1024
```

The terminal label `KB` for frame size uses 1024 bytes per unit, i.e. KiB. The node reports statistics approximately once per second and resets them on a profile change. These are source-node measurements, not browser playback FPS or camera-to-display latency. The JPEG rate excludes protocol overhead and the browser server's separate encoding. `iw` PHY TX bitrate is a radio-link rate, not available application throughput.

Useful checks from a sourced Jetson terminal:

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /adaptive/network_state
ros2 param get /adaptive_stream_node mode
ros2 topic info -v /zed/zed_node/rgb/color/rect/image
ros2 topic hz /adaptive/image
```

Stop continuous `echo` or `hz` commands with Ctrl+C before starting the next check. Topic-frequency measurements use their own subscriber and can differ from the node's interval statistics. Check camera-topic naming and publisher/subscriber QoS compatibility if no images arrive. Verify `iw` can read the configured interface if network checks fail.

For temporary manual profile inspection, stop the network monitor first so automatic adaptation does not overwrite the requested mode:

```bash
ros2 param set /adaptive_stream_node mode high
```

Accepted values are lowercase `high`, `medium`, and `low`. Restart the network monitor to resume automatic adaptation. This is a diagnostic procedure, not an additional experiment reported in the submission.

## 8. Submission scope

The repository contains the two implemented nodes, complete installation and viewing instructions, and the declared `iw` runtime dependency. It contains no generated build directories, Python caches, or unused development backups. The accompanying technical report provides the outdoor evaluation, figures, and literature comparison.
