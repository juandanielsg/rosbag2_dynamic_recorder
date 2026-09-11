# Install

You need **ROS 2 Rolling**. Everything else comes from apt; there is no need to build rosbag2 from
source. That is deliberate: a source build costs forty minutes across 22 packages, and the only
thing it buys is a service definition this project declares for itself.

## Full install

Everything: the recorder, the CLI, the Python library and the browser UI.

```bash
sudo apt install ros-rolling-rosbag2 ros-rolling-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws
rosdep install -r -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Five packages, a couple of minutes. Then:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

## Launch arguments

`dynamic_recorder.launch.py` forwards these to the node:

| Argument | Default | Meaning |
|---|---|---|
| `uri` | `dynamic_bag` | Output bag path. |
| `storage_id` | `mcap` | Storage plugin. |
| `serialization_format` | `cdr` | Message serialization format. |
| `topics` | `[]` | Topics to record at startup, as a YAML list. May be empty. |
| `start_paused` | `false` | Start with recording paused. |
| `snapshot_mode` | `false` | Buffer in memory and write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. `snapshot_mode` requires it to be > 0. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |
| `params_file` | *(empty)* | Optional YAML of extra parameters, e.g. profiles. |

The other node parameters (`record_pause_events`, `status_publish_period`, `profile_names` and the
profiles themselves) are not launch arguments. Set them through `params_file`, or with
`-p name:=value` when running the node directly:

```bash
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:=/tmp/mybag -p record_pause_events:=false
```

`recorder_with_ui.launch.py` takes the same list, plus:

| Argument | Default | Meaning |
|---|---|---|
| `port` | `8088` | Port for the browser UI. |
| `bind` | `127.0.0.1` | Address the UI listens on. Loopback by default because the UI has no authentication; see [the UI page](ui.md). |

## Client-only install

A machine that only drives a recorder running somewhere else (a fleet console, a laptop on the same
DDS domain, a supervisor host) does not need the recorder at all. `dynrec` and the CLI depend on
`rclpy` and two interface packages; neither depends on the C++ recorder, on rosbag2's storage
plugins, or on MCAP.

```bash
sudo apt install ros-rolling-rclpy ros-rolling-rosbag2-interfaces

colcon build --packages-select rosbag2_dynamic_recorder_interfaces dynrec
```

Two small packages instead of five, and none of the storage stack. Add
`rosbag2_dynamic_recorder_cli` to that list if you want `ros2 dynrec` as well.

## Why there is no `pip install`

`dynrec` is a Python package, so this is a fair thing to expect. It cannot work, for two reasons:

1. **The interface bindings are generated, not pure Python.** `dynrec` imports
   `rosbag2_dynamic_recorder_interfaces`, whose Python bindings are a C extension plus typesupport
   built by `rosidl` against one exact ROS distribution. There is no supported way to ship that as a
   wheel, so a PyPI release of `dynrec` would install a package that raises `ImportError` on first
   use.
2. **The environment it targets is apt-managed.** A stock ROS Rolling image has no `pip` at all, and
   its interpreter is marked `EXTERNALLY-MANAGED` under PEP 668.

So the unit of installation is the colcon workspace, and the lightest honest version of it is the
client-only build above.

## Docker

A dev container is provided for working on the project, and for running the recorder against a
simulator without installing ROS on the host:

```bash
docker compose build
docker compose up -d
docker compose exec rosbag2-dev bash -l
```

Notes:

- The container publishes port 8088 for the browser UI, but you must launch the UI with
  `bind:=0.0.0.0`. The default loopback bind is loopback inside the container, and Docker would have
  nothing to forward to.
- It does not use `network_mode: host`. On Docker Desktop that is the WSL2 VM's network, which the
  host OS cannot reach.
- Our packages live in `packages/`, mounted into the workspace separately, so the optional upstream
  rosbag2 checkout in `src/` stays pristine.
- If you add a package with new dependencies, run
  `rosdep install -r -y --from-paths src --ignore-src` inside the container.
