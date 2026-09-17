# Install

You need **ROS 2 Jazzy, Kilted or Rolling**. Everything else comes from apt; there is no need to
build rosbag2 from source. That is deliberate: a source build costs forty minutes across 22
packages, and the only thing it buys is a service definition this project declares for itself.

## Full install

Everything: the recorder, the CLI, the Python library and the browser UI.

```bash
sudo apt install ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone --branch v0.1.0 https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws
rosdep install -r -y --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Five packages, a couple of minutes. `--branch v0.1.0` pins a release; every tag is a commit CI
has built and tested on Jazzy, Kilted and Rolling, so it is a known-good point on every distro at
once. Leave the flag off for `main`, which carries the same guarantee for its latest push but
moves. Then:

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
| `use_sim_time` | `false` | Stamp messages on the node clock driven by `/clock`, and open nothing until it has started. |
| `snapshot_mode` | `false` | Buffer in memory and write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. `snapshot_mode` needs this or `max_cache_duration` > 0. |
| `max_cache_duration` | `0` | Writer cache bound in seconds; `0` for none. Combines with `max_cache_size`. Rolling only. |
| `max_bagfile_size` | `0` | Split when a file reaches this many bytes; `0` never. |
| `max_bagfile_duration` | `0` | Split every this many seconds; `0` never. |
| `storage_preset_profile` | *(empty)* | Storage plugin preset. mcap: `none`, `fastwrite`, `zstd_fast`, `zstd_small`. |
| `storage_config_uri` | *(empty)* | Storage plugin YAML, overlaid on the preset. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `min_free_space` | `0` | Stop recording below this many bytes free on the bag filesystem; `0` disables. |
| `min_free_space_percent` | `0.0` | Same, as a percentage of the filesystem; `0.0` disables. The stricter applies. |
| `max_bag_size` | `0` | Stop recording once the bag directory, across every split, exceeds this many bytes; `0` disables. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |
| `params_file` | *(empty)* | Optional YAML of extra parameters, e.g. profiles. |

The other node parameters (`record_pause_events`, `record_low_disk_events`,
`record_bag_size_limit_events`, `storage_check_period`, `status_publish_period`, `profile_names`
and the profiles themselves)
are not launch arguments. Set them through `params_file`, or with `-p name:=value` when running the
node directly:

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
sudo apt install ros-$ROS_DISTRO-rclpy ros-$ROS_DISTRO-rosbag2-interfaces

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
2. **The environment it targets is apt-managed.** A stock ROS image has no `pip` at all, and its
   interpreter is marked `EXTERNALLY-MANAGED` under PEP 668.

So the unit of installation is the colcon workspace, and the lightest honest version of it is the
client-only build above.

## ROS 2 distributions

One source tree builds on Jazzy, Kilted and Rolling; CI runs the full suite on each. Rolling
(rosbag2 0.34) is what the project is written against and behaves exactly as documented. Jazzy
(0.26) and Kilted (0.32) differ in three places, all consequences of what their rosbag2 lacks.
The recorder's CMake finds each by probing the installed headers, and `dynrec.services` applies the
same rules on the client side, so nothing has to be configured:

| | Rolling | Jazzy, Kilted |
|---|---|---|
| `~/record`, `~/resume`, `~/split_bagfile`, `~/stop` | stock `rosbag2_interfaces` types | field-identical copies in `rosbag2_dynamic_recorder_interfaces`; same behaviour, scheduling included |
| `~/events/messages_lost` | stock `MessagesLostEvent` | field-identical copy; `messages_lost_in_recorder` always 0, because older writers do not report their losses |
| `max_cache_duration` | supported | refused at startup if non-zero; use `max_cache_size` |

The rule for a service is: the stock type where the installed definition is field-identical to the
0.34 one, the copy where it is not, so a client written for `ros2 bag record` drives the recorder
wherever that is possible. In Python, import the service classes from `dynrec.services` rather
than from `rosbag2_interfaces.srv` and they match the recorder built on the same machine.
`ros2 service type <service>` shows what a running recorder offers.

There are no per-distro branches. A distro-specific difference is a probe and an `#if`, kept
next to the code it guards, and a release is a tag on `main`. Pushing a tag runs the full matrix
on that commit and publishes a GitHub Release only if it passes, so every release page is a
commit proven on every distro.

Kilted reaches end of life in November 2026. Humble has not been tried; it is older than Jazzy and
would need its own look.

## Docker

A dev container is provided for working on the project, and for running the recorder against a
simulator without installing ROS on the host. It is the same environment CI tests in: an official
`ros:<distro>-ros-base` image plus the dependencies from our own package manifests, and it lives in
`.devcontainer/` so VS Code offers to reopen the repository inside it. From the repository root:

```bash
docker compose --project-directory .devcontainer build       # ROS_DISTRO=jazzy|kilted|rolling
docker compose --project-directory .devcontainer up -d
docker compose --project-directory .devcontainer exec rosbag2-dev bash -l
```

Notes:

- The container publishes port 8088 for the browser UI, but you must launch the UI with
  `bind:=0.0.0.0`. The default loopback bind is loopback inside the container, and Docker would have
  nothing to forward to.
- It does not use `network_mode: host`. On Docker Desktop that is the WSL2 VM's network, which the
  host OS cannot reach.
- Our packages live in `packages/`, mounted at `src/packages` in the workspace so a plain
  `colcon build` finds them. Anything local, such as an upstream rosbag2 checkout to read, goes in
  `.devcontainer/docker-compose.override.yml`, which compose merges in and git ignores.
- Dependencies are installed into the image from the manifests. If you add a package or a
  `<depend>`, rebuild the image, or run `rosdep install -r -y --from-paths src --ignore-src` inside
  the container.
