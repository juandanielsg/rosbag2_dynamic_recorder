<p align="center">
  <img src="docs/_static/dynrec_logo.png" alt="dynrec" width="300">
</p>

# rosbag2_dynamic_recorder

A ROS 2 bag recorder that can change which topics it records while it is running. Add a topic,
remove one, or replace the whole set without stopping the writer or splitting the bag. Topics you
do not touch keep recording without interruption.

## Why this exists

`rosbag2_transport::Recorder` fixes its topic set at construction, from `RecordOptions`. Changing
it means stopping and starting again, which tears down every subscription and closes the bag:
unrelated topics lose messages and each change starts a new file.

This recorder keeps one continuous MCAP stream. A topic dropped and re-added becomes a sparse
channel with a gap, which MCAP supports natively and `ros2 bag play`, `mcap info` and Foxglove
handle. The recorder writes its own events into the bag (topic changes, pauses, file splits,
self-inflicted stops), so every gap explains itself.

It is not a replacement for rosbag2: bags are written with `rosbag2_cpp::Writer`, so the output is
a standard rosbag2 bag. Only the control plane is different.

## Install

ROS 2 Jazzy, Kilted or Rolling. Everything else comes from apt.

```bash
sudo apt install ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone --branch v0.1.0 https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

To only drive a recorder running elsewhere, see the
[client-only install](docs/install.md#client-only-install).

## Quick start

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"

ros2 dynrec add /diagnostics      # start recording one more topic
ros2 dynrec set /tf /odom         # record exactly these; /odom never stops
ros2 dynrec status
```

Or with the browser UI, then open <http://localhost:8088>:

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

From a Python supervisor script:

```python
from dynrec import Recorder

with Recorder() as rec:
    rec.set_topics(['/tf', '/odom'])
    rec.profile('navigation')
    rec.resume(at='+30s')
```

## Documentation

| | |
|---|---|
| [Install](docs/install.md) | Full and client-only installs, launch arguments, distro differences, Docker. |
| [Service API](docs/services.md) | Every service, event, status field and parameter. |
| [Profiles](docs/profiles.md) | Named topic sets, switched in one call. |
| [Command line](docs/cli.md) | `ros2 dynrec`, including `info` for reading a finished bag. |
| [Python library](docs/library/index.md) | `dynrec`, for supervisor scripts and bag analysis. |
| [Browser UI](docs/ui.md) | The page, its timeline and its scheduling card. |

## Layout

```
packages/
  rosbag2_dynamic_recorder_interfaces/   message and service definitions
  rosbag2_dynamic_recorder/              the recorder node
  rosbag2_dynamic_recorder_cli/          `ros2 dynrec`
  dynrec/                                the Python library
  rosbag2_dynamic_recorder_ui/           the browser UI
docs/                                    Sphinx documentation sources
.devcontainer/                           dev container, the same environment as CI
```

## Development

The dev container is the CI environment: one `ros:<distro>-ros-base` image plus the dependencies
from the package manifests. From the repository root:

```bash
docker compose --project-directory .devcontainer build      # ROS_DISTRO=jazzy|kilted|rolling
docker compose --project-directory .devcontainer up -d
docker compose --project-directory .devcontainer exec rosbag2-dev bash -l
```

Inside it:

```bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose
```

Packages are mounted at `src/packages`. Local additions, such as an upstream rosbag2 checkout in
`src/` for reading, go in `.devcontainer/docker-compose.override.yml`, which git ignores. To reach
the UI from the host, launch it with `bind:=0.0.0.0`.

## Status

Working end to end and tested in simulation on Jazzy, Kilted and Rolling; not yet run on real
hardware.

## License

Apache-2.0.
