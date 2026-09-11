<p align="center">
  <img src="docs/_static/dynrec_logo.png" alt="dynrec" width="300">
</p>

# rosbag2_dynamic_recorder

A ROS 2 bag recorder that can change which topics it records while it is running. Add a topic,
remove one, or replace the whole set without stopping the writer or splitting the bag. Topics you
do not touch keep recording without interruption.

Documentation: <https://juandanielsg.github.io/rosbag2_dynamic_recorder/> covers the service API,
the three clients and the Python library reference. It is published from [`docs/`](docs/) on every
push to `main`.

## Why this exists

`rosbag2_transport::Recorder` fixes its topic set at construction, from `RecordOptions`. Changing
it means stopping and starting again, which tears down every subscription and closes the bag:
unrelated topics lose messages and each change starts a new file.

This recorder keeps one continuous MCAP stream. A topic dropped and re-added becomes a sparse
channel with a gap, which MCAP supports natively and `ros2 bag play`, `mcap info` and Foxglove
handle.

It is not a replacement for rosbag2: bags are written with `rosbag2_cpp::Writer`, so the output is
a standard rosbag2 bag and the ecosystem keeps working. Only the control plane is different.

## Install

You need ROS 2 Rolling. Everything else comes from apt; no need to build rosbag2 from source.

```bash
sudo apt install ros-rolling-rosbag2 ros-rolling-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

Five packages, a minute or two. To only drive a recorder running elsewhere you need `rclpy`, the
interfaces and `dynrec` (plus the CLI for `ros2 dynrec`), not the recorder or storage stack; see
[docs/install.md](docs/install.md#client-only-install).

## Quick start

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

Add a topic while it records:

```bash
ros2 service call /rosbag2_dynamic_recorder/subscribe_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SubscribeTopics "{topics: ['/diagnostics']}"
```

The same through the browser UI:

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open <http://localhost:8088>.

## Services

Topic management:

| Service | Purpose |
|---|---|
| `~/subscribe_topics` | Add topics. Types are discovered from the graph if they are not given. |
| `~/unsubscribe_topics` | Remove topics. Messages already written are kept. |
| `~/set_topics` | Replace the whole set. Topics in both the old and the new set keep running. |
| `~/get_subscribed_topics` | Return the current set. |
| `~/set_profile` | Apply a named topic set. Same guarantee as `set_topics`. |
| `~/get_profiles` | List the configured profiles. |

`subscribe_topics`, `unsubscribe_topics` and `set_topics` also accept `regex` and `exclude_regex`.
Semantics match `ros2 bag record -e` (ECMAScript), so `camera` matches `/robot/camera/image`, and
`exclude_regex` filters the combined selection, making "all of `/robot` except the depth camera" one
call. Patterns never match the recorder's own or hidden topics, and an invalid expression is
rejected with the reason.

Recording control. These reuse the stock `rosbag2_interfaces` definitions, so a stock rosbag2 client
drives this node unchanged:

| Service | Purpose |
|---|---|
| `~/pause` `~/resume` `~/toggle_paused` `~/is_paused` | Stop writing without tearing down subscriptions. |
| `~/split_bagfile` | Close the current file and open the next. |
| `~/snapshot` | Flush the in-memory buffer. Requires `snapshot_mode`. |
| `~/stop` | Close the bag. |
| `~/record` | Open a new bag and start again with the previous topic set. |
| `~/get_status` | Bag state, topic set and counters in one call. |

The project's services return `return_code` 0 on success and 1 on error or when nothing was
actioned. The reused `rosbag2_interfaces` services keep their own return codes; `snapshot` returns a
bool.

`resume`, `split_bagfile` and `record` accept a future timestamp. Node time (`mode: 0`) runs on a
timer and fires even if the robot has gone quiet; it is the only mode `record` supports. Publish
time (`1`) and receive time (`2`) compare against arriving messages, on any recorded topic or
`tracking_topic_name` alone, so they cannot fire while none arrive. A mode outside 0–2, or an
unrecorded `tracking_topic_name`, is rejected.

`~/record` opens a fresh bag and restores the set active at stop. Since rosbag2 will not open over
an existing directory, it uses the next free suffix: `mybag`, then `mybag(1)`.

`~/get_status` reports the bag URI and storage id, recording/paused/snapshot state, start time and
elapsed seconds, the topic set, messages written, messages lost, split count and bag size.
`bag_size_bytes` counts flushed bytes, not captured, so it reads 0 early; use `messages_written` to
tell whether it is recording.

## Profiles

A named topic set, switched in one call. For a robot that records different topics per mode, a
supervisor changes the mode and the recorded set follows, while topics the modes share keep running.

```yaml
rosbag2_dynamic_recorder:
  ros__parameters:
    profile_names: ["idle", "navigation", "inspection"]
    profiles:
      idle: ["/tf", "/odom", "/imu"]
      navigation: ["/tf", "/odom", "/imu", "/scan"]
```

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py \
  uri:=/tmp/mybag params_file:=profiles.yaml
```

In the UI they are buttons, the active one highlighted. A worked example using the TurtleBot 4
simulator topics ships in `rosbag2_dynamic_recorder/config/profiles.example.yaml`.

`active_profile` is derived from the live topic set, not remembered: record one topic no profile
contains and it goes empty; land exactly on a profile's set and it reports that profile, however
you got there.

## Events

| Topic | Message |
|---|---|
| `~/events/subscription_change` | `SubscriptionChangeEvent`, also written into the bag |
| `~/events/pause` | `PauseEvent`, also written into the bag |
| `~/events/write_split` | `WriteSplitEvent` |
| `~/events/messages_lost` | `MessagesLostEvent` |

Writing the first two into the bag is what makes a gap legible; without them it is
indistinguishable from a dropout, a crash or a network fault.

- `SubscriptionChangeEvent` marks where one topic stops while others carry on. Disable with
  `record_subscription_events:=false`.
- `PauseEvent` marks where the whole recording stops: a gap in every topic at once, the same shape
  as a crash, a network fault, or the rosbag2 stall that drops about 1.2s from every topic every
  ~31.2s. The stock `Pause` service has empty request and response fields and can carry no reason;
  this event can. Disable with `record_pause_events:=false`.

While paused no messages are written, but these events still are; an event suppressed by the pause
it describes would leave the gap unexplained. `reason` distinguishes `service:pause`,
`service:resume`, `service:toggle_paused`, `schedule:resume` and `startup` (the last covers
`start_paused` and a `~/record` while paused, both opening a bag with an empty beginning).

`ros2 bag play` replays the sparse channels at full fidelity and `info` and `convert` process them.
One caveat: `mcap info` averages a channel's rate over the bag, so a topic that ran at 20 Hz and
then stopped is shown as 4.43 Hz. The recorded events recover the real rate.

## Command line

`ros2 dynrec` drives a running recorder without service-call syntax or a display:

```bash
ros2 dynrec status            # what is it doing
ros2 dynrec topics            # what is it recording, one per line
ros2 dynrec add /scan         # start recording /scan, leave everything else alone
ros2 dynrec remove /scan      # stop recording it
ros2 dynrec set /tf /odom     # record exactly these
ros2 dynrec add -e '^/camera/'                     # everything under /camera
ros2 dynrec set -e '^/robot/' --exclude-regex depth  # all of /robot but the depth camera
ros2 dynrec remove -e '/image'                     # drop the heavy ones
ros2 dynrec profile navigation
ros2 dynrec profiles          # list configured profiles, active one marked
ros2 dynrec pause | resume | toggle | split | snapshot | stop | record
ros2 dynrec info /tmp/mybag   # read a finished bag back
```

With one recorder on the graph there is nothing to configure: it finds a node offering
`~/get_status` of this project's type, so any name or namespace works, and a node that merely
borrows the name does not. With several it refuses to guess and lists them; pass `--node`.

Every verb exits non-zero when the recorder refuses, so a supervisor script can use the exit code
instead of parsing `return_code`:

```bash
ros2 dynrec add /scan || echo "could not record /scan"
```

`status --json` prints one `GetStatus` call as data. `messages_missed` is `null` when the middleware
supplies no publication sequence numbers, separating "nothing was missed" from "unknown"; other
counters pass through as reported.

`ros2 dynrec info` is the one verb that reads a finished bag rather than driving a recorder. Other
tools average a channel's rate over the bag, so a topic that ran at 20 Hz and was then removed shows
as 5.95 Hz; the recorder's own events measure each channel over the time it was actually recorded:

```
topic                                 msgs  recorded   active       rate    averaged
/demo/alpha                            590     29.5s    29.5s   20.00 Hz    16.12 Hz
/demo/beta                             218     11.0s    11.0s   19.87 Hz     5.95 Hz
/demo/delta                             80      4.0s     4.0s   20.25 Hz     2.19 Hz
/demo/gamma                            365     18.2s    18.2s   20.10 Hz     9.97 Hz
```

All four published at 20 Hz. It exits 2 when the bag contains a gap no event explains.

Notes:

- `-e/--regex` searches the graph on `add` and `set`, and what is being recorded on `remove`. A
  pattern matching nothing is refused on `add`.
- `add /scan:sensor_msgs/msg/LaserScan` names the type explicitly, which is how you record a topic
  whose publisher has not started; otherwise the type is discovered from the graph.
- `resume`, `split` and `record` take `--at` (`+30s`, `14:05`, `2026-09-03T14:05`, or epoch
  seconds). Relative and wall-clock times use your clock while the recorder uses its node clock; on
  one machine or a clock-synced fleet they agree.

## Python library

`ros2 dynrec` covers the shell; `dynrec` covers the supervisor script that changes what is recorded
as the robot changes what it does:

```python
from dynrec import Recorder

with Recorder() as rec:                    # the only recorder on the graph
    rec.set_topics(['/tf', '/odom'])
    rec.add(['/camera/image:sensor_msgs/msg/Image'])
    rec.add(regex='^/lidar/', exclude_regex='intensity')
    rec.profile('navigation')
    rec.pause()
    rec.resume(at='+30s')
    print(rec.status().subscribed_topics)
```

Discovery, the pattern arguments, the `TOPIC:TYPE` form and the `at=` dialect are the same as the
CLI's.

Calls raise when the recorder refuses, the library's version of the CLI's exit code. A partial
success does not, because the service does not treat it as a failure:

```python
change = rec.set_topics(['/scan', '/not_yet_published'])
if not change.complete:
    print('not recording:', change.unavailable)
```

`status().messages_missed` is `None`, never `0`, when the middleware supplies no publication
sequence numbers.

It reads bags back too, where the recorded events pay off:

```python
from dynrec import describe

for channel in describe('/tmp/mybag').channels:
    print(channel.topic, channel.rate, 'vs averaged', channel.averaged_rate)
```

Use it instead of shelling out when you want to watch rather than poll:

```python
rec.on_event(lambda e: print(e.stamp, e.kind, e.action, e.topic or 'all topics'))
rec.on_status(lambda s: print(s.messages_written))
rec.wait_for(lambda s: not s.recording, timeout=60)
```

Both event streams arrive as one `Event` shape on separate subscriptions, so sort by `event.stamp`
(the recorder's clock), not arrival order.

Notes:

- It brings its own rclpy context and executor thread, so it works in a plain script and inside a
  callback of your own node, where handing it your node to spin would deadlock.
- A `Recorder` is long-lived: one client per service, not per call. That is faster and avoids an
  observed rclpy limit where a single node stopped receiving replies after roughly 7,000 calls.
  Close it with `close()` or a `with` block.

With several recorders, `Recorder()` refuses to guess; pass a name, or use `discover()` to list
them.

## Browser UI

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open <http://localhost:8088>.

Tick a topic to record it, untick to stop; untouched topics keep recording without a gap. Pause,
split and stop are buttons.

The topic list has a filter whose regex mode sends one `set_topics` call. The timeline draws a bar
per topic showing when it was recorded, pauses as a band across all of them, built from the
recorder's events; anything from before the page connected is hatched rather than guessed.

Recent changes merges both event streams (topic changes by name, pauses as all topics), ordered by
the recorder's timestamps, so a change made during a pause appears between the pause and the resume.

By default the UI listens on loopback only, because it has no authentication. Pass `bind:=0.0.0.0`
to expose it on a trusted network; inside Docker you must, or the published port has nothing to
forward to.

The page is one static file served by a ROS node on your machine: no account, cloud, separate
application or internet, and no web framework or npm build step. It needs `rclpy`,
`ament_index_python` and the interface packages. Where it cannot measure something it shows
unknown, not a reassuring zero: on one TurtleBot 4 simulator run, `messages_lost_in_transport`
read 0 while 3-4% of messages were absent. Losses the writer itself drops, because the disk cannot
keep up, are shown on their own line: the remedy is local and the page should not send you to
debug the network.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `uri` | `dynamic_bag` | Output bag path. |
| `storage_id` | `mcap` | Storage plugin. |
| `serialization_format` | `cdr` | Message serialization format. |
| `topics` | `[]` | Topics to subscribe at startup. |
| `start_paused` | `false` | Start with recording paused. |
| `snapshot_mode` | `false` | Buffer in memory, write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. `snapshot_mode` needs this or `max_cache_duration` > 0. |
| `max_cache_duration` | `0` | Writer cache bound in seconds; `0` for none. Combines with `max_cache_size`. |
| `max_bagfile_size` | `0` | Split when a file reaches this many bytes; `0` never. |
| `max_bagfile_duration` | `0` | Split every this many seconds; `0` never. |
| `storage_preset_profile` | *(empty)* | Storage plugin preset. mcap: `none`, `fastwrite`, `zstd_fast`, `zstd_small`. |
| `storage_config_uri` | *(empty)* | Storage plugin YAML, overlaid on the preset. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `record_pause_events` | `true` | Write pauses and resumes into the bag. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |
| `status_publish_period` | `1.0` | Seconds between status publications. |

`dynamic_recorder.launch.py` forwards the common arguments. `record_pause_events`,
`status_publish_period` and the profiles are not exposed, so set them through `params_file:=...`;
the full launch-argument list is in [docs/install.md](docs/install.md#launch-arguments).

## ROS 2 compatibility

Developed against Rolling, and at present Rolling only. CI builds all five packages and runs the
full suite against `ros:rolling-ros-base` on pushes to `main` and on PRs.

The table was measured in September 2026 with `colcon build` in each image; upstream support may
have moved since.

| Distro | Status | What happens |
|---|---|---|
| **Rolling** | **Supported** | Builds and passes the suite in CI |
| **Kilted** | **Does not build** | `ament_cmake_ros_core` is present, but not the `ament_cmake_ros_core::ament_ros_defaults` target Rolling exports |
| **Jazzy** (LTS) | **Does not build** | `ament_cmake_ros_core` is not present, so `find_package` fails |
| **Humble** (LTS) | **Not tested** | Older than Jazzy, which already fails on a package Humble does not ship either |

Kilted and Jazzy failed during CMake configuration, before this project compiled. The CMake links
exported namespaced targets directly instead of using `ament_target_dependencies()`; how large a
backport would be is unknown.

## Status

Working end to end and tested in simulation; not yet run on real hardware. The recorder, events,
profiles, CLI, library and browser UI are all implemented. Next is field evidence: hours of
recording, topic changes and message loss under real load, using the sources in `demo/`.

## Layout

```
packages/
  rosbag2_dynamic_recorder_interfaces/   service definitions
  rosbag2_dynamic_recorder/              the node
  rosbag2_dynamic_recorder_cli/          `ros2 dynrec`, the command line client
  dynrec/                                the Python library, for scripts
  rosbag2_dynamic_recorder_ui/           the browser UI
docs/                                    Sphinx documentation sources
src/                                     optional upstream rosbag2 checkout (untracked)
```

Two decisions shape the design. It uses `rosbag2_cpp::Writer` rather than wrapping
`rosbag2_transport::Recorder`, which binds its topic set at construction and can only reconfigure by
tearing every subscription down and closing the bag. And adding a topic costs about half a second,
almost all of it message-definition resolution, so a multi-topic `set_topics` runs in its own
service callback group.

## Tests

```bash
colcon test
colcon test-result --verbose
```

The suite has 186 tests. The C++ package has 41 integration tests that start a real recorder and
drive its services on a private `ROS_DOMAIN_ID`. They check the service contract (common topics
survive `set_topics`, `~/record` restores the selection, scheduled operations fire on time and not
before) and inspect the bag (a topic change does not split the file, untouched topics have no hole,
sparse channels carry their events). The untouched topic's largest gap across two changes is 0.05s,
one publish interval at 20 Hz.

The UI has 16 tests, mostly for the unknown-vs-zero rule; the CLI has 39 (27 unit, 12 subprocess);
`dynrec` has 90 (55 unit, 35 live). Both `ament_python` packages declare
`extras_require={'test': ['pytest']}`; without it `colcon` falls back to `python -m unittest` and
silently collects nothing.

## Development

A Docker dev container is provided for working on the project and for running the recorder against
a simulator without installing ROS on the host.

```bash
docker compose build
docker compose up -d
docker compose exec rosbag2-dev bash -l
```

Inside the container:

```bash
colcon build --packages-select \
  rosbag2_dynamic_recorder_interfaces rosbag2_dynamic_recorder --symlink-install
source install/setup.bash
colcon test --packages-select rosbag2_dynamic_recorder && colcon test-result --verbose
```

The upstream rosbag2 sources are optional, useful for reading the code this builds against but not
required to build:

```bash
git clone https://github.com/ros2/rosbag2.git src && git -C src checkout ae42fb9
```

Notes:

- Port 8088 is published, so launch the UI with `bind:=0.0.0.0`; the default loopback bind is
  inside the container, and Docker would have nothing to forward to. The container does not use
  `network_mode: host`, which on Docker Desktop is the WSL2 VM's network that the host cannot reach.
- Packages live in `packages/`, mounted separately so the optional upstream checkout in `src/`
  stays pristine.
- If you add a package with new dependencies, run
  `rosdep install -r -y --from-paths src --ignore-src` in the container.

## License

Apache-2.0.
