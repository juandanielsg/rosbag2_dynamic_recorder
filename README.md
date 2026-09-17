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

You need ROS 2 Jazzy, Kilted or Rolling. Everything else comes from apt; no need to build rosbag2
from source.

```bash
sudo apt install ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone --branch v0.1.0 https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
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

On Jazzy and Kilted, `record`, `resume`, `split_bagfile` and `stop` are offered under field-identical
copies in `rosbag2_dynamic_recorder_interfaces`, because the stock definitions there predate
scheduling; see [ROS 2 compatibility](#ros-2-compatibility).

`resume`, `split_bagfile` and `record` accept a future timestamp. Node time (`mode: 0`) runs on a
timer and fires even if the robot has gone quiet; it is the only mode `record` supports. Publish
time (`1`) and receive time (`2`) compare against arriving messages, on any recorded topic or
`tracking_topic_name` alone, so they cannot fire while none arrive. A mode outside 0–2, or an
unrecorded `tracking_topic_name`, is rejected.

`~/record` opens a fresh bag and restores the set active at stop. Since rosbag2 will not open over
an existing directory, it uses the next free suffix: `mybag`, then `mybag(1)`.

`~/get_status` reports the bag URI and storage id, recording/paused/snapshot state, start time and
elapsed seconds, the topic set, messages written, messages lost, split count, bag size, free space
on the bag filesystem and whether a low-disk guard stopped the recording.
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
| `~/events/low_disk` | `LowDiskEvent`, also written into the bag |
| `~/events/bag_size_limit` | `BagSizeLimitEvent`, also written into the bag |
| `~/events/write_split` | `WriteSplitEvent` |
| `~/events/messages_lost` | `MessagesLostEvent` |

Writing the first four into the bag is what makes a gap legible; without them it is
indistinguishable from a dropout, a crash or a network fault.

`MessagesLostEvent` is the stock `rosbag2_interfaces` type on Rolling and a field-identical copy from
`rosbag2_dynamic_recorder_interfaces` on Jazzy and Kilted, whose rosbag2 has none. Its
`messages_lost_in_recorder` is only ever non-zero on Rolling; older writers do not report their own
losses, so there the count is unknown rather than zero.

- `SubscriptionChangeEvent` marks where one topic stops while others carry on. Disable with
  `record_subscription_events:=false`.
- `PauseEvent` marks where the whole recording stops: a gap in every topic at once, the same shape
  as a crash, a network fault, or the rosbag2 stall that drops about 1.2s from every topic every
  ~31.2s. The stock `Pause` service has empty request and response fields and can carry no reason;
  this event can. Disable with `record_pause_events:=false`.
- `LowDiskEvent` marks the end of a recording the recorder stopped itself to protect the disk; see
  [Disk space](#disk-space). Disable the in-bag copy with `record_low_disk_events:=false`.
- `BagSizeLimitEvent` marks the end of a recording that reached its `max_bag_size` cap; see
  [Bag size](#bag-size). Disable the in-bag copy with `record_bag_size_limit_events:=false`.

While paused no messages are written, but these events still are; an event suppressed by the pause
it describes would leave the gap unexplained. `reason` distinguishes `service:pause`,
`service:resume`, `service:toggle_paused`, `schedule:resume` and `startup` (the last covers
`start_paused` and a `~/record` while paused, both opening a bag with an empty beginning).

`ros2 bag play` replays the sparse channels at full fidelity and `info` and `convert` process them.
One caveat: `mcap info` averages a channel's rate over the bag, so a topic that ran at 20 Hz and
then stopped is shown as 4.43 Hz. The recorded events recover the real rate.

## Disk space

`max_bagfile_size` and `max_bagfile_duration` bound the bag, not the disk. Anything else growing on
the same filesystem -- system logs, core dumps, a second recorder, a software update -- can still
fill it, and a full disk on an embedded robot also stops logging and DDS shared memory. Set
`min_free_space` (bytes) or `min_free_space_percent` (percentage of the filesystem); when free
space falls below either, the recorder logs once, writes a `LowDiskEvent`, and stops. With both
unset the behaviour is unchanged.

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag min_free_space:=1073741824     # keep 1 GiB free
```

The two thresholds combine to the stricter one. The check runs every `storage_check_period`
seconds (default 1.0). The event is written into the bag before the writer closes, so the recording
ends with an explanation rather than looking like a crash, and `ros2 dynrec info` reports it.
`~/get_status` carries `free_space_bytes`, `total_space_bytes` and `stopped_for_low_disk`, so a
supervisor script can watch the same figures the recorder acts on.

## Bag size

The disk guard bounds what is left on the filesystem; `max_bag_size` bounds the bag itself. It is
the cap for an unattended recorder: `max_bagfile_size` only rolls to a new file, so a recording left
running keeps growing across splits. When the bag directory, across every split, grows past
`max_bag_size` bytes the recorder logs once, writes a `BagSizeLimitEvent`, and stops. `0`, the
default, disables it.

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag max_bag_size:=10737418240     # never more than 10 GiB
```

The check runs on the same `storage_check_period` timer as the disk guard and measures size on
disk, so the bag can overshoot the cap by one check period's worth of data plus whatever the
storage plugin had not yet flushed. Treat it as a ceiling with some give, not an exact size.
`~/get_status` carries `max_bag_size` and `stopped_for_max_bag_size`; `~/record` opens a fresh bag
with the same cap.

## Simulation time

`use_sim_time:=true` does what `ros2 bag record --use-sim-time` does: messages are stamped on the
node clock driven by `/clock`, so the bag's timeline is the simulation's, and nothing is opened or
subscribed until `/clock` has been heard, because a bag opened on a clock that reads zero would
begin in 1970. The recorder answers in the meantime -- `~/status` says `waiting_for_clock`, and a
call that needs an open bag is refused with a message that says why. Its own events were always on
the node clock, so the two timelines in one bag agree; and node-time schedules follow the simulation
when it runs slow or stops. Details in [docs/services.md](docs/services.md#simulation-time).

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

All four event streams arrive as one `Event` shape on separate subscriptions, so sort by
`event.stamp` (the recorder's clock), not arrival order. The low-disk and bag-size stops are among
them, so a supervisor can react without polling:
`rec.on_event(lambda e: e.kind in ('low_disk', 'bag_size_limit') and shutdown())`.

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
| `use_sim_time` | `false` | Stamp messages on the node clock driven by `/clock`, and open nothing until it has started. See [Simulation time](#simulation-time). |
| `snapshot_mode` | `false` | Buffer in memory, write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. `snapshot_mode` needs this or `max_cache_duration` > 0. |
| `max_cache_duration` | `0` | Writer cache bound in seconds; `0` for none. Combines with `max_cache_size`. |
| `max_bagfile_size` | `0` | Split when a file reaches this many bytes; `0` never. |
| `max_bagfile_duration` | `0` | Split every this many seconds; `0` never. |
| `storage_preset_profile` | *(empty)* | Storage plugin preset. mcap: `none`, `fastwrite`, `zstd_fast`, `zstd_small`. |
| `storage_config_uri` | *(empty)* | Storage plugin YAML, overlaid on the preset. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `record_pause_events` | `true` | Write pauses and resumes into the bag. |
| `record_low_disk_events` | `true` | Write a low-disk stop into the bag. |
| `record_bag_size_limit_events` | `true` | Write a bag-size-limit stop into the bag. |
| `min_free_space` | `0` | Stop recording below this many bytes free on the bag filesystem; `0` disables. |
| `min_free_space_percent` | `0.0` | Same, as a percentage of the filesystem; `0.0` disables. The stricter applies. |
| `max_bag_size` | `0` | Stop recording once the bag directory, across every split, exceeds this many bytes; `0` disables. |
| `storage_check_period` | `1.0` | Seconds between free-space and bag-size checks when either limit is set. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |
| `status_publish_period` | `1.0` | Seconds between status publications. |

`dynamic_recorder.launch.py` forwards the common arguments, including `use_sim_time`, `min_free_space`,
`min_free_space_percent` and `max_bag_size`. `record_pause_events`, `record_low_disk_events`,
`record_bag_size_limit_events`, `storage_check_period`, `status_publish_period` and the profiles
are not exposed, so set them
through `params_file:=...`; the full launch-argument list is in
[docs/install.md](docs/install.md#launch-arguments).

## ROS 2 compatibility

Jazzy, Kilted and Rolling, from one branch. CI builds all five packages and runs the full suite in
`ros:jazzy-ros-base`, `ros:kilted-ros-base` and `ros:rolling-ros-base` on pushes to `main` and on
PRs; a distro is listed as supported because that job is green on it. Releases are tags on `main`
(`v0.1.0` is the first), each placed on a commit that job has passed, so one tag is a known-good
ref on every distro; there are no per-distro branches.

| Distro | Status | Differences |
|---|---|---|
| **Rolling** | Supported | None; this is what the project is written against (rosbag2 0.34). |
| **Kilted** | Supported | Below. Kilted reaches end of life in November 2026. |
| **Jazzy** (LTS) | Supported | Below. |
| **Humble** (LTS) | Not tested | Older than Jazzy; would need its own look. |

The differences on Jazzy and Kilted come from what their rosbag2 (0.26 and 0.32) does not have. The
recorder's CMake probes the installed headers rather than trusting a version number, and
`dynrec.services` makes the same decisions for clients:

- **Four services are offered under copies.** `Record`, `Resume` and `SplitBagfile` gained their
  scheduling fields and `Stop` its return code in rosbag2 0.34; Kilted has no `Record` at all. Where
  the installed stock definition is field-identical to 0.34 the recorder offers the stock type, so a
  client written for `ros2 bag record` drives it unchanged; elsewhere it offers the copy from
  `rosbag2_dynamic_recorder_interfaces`, with the same fields and the same behaviour, so scheduling
  works everywhere. `ros2 service type ~/resume` tells you which. `pause`, `toggle_paused`,
  `is_paused` and `snapshot` are identical on every distro and are always stock.
- **`MessagesLostEvent` is a copy, and `messages_lost_in_recorder` stays 0.** Only Rolling's writer
  reports its own losses. Transport losses are counted on every distro.
- **`max_cache_duration` is refused.** Older `StorageOptions` have no time bound on the writer cache.
  The parameter still exists so a params file loads everywhere, but a non-zero value fails at
  startup rather than being dropped silently; use `max_cache_size`.

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
.devcontainer/                           dev container, the same environment as CI
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

A dev container is provided for working on the project without installing ROS on the host. It is
the same environment as CI, one `ros:<distro>-ros-base` image plus the dependencies from our own
manifests, and lives in `.devcontainer/` so that VS Code offers to reopen the repository in it.
From the repository root:

```bash
docker compose --project-directory .devcontainer build      # ROS_DISTRO=jazzy|kilted|rolling
docker compose --project-directory .devcontainer up -d
docker compose --project-directory .devcontainer exec rosbag2-dev bash -l
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

The upstream rosbag2 sources are optional, useful for reading the code this builds against but not
required to build. Anything local of that kind goes in `.devcontainer/docker-compose.override.yml`,
which compose merges in and git ignores:

```bash
git clone https://github.com/ros2/rosbag2.git src && git -C src checkout ae42fb9
```

Notes:

- Port 8088 is published, so launch the UI with `bind:=0.0.0.0`; the default loopback bind is
  inside the container, and Docker would have nothing to forward to. The container does not use
  `network_mode: host`, which on Docker Desktop is the WSL2 VM's network that the host cannot reach.
- Packages live in `packages/`, mounted at `src/packages` so a plain `colcon build` finds them.
- Dependencies are installed into the image from the manifests. If you add a `<depend>`, rebuild
  the image, or run `rosdep install -r -y --from-paths src --ignore-src` in the container.

## License

Apache-2.0.
