# rosbag2_dynamic_recorder

A ROS 2 bag recorder whose **recorded topic set can change while it is recording** — add a topic,
drop a topic, or swap the whole set — without stopping the writer and without splitting the bag.

Recording continues uninterrupted on every topic you did not touch.

## Why

`rosbag2_transport::Recorder` fixes its topic set at construction from `RecordOptions`. Changing
it means `stop()` → reconfigure → `record()`, which:

- **tears down every subscription**, so unrelated topics lose messages every time you add or
  remove one; and
- **closes the bag**, so each change forces a new file.

For a robot that records different topics per operational mode, that means a gap on `/tf` and
`/odom` every time an unrelated diagnostic topic is toggled, plus a file set whose boundaries
encode two unrelated things at once.

This records into **one continuous MCAP stream**. A topic that gets dropped and re-added becomes a
sparse channel with a hole — which MCAP supports natively, and which `ros2 bag play`, `mcap info`
and Foxglove all handle.

## What it is not

Not a replacement for rosbag2. Bags are written by `rosbag2_cpp::Writer`, so the output is a
standard rosbag2 bag and the whole ecosystem keeps working. Only the *control plane* is different.

## Services

Topic management:

| Service | Purpose |
|---|---|
| `~/subscribe_topics` | Add topics. Types are discovered from the graph if not given. |
| `~/unsubscribe_topics` | Drop topics. Recorded messages are kept. |
| `~/set_topics` | Replace the whole set atomically. Topics in both old and new are untouched. |
| `~/get_subscribed_topics` | Current set. |
| `~/set_profile` | Apply a named topic set. Same guarantee as `set_topics`. |
| `~/get_profiles` | List the configured profiles. |

The first three also take **`regex`** and **`exclude_regex`**, so a topic set can be selected
by pattern instead of enumerated — the same ECMAScript search semantics as stock
`ros2 bag record -e`, so `camera` finds `/robot/camera/image`. `exclude_regex` is applied to
the combined selection, which makes *everything under `/robot` except the depth camera* a
single call. A pattern never matches this recorder's own topics (its events are already
written into the bag directly, so subscribing would record them twice) nor hidden topics.
An invalid expression is refused with the reason rather than quietly matching nothing.

Recording control. These use the stock `rosbag2_interfaces` definitions, so a client written
against standard rosbag2 drives this node unchanged:

| Service | Purpose |
|---|---|
| `~/pause` `~/resume` `~/toggle_paused` `~/is_paused` | Stop writing without tearing down subscriptions. |
| `~/split_bagfile` | Close the current file and open the next. |
| `~/snapshot` | Flush the in-memory buffer. Requires `snapshot_mode`. |
| `~/stop` | Close the bag. |
| `~/record` | Open a new bag and start again, restoring the previous topic selection. |
| `~/get_status` | Everything a UI or CLI needs for a status line, in one round trip. |

`return_code` is `0` on success, `1` when nothing was actioned or on error.

`resume`, `split_bagfile` and `record` also accept a **future timestamp** instead of acting
immediately:

- **Node time** (`mode: 0`, and the only option for `record`) is driven by a timer, so it fires
  even if the robot has gone quiet.
- **Publish time** (`1`) and **receive time** (`2`) are compared against arriving messages, either
  on any recorded topic or on `tracking_topic_name` alone. These cannot fire while no messages are
  arriving, which is inherent to what they mean.

A mode outside 0–2, or a `tracking_topic_name` nobody is recording, is rejected rather than
accepted — a schedule keyed to a topic that is not being recorded would simply wait forever.

Stopping is not a dead end: `~/record` opens a fresh bag and re-subscribes whatever was being
recorded when you stopped. Since rosbag2 will not open over an existing bag directory, the new one
gets the next free suffix — `mybag`, then `mybag(1)`.

`~/get_status` reports the bag URI and storage id, recording/paused/snapshot state, start time and
elapsed seconds, the subscribed topics, messages written, messages lost, split count and bag size
on disk. Note that `bag_size_bytes` counts bytes *flushed*, not captured: the writer caches, so it
reads 0 early in a recording. Use `messages_written` to answer "is it recording?".

## Profiles

A named topic set, switched in one call. The fleet case this exists for: a supervisor changes the
robot's operating mode and the recorded topic set follows, **without interrupting the topics the
two modes share**.

```yaml
rosbag2_dynamic_recorder:
  ros__parameters:
    profile_names: ["idle", "navigation", "inspection"]
    profiles:
      idle: ["/tf", "/odom", "/imu"]
      navigation: ["/tf", "/odom", "/imu", "/scan"]
```

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py   uri:=/tmp/mybag params_file:=profiles.yaml
```

In the UI they appear as buttons; the active one is highlighted. A worked example ships at
`rosbag2_dynamic_recorder/config/profiles.example.yaml`, using the TurtleBot 4 simulator's topics.

`active_profile` in the status is **derived from the live topic set, not remembered**. Tick one
extra topic and it goes empty, because no profile is in effect any more. Drop a topic and land
exactly on another profile's set and it reports that one — the recorder really is recording that
profile, whatever command got it there. A remembered label would keep claiming a profile that had
stopped being true.

## Events

| Topic | Message |
|---|---|
| `~/events/subscription_change` | `SubscriptionChangeEvent` — **also written into the bag** |
| `~/events/pause` | `PauseEvent` — **also written into the bag** |
| `~/events/write_split` | `WriteSplitEvent` |
| `~/events/messages_lost` | `MessagesLostEvent` |

Recording these into the bag is the point: a gap mid-bag is otherwise indistinguishable from a
dropout, a crash, or a network fault. The event says what changed, when, and why — in-stream and
timestamped.

The two cover different shapes of gap, and you need both:

- **`SubscriptionChangeEvent` explains a channel that stops.** One topic goes sparse while the
  others carry on. Disable with `record_subscription_events:=false`.
- **`PauseEvent` explains a bag that stops.** A pause leaves a hole in *every* topic at once —
  which is exactly what a crash, a network fault, or the environmental stall in
  [notes/recording-stall.md](notes/recording-stall.md) also look like. Stock
  `rosbag2_interfaces/srv/Pause` has an empty request and response, so it can carry no
  explanation; this event is ours. Disable with `record_pause_events:=false`.

Downstream tools cope with the resulting sparse channels — `ros2 bag play` replays them at exact
fidelity, and `info`/`convert` handle them correctly. One caveat worth knowing: `mcap info`
averages a channel's rate over the whole bag, so a topic that ran at 20 Hz and then stopped is
displayed as 4.43 Hz. The events are what turn that back into the truth. Measured in
[notes/downstream-tools.md](notes/downstream-tools.md).

Note the deliberate asymmetry: while paused **no messages are written, but these events still
are**. An event suppressed by the pause it describes would leave exactly the hole it exists to
account for. `reason` distinguishes `service:pause`, `service:resume`, `service:toggle_paused`,
`schedule:resume`, and `startup` — the last covering both `start_paused` and a `~/record` issued
while still paused, either of which opens a bag whose head is empty.

```bash
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:=/tmp/mybag -p "topics:=['/scan','/odom']"

ros2 service call /rosbag2_dynamic_recorder/subscribe_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SubscribeTopics "{topics: ['/diagnostics']}"

ros2 service call /rosbag2_dynamic_recorder/set_topics \
  rosbag2_dynamic_recorder_interfaces/srv/SetTopics "{topics: ['/scan','/odom','/tf']}"
```

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `uri` | `dynamic_bag` | Output bag path. |
| `storage_id` | `mcap` | Storage plugin. |
| `serialization_format` | `cdr` | Message serialization format. |
| `topics` | `[]` | Topics to subscribe at startup. |
| `start_paused` | `false` | Start with recording paused. |
| `snapshot_mode` | `false` | Buffer in memory, write only on `~/snapshot`. |
| `max_cache_size` | `104857600` | Writer cache in bytes. Must be > 0 for `snapshot_mode`. |
| `record_subscription_events` | `true` | Write subscription changes into the bag. |
| `record_pause_events` | `true` | Write pauses and resumes into the bag. |
| `messages_lost_report_period` | `5.0` | Seconds between `MessagesLostEvent`. `0` disables. |

There is a launch file too:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

## Status

Working and verified end to end; not yet run on real hardware. See
[notes/roadmap.md](notes/roadmap.md).

- **M1 done** — topic management, verified by
  [`m1_smoke_test.sh`](packages/rosbag2_dynamic_recorder/test/m1_smoke_test.sh): topics added and
  removed mid-recording land in a single MCAP with no gap on untouched topics.
- **M2 done** — recording control, events and provenance, verified by
  [`m2_smoke_test.sh`](packages/rosbag2_dynamic_recorder/test/m2_smoke_test.sh).
- **M3 done** — named topic-set profiles switched atomically on mode change, sharing `set_topics`'
  guarantee that topics common to both sets are never torn down.
- **Clients done** — `ros2 dynrec` and the browser UI, in that order: the CLI first so the service
  API was proved by something scriptable before anything was built on top of it.
- **M4 next** — field evidence. Hours recorded, topic changes performed, and message loss on
  untouched topics measured under real load rather than in a smoke test.

## Layout

```
packages/
  rosbag2_dynamic_recorder_interfaces/   service definitions
  rosbag2_dynamic_recorder/              the node
  rosbag2_dynamic_recorder_cli/          `ros2 dynrec`, the command line client
  dynrec/                                the Python library, for scripts
  rosbag2_dynamic_recorder_ui/           the browser UI
notes/                                   architecture, spike findings, roadmap
spike/                                   throwaway validation of the core premise
src/                                     optional upstream rosbag2 checkout (untracked)
```

`notes/` is worth reading before changing anything —
[architecture.md](notes/architecture.md) records *why* this is built on `Writer` rather than
wrapping `Recorder`, and [spike-plan.md](notes/spike-plan.md) has measured timings (adding a topic
costs ~0.4–0.6s, almost all of it message-definition resolution) plus one retracted conclusion
worth not rediscovering.

## Install

You need ROS 2 Rolling. Everything else comes from apt; there is no need to build rosbag2 from
source.

```bash
sudo apt install ros-rolling-rosbag2 ros-rolling-rosbag2-storage-mcap

mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/juandanielsg/rosbag2_dynamic_recorder.git
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

The build is two small packages and takes about a minute. Then:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag topics:="['/scan','/odom']"
```

## The command line

`ros2 dynrec` drives a running recorder without service-call syntax, and without a display:

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
ros2 dynrec pause | resume | toggle | split | snapshot | stop | record
```

With one recorder on the graph there is nothing to configure: it is found by looking for a node
offering `~/get_status` of our type, so a recorder launched under any name or in any namespace is
found, and a node that merely borrows the name is not. Pass `--node` when several are running —
with more than one it refuses to guess and lists them, because guessing could stop the wrong
recording.

Every verb **exits non-zero when the recorder refuses**, so a supervisor script can rely on the
exit code alone and does not have to parse `return_code` out of a service reply:

```bash
ros2 dynrec add /scan || echo "could not record /scan"
```

`status --json` gives the same information as data. Fields the recorder cannot vouch for come out
as `null`, never as a convenient zero — `messages_missed` is `null` when the middleware supplies
no publication sequence numbers, which is the difference between "nothing was missed" and "no
idea".

Three things worth knowing:

- `-e/--regex` selects by pattern rather than by name, and `--exclude-regex` filters the
  result. On `add` and `set` the pattern searches the graph; on `remove` it searches what is
  actually being recorded, since only a recorded topic can be dropped. A pattern that matches
  nothing is refused on `add` — adding nothing is a request that was not met, and the likely
  cause is a typo.
- `add /scan:sensor_msgs/msg/LaserScan` names the type explicitly. That is how you record a topic
  whose publisher has not started yet; without it the type is discovered from the graph, which
  needs the topic to be there already.
- `resume`, `split` and `record` take `--at` for the scheduled variants: `--at +30s`, `--at 14:05`,
  `--at 2026-09-03T14:05`, or epoch seconds. Relative and wall-clock times are resolved against
  *your* clock while the recorder compares against its node clock; on one machine or a clock-synced
  fleet those agree, and where they might not, use an absolute time.

Built before the browser UI on purpose. A service API is only as good as the thinnest client that
can drive it, and proving it against something scriptable first is what keeps the API honest.

## The Python library

`ros2 dynrec` covers the shell. `dynrec` covers the supervisor — the mission script that changes
what is recorded because the robot changed what it is doing:

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

Discovery, the pattern arguments, the `TOPIC:TYPE` form and the `at=` dialect are all the same as
the CLI's, so a command that worked in a shell keeps working when it becomes a script.

**Calls raise when the recorder refuses**, which is the library's version of the CLI's exit code —
a script reads top to bottom with no return codes to check. A *partial* success does not raise,
because the service does not call it a failure either:

```python
change = rec.set_topics(['/scan', '/not_yet_published'])
if not change.complete:
    print('not recording:', change.unavailable)   # the only warning there is
```

**The same unknown-versus-zero rule** the CLI's `--json` and the browser UI enforce:
`status().messages_missed` is `None` when the middleware supplies no publication sequence
numbers, never `0`.

The reason to prefer this over shelling out to the CLI is that it can *watch* rather than poll:

```python
rec.on_event(lambda e: print(e.stamp, e.kind, e.action, e.topic or 'all topics'))
rec.on_status(lambda s: print(s.messages_written))
rec.wait_for(lambda s: not s.recording, timeout=60)
```

Both event streams arrive flattened into one shape, because a channel that stops and a bag that
stops are the same question asked twice. They come in on separate subscriptions, so sort by
`event.stamp` — the recorder's clock — rather than trusting arrival order.

Two things worth knowing:

- **It brings its own rclpy context and its own executor thread.** So it works in a plain script
  with no ROS of its own, *and* inside a callback of your own node — where handing it your node
  and spinning that would deadlock, since the executor calling you already owns it. Reacting to
  an event by changing the recording is the case this exists for, so it must not be the case that
  hangs.
- **A `Recorder` is meant to be long-lived.** It holds one client per service rather than building
  one per call, which is both faster and, on the evidence in
  [roadmap.md](notes/roadmap.md), the difference between a script that still works after a few
  thousand calls and one that quietly stops getting replies. Close it with `close()` or the
  `with` block.

With several recorders running, `Recorder()` refuses to guess — as the CLI does, and for the same
reason. Name one, after asking who is out there:

```python
from dynrec import Recorder, discover

for name in discover():
    Recorder(name).stop()
```

## The browser UI

The way to use this without learning service-call syntax:

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open **http://localhost:8088**.

Tick a topic to start recording it, untick to stop. Topics you did not touch keep recording
without a gap. Pause, starting a new file, and stopping are buttons.

The **topic list has a filter**, with a regex mode that sends the pattern to the recorder as a
single `set_topics` call rather than ticking boxes one at a time. The **recording timeline**
draws one bar per topic showing when it was actually being recorded, with pauses as a band
across all of them — built from the events the recorder already publishes, and the one view a
stock recorder cannot produce, since every boundary on it would otherwise have been a separate
file. Anything from before the page connected is hatched rather than guessed at.

**Recent changes** shows both event streams merged: topic changes by name, and pauses as *all
topics*, because that is what a pause affects. They are ordered by the recorder's own timestamps
rather than by arrival, since the two come in on separate subscriptions — so a topic change made
while paused appears in its real place between the pause and the resume.

By default it listens on **loopback only**, because it has no authentication: anyone who can
reach the port can stop your recording. Pass `bind:=0.0.0.0` to expose it on a trusted network —
and note that inside Docker you must, or the published port has nothing to forward to.

The page is a single static file served by a ROS node **on your own machine** — no account, no
cloud, no separate application, and no internet, so it works on a robot with no network. It
depends on nothing beyond `rclpy` and the Python standard library: no web framework and no npm
build step, because every dependency there is an install barrier.

It also refuses to overstate what it knows. Where the recorder cannot actually measure something,
the UI says **unknown** rather than showing a reassuring zero — see
[notes/recording-stall.md](notes/recording-stall.md) for the case where `messages_lost` read 0
while 3–4% of messages were genuinely absent.

## Tests

```bash
colcon test
colcon test-result --verbose
```

**Thirty-three integration tests** start a real recorder process and drive its services, on a private
`ROS_DOMAIN_ID` with their own publishers — no simulator needed, and they cannot collide with
anything else running on the machine.

Some check the service contract: that a topic present before and after a `set_topics` is never
torn down, that a stopped recorder refuses for the right reason, that `~/record` restores the
previous selection, and that a scheduled operation has *not* fired before its time and then
does — asserted in both directions, so none of them can pass vacuously.

The rest open the resulting bag and check what was actually written — that a topic change does not
split the file, that the untouched topic spans the whole recording without a hole, that the
dropped and added topics appear as sparse channels, and that the bag carries the
`SubscriptionChangeEvent`s explaining them. The pause tests are two-sided on purpose: they assert
both that a pause-sized hole is really in the data *and* that the `PauseEvent`s sit at its two
edges, since either half alone would pass against a bag that explains nothing. Measured, the
untouched topic's largest gap across two
changes is **0.05s — one publish interval**, which is the central claim of this project stated as
a number rather than a promise.

**Ten unit tests** cover the UI's rigor rule: unmeasurable values must render as *unknown*, never
as a convenient zero. `build_state()` is a plain function precisely so this is testable without a
ROS graph, and `recent_events()` was extracted for the same reason — it pins that the two event
streams are ordered by when things happened, not by when they arrived.

**Thirty-seven tests** cover `ros2 dynrec`. Twenty-five are unit tests over the parts with
judgement in them — how a recorder is identified on the graph, what `--at` accepts, and the same
unknown-versus-zero rule at the `--json` boundary where a wrong number is easiest to pipe onward.
The other twelve run the real command as a subprocess against a real recorder, because every unit
test imports the code directly and would still pass if the entry points were wrong and
`ros2 dynrec` did not exist at all.

**Fifty-eight tests** cover the `dynrec` library. Thirty-six are unit tests over the modules that
hold the judgement — discovery, the `at=` dialect, and the result types where the unknown-versus-
zero rule is enforced for the third time. Those modules import no ROS at all, which is what makes
them runnable without a graph and is the same split `build_state()` made. The other twenty-two
drive a real recorder: they cover the parts only a live graph can prove, chiefly that the client's
own context and executor thread work, and that a script can watch the event stream while making
calls on the same object.

One colcon quirk, now fixed rather than worked around: colcon chooses its Python test runner from
`setup.py`, not `package.xml`, and without a declared test dependency on pytest it falls back to
`python -m unittest`, which collects nothing here. It reported that only on stderr, so
`colcon test` looked like it passed while running no Python tests at all. Both `ament_python`
packages now declare `extras_require={'test': ['pytest']}`, so plain `colcon test` runs them.

## Development

A Docker dev container is provided for working on the project itself, and for running the
recorder against a simulator without installing ROS on the host.

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
bash src/packages/rosbag2_dynamic_recorder/test/m1_smoke_test.sh
```

The upstream rosbag2 sources are **optional** — useful for reading the code this builds against,
not required to build:

```bash
git clone https://github.com/ros2/rosbag2.git src && git -C src checkout ae42fb9
```

Notes:

- The container publishes port 8088, so the browser UI is reachable at `http://localhost:8088`
  from the host — but you must launch the UI with `bind:=0.0.0.0`, since the default loopback bind
  is loopback *inside the container* and Docker would have nothing to forward to. 8088 rather than 8080 because 8080 is very often already taken. It does **not** use `network_mode: host`: on Docker Desktop that is the WSL2 VM's
  network, which the host OS cannot reach.
- Our packages live in `packages/` and `spike/`, mounted into the workspace separately, so the
  optional upstream checkout in `src/` stays pristine.
- The image carries the workspace dependencies. If you add a package with new dependencies, run
  `rosdep install -r -y --from-paths src --ignore-src` inside the container.

## License

Apache-2.0.
