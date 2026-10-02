# Recording profiles

A profile is a named set of topics. Applying one makes the recorder record exactly those topics:
it drops what the profile leaves out, adds what it names, and leaves the topics the old and new sets
share recording without a gap. It is the one-call way to follow a robot's operating mode: idle,
navigating, inspecting.

With stock `ros2 bag record` the same change means stopping and restarting, which leaves a hole in
every topic and starts a new bag. Here it is a topic change like any other: the bag stays open, and
the recorder writes a `SubscriptionChangeEvent` into it for every topic that starts or stops, so the
bag explains its own boundaries.

## Declaring profiles

Profiles are recorder parameters: a list of names, and one topic list per name.

```yaml
rosbag2_dynamic_recorder:
  ros__parameters:
    profile_names: ["idle", "navigation", "inspection"]
    profiles:
      idle: ["/tf", "/tf_static", "/odom", "/imu"]
      navigation: ["/tf", "/tf_static", "/odom", "/imu", "/scan"]
      inspection:
        - "/tf"
        - "/tf_static"
        - "/odom"
        - "/imu"
        - "/scan"
        - "/rgbd_camera/image"
        - "/rgbd_camera/depth_image"
```

- Only the names in `profile_names` are read. A list under `profiles` with no entry in
  `profile_names` is ignored.
- The names keep their order: the CLI lists them and the UI draws their buttons in that order.
- Each topic list is sorted and de-duplicated when it is read.
- A profile with no topics is accepted with a warning at startup. Applying it records nothing.
- Entries are plain topic names. There is no type syntax and no regular expression: the type of
  each topic is read from the graph when the profile is applied.

Profiles are read once, at startup. To change one, edit the file and restart the recorder.

A worked example, written for the TurtleBot 4 simulator's topics, ships with the package as
`config/profiles.example.yaml`.

## Starting a recorder with profiles

Pass the file as `params_file`:

```bash
ros2 launch rosbag2_dynamic_recorder dynamic_recorder.launch.py \
  uri:=/tmp/mybag params_file:=profiles.yaml
```

The browser UI's launch file takes the same argument:

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py \
  uri:=/tmp/mybag params_file:=profiles.yaml
```

Without a launch file, give the node the file, or the parameters one by one:

```bash
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:=/tmp/mybag --params-file profiles.yaml
```

There is no parameter that starts the recorder in a profile. Set `topics` to the same list as the
profile you want, or apply the profile once the recorder is up. Either way the recorder reports
that profile as active, because it compares what it is recording with each profile's list.

## Switching

Every client calls the same `~/set_profile` service. A switch is refused while the recorder is
stopped, with the reason, since there is no bag to record into.

**Browser UI.** Each profile is a button, with the active one highlighted and its topics in the
button's tooltip. Pressing one applies it.

**Command line.**

```bash
ros2 dynrec profiles              # list them; the active one is marked *
ros2 dynrec profiles --names      # just the names, one per line
ros2 dynrec profile navigation    # apply one
```

`ros2 dynrec profile` prints what is now recorded, what stopped, and anything it could not
subscribe.

**Python.**

```python
from dynrec import Recorder

with Recorder() as rec:
    change = rec.profile('navigation')
    if change.unavailable:
        print('not recording yet:', change.unavailable)
    print('active profile:', rec.profiles().active)
```

**Service call.**

```bash
ros2 service call /rosbag2_dynamic_recorder/set_profile \
  rosbag2_dynamic_recorder_interfaces/srv/SetProfile "{name: navigation}"
```

An unknown name is refused, and the reply lists the profiles that are configured.

## What a switch does

Going from `navigation` to `inspection` in the example above:

| Topic | Before | After | What happens |
|---|---|---|---|
| `/tf`, `/tf_static`, `/odom`, `/imu`, `/scan` | recorded | recorded | untouched: no gap, no event |
| `/rgbd_camera/image`, `/rgbd_camera/depth_image` | not recorded | recorded | subscribed; a `SUBSCRIBED` event goes into the bag |

and back from `inspection` to `idle`:

| Topic | Before | After | What happens |
|---|---|---|---|
| `/tf`, `/tf_static`, `/odom`, `/imu` | recorded | recorded | untouched |
| `/scan`, both cameras | recorded | not recorded | dropped; an `UNSUBSCRIBED` event goes into the bag |

The recorder drops before it adds, so the expensive topics of the old profile are gone before the
new profile's arrive. Topics added by hand stay only if the new profile names them: applying a
profile makes the recorded set equal to the profile, whatever was recorded before.

The events carry the reason `service:set_profile:<name>`, so reading the bag back shows which
profile caused each channel to start or stop. They are written into the bag unless the recorder
runs with `record_subscription_events:=false`. The bag is never split by a switch.

## Which profile is active

The recorder does not remember which profile was applied last. It compares what it is recording
with each profile's list, every time it reports its status, and names the profile that matches
exactly, or none.

That keeps the answer true when something else changes the topics. Add one topic by hand and no
profile is active any more, even though you started from one. Reach a profile's exact set by
other means and that profile is reported, though it was never applied.

The status carries it as `active_profile`, `ros2 dynrec profiles` marks it with `*`, and the UI
highlights its button and says "The current selection does not match a profile" when none does.

## When a topic is not there

A topic in the profile that has no publisher when the profile is applied cannot be subscribed: its
type comes from the graph, and the graph does not have it. The same goes for a topic published
under more than one type, since the recorder will not guess which one you meant. The switch still goes through for every
other topic, and the missing ones come back as `unavailable`. Until they are recorded, the profile
does not match exactly, so none is reported active.

Apply the profile again once the publisher is up. Topics already recorded are untouched, so
repeating a switch costs nothing but the missing subscriptions.

If none of a profile's topics can be subscribed, the call is reported as a failure.

## Designing profiles

- **Nest them.** Give every profile the core it needs to interpret anything else, such as
  transforms and odometry, and add to it. Topics shared between profiles are the ones that never
  get a gap, so a large shared core means fewer seams in the data.
- **Put the expensive topics in one profile.** Cameras and point clouds belong in the profile for
  the task that needs them, so switching away from it is what saves the bandwidth and the disk.
- **Name them after what the robot is doing**, not after the topics. A supervisor that knows the
  robot is inspecting should not need to know which topics inspection needs.
- **Keep the list in version control** next to the launch files that use it, since profiles are
  fixed for the life of the recorder.
