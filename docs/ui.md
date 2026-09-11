# The browser UI

The way to use this without learning service-call syntax.

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open **http://localhost:8088**.

Tick topics to change what is being recorded. Profiles appear as buttons with the active one
highlighted; there is a status line, a feed of recent changes, and a recording timeline. Pause,
starting a new file, saving a snapshot and stopping are buttons too, and after a stop they give way
to **Start recording**, which opens a fresh bag and restores the previous selection.

## The timeline

One bar per topic showing when it was being recorded, with pauses drawn as a band across all of
them. It is built entirely from the `SubscriptionChangeEvent` and `PauseEvent` messages the recorder
already publishes, so it needed no recorder change. With stock rosbag2, every one of those
boundaries would have been a separate file.

Two details are deliberate:

- **The axis is drawn on the recorder's clock.** Event stamps are the recorder's, and mixing in the
  browser's would put every bar in the wrong place, which is not hypothetical on a host measured
  stepping ~120 ms backwards.
- **Anything before the page connected is hatched and labelled, not extended.** The recorder keeps
  only ten events for a late joiner, so earlier history genuinely was not observed, and the chart
  says so instead of guessing. Same rule as reporting unknown rather than zero.

## Filtering topics

The topic picker has a filter, with a regex mode that hands the pattern to the recorder as a single
`set_topics {regex: ...}` call rather than ticking boxes one at a time. Three test topics need no
filter; a robot with a hundred does.

## Endpoints

The page is served by a small node that also exposes the two JSON endpoints it uses: `GET
/api/state` returns everything the page renders in one response, and `POST /api/action` takes
`{"action": "...", "topics": [...], "regex": "...", "exclude_regex": "...", "name": "..."}` for the
same operations the buttons perform. They are not a versioned API, but they are enough to drive the
UI from a script.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `recorder_node` | `/rosbag2_dynamic_recorder` | Recorder the UI drives. |
| `port` | `8088` | Port to serve on. |
| `bind` | `127.0.0.1` | Interface to listen on. Loopback by default; see the warning below. |

The bundled launch file fixes `recorder_node` at `/rosbag2_dynamic_recorder`, so driving a
differently named recorder means running `ui_node` yourself.

## What it is built from

`rclpy`, `ament_index_python` and the Python standard library. No web framework, no npm build step
and no CDN: the page is a single static file served from the package, so it loads on a robot with no
internet. Every dependency added there would be an install barrier in front of the people this is
meant to be usable by.

It is a separate node rather than an HTTP server inside the recorder, so the recorder's write path
does not share a process with a web server, and the UI can be restarted or omitted without touching
recording.

:::{warning}
The UI has no authentication of any kind. It binds to `127.0.0.1` by default for that reason.
`bind:=0.0.0.0` lets anyone who can reach the machine on port 8088 stop the recording or change what
is captured, and the node logs a warning when you do it. Only use it on a trusted network.

In Docker you must use `bind:=0.0.0.0`, because the default loopback bind is loopback inside the
container and Docker would have nothing to forward to.
:::
