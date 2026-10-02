# The browser UI

The way to use this without learning service-call syntax.

```bash
ros2 launch rosbag2_dynamic_recorder_ui recorder_with_ui.launch.py uri:=/tmp/mybag
```

Then open **http://localhost:8088**.

Tick topics to change what is being recorded. Profiles appear as buttons with the active one
highlighted; there is a status line, a feed of recent changes, and a recording timeline. Pause,
starting a new file, saving a snapshot and stopping are buttons too, and after a stop they give way
to **Start recording**, which opens a fresh bag and restores the previous selection. It takes an
optional bag path; left empty, the new bag goes next to the last one with a `(1)`, `(2)` suffix.

A topic that has lost messages in this bag carries a red tag with the count, and its tooltip breaks
it down into missed, lost in transport and dropped by the recorder. Missed is left out of the count,
not counted as zero, where the middleware cannot measure it.

The status line reports the recorder's own words. A recorder launched with `use_sim_time` reads
**Waiting for /clock** until the simulation publishes its clock, rather than appearing stopped,
because no bag is open yet; the timeline is then drawn on the simulation's clock, like the bag.

## Scheduling

The **Schedule** card queues an operation for later: a new file or a resume while recording, or a
start while stopped. Times are written as on the command line: `+30s`, `14:05`, an ISO instant, or
epoch seconds. A resume or a new file can fire **on a timer**, even on a robot that has gone quiet,
or **by publish time** or **by receive time**: on the first message, on one topic or any, stamped
at or after that time. Those two wait for traffic.

A relative time counts on the recorder's clock, read from its last status, not on the browser's or
the UI machine's: under `use_sim_time` that is the simulation's clock, so `+30s` means thirty
simulated seconds, and the pending list shows simulation times rather than clock times.

Below the form is everything the recorder has queued, whichever client queued it, with a countdown
for the timed ones. Only the newest schedule of each operation is live, so scheduling one again
replaces it; there is no cancel.

## The timeline

One bar per topic showing when it was being recorded, with pauses drawn as a band across all of
them and each rollover to a new file as a line across every row. It is built entirely from the
`SubscriptionChangeEvent`, `PauseEvent` and `FileSplitEvent` messages the recorder publishes. With
stock rosbag2, every one of those topic boundaries would have been a separate file.

Two details are deliberate:

- **The axis is drawn on the recorder's clock.** Event stamps are the recorder's, and mixing in the
  browser's would put every bar in the wrong place, which is not hypothetical on a host measured
  stepping ~120 ms backwards.
- **Anything before the page connected is hatched and labelled, not extended.** The recorder keeps
  only ten events for a late joiner, so earlier history genuinely was not observed, and the chart
  says so instead of guessing. Same rule as reporting unknown rather than zero.

## Filtering topics

The topic picker has a filter, with a regex mode that hands the pattern to the recorder as a single
call rather than ticking boxes one at a time. Three test topics need no filter; a robot with a
hundred does. Regex mode adds an **Except** pattern, applied after the first as the recorder applies
`exclude_regex`, and two ways to send the result: **Add** keeps what is already recorded
(`subscribe_topics`), **Record only** makes the match the whole selection (`set_topics`).

## Endpoints

The page is served by a small node that also exposes the two JSON endpoints it uses: `GET
/api/state` returns everything the page renders in one response, and `POST /api/action` takes
`{"action": "...", "topics": [...], "regex": "...", "exclude_regex": "...", "name": "...", "at":
"...", "mode": "...", "topic": "...", "uri": "..."}` for the same operations the buttons perform. They are not a versioned API, but they are enough to drive the
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

`rclpy`, `ament_index_python`, the project's own `dynrec` library and the Python standard library. No web framework, no npm build step
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
