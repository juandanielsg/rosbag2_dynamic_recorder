# rosbag2_dynamic_recorder

A ROS 2 bag recorder that can **change what it is recording without stopping the recording**, and
that writes down what it changed so the resulting bag explains itself.

Stock `ros2 bag record` fixes its topic set at launch. Recording a different set means stopping and
starting again, which ends the bag and begins another; a long run therefore becomes a pile of
files that have to be stitched back together, and every seam is a gap in every topic at once.

This records into one continuous bag while topics come and go, and it emits an event on every
change — into the bag itself — so that a channel which stops mid-recording can be told apart from
a channel that was lost.

```{toctree}
:maxdepth: 2
:caption: Using it

install
services
cli
library/index
ui
```

```{toctree}
:maxdepth: 2
:caption: Reference

library/api
```

## Where to start

- **Just want it running?** [Install](install.md), then the [browser UI](ui.md).
- **Driving it from a shell or a supervisor script?** [The command line](cli.md).
- **Driving it from Python?** [The library guide](library/index.md) and its
  [API reference](library/api.md).

## The four ways in

| | For | Needs |
|---|---|---|
| **Services** | Anything that speaks ROS | Nothing beyond ROS |
| **`ros2 dynrec`** | A shell, an ssh session, a supervisor script | The CLI package |
| **`dynrec` (Python)** | A mission script that reacts to what the robot is doing | The library package |
| **Browser UI** | Clicking checkboxes, no syntax to learn | The UI package, a browser |

They are layered deliberately, thinnest first: the services came before the CLI, the CLI before the
UI. A service API is only as good as the thinnest client that can drive it, and proving it against
something scriptable is what keeps it honest.

## One caveat, carried everywhere

Numbers the recorder cannot vouch for are reported as **unknown, never as zero**. `messages_missed`
is meaningful only when the middleware supplies publication sequence numbers; `messages_lost`
counts only what the transport chose to report, and has been observed reading 0 while roughly 3–4%
of messages were genuinely absent from a bag. Every interface here — the services, `--json`, the
library's `Status`, the browser UI — preserves that distinction rather than flattening it to a
convenient zero.
