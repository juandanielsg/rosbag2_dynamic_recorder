# Design notes

Working notes, included verbatim from `notes/` in the repository rather than copied — so there is
one source for them and it is the one a contributor edits.

They are working notes, not user documentation: they argue with themselves, record measurements,
and in one case retract a conclusion. That is the point of keeping them. Read
[architecture.md](architecture.md) before changing anything in the recorder — it records *why*
this is built on `rosbag2_cpp::Writer` rather than by wrapping `Recorder`, which is the decision
everything else follows from.

```{toctree}
:maxdepth: 1

architecture
installation
spike-plan
roadmap
recording-stall
downstream-tools
connecting-a-gui
gui-plan
```

## What each one is for

[architecture.md](architecture.md)
: Why `Writer` and not `Recorder`, and what that costs. The foundational decision.

[installation.md](installation.md)
: Why this builds against apt rather than a rosbag2 source checkout — a forty-minute build of 22
  packages avoided in exchange for defining one `.srv` file ourselves.

[spike-plan.md](spike-plan.md)
: The throwaway validation of the core premise, with measured timings — adding a topic costs
  ~0.4–0.6 s, almost all of it message-definition resolution — and one retracted conclusion worth
  not rediscovering.

[roadmap.md](roadmap.md)
: What is done, what is next, and the reasoning behind each milestone. The longest and the most
  useful; it is also where the stress-test results and the client-side call ceiling are recorded.

[recording-stall.md](recording-stall.md)
: An investigation into a periodic recording stall, closed as environmental rather than ours. A
  worked example of the project's method: measure stock and measure ours, then compare.

[downstream-tools.md](downstream-tools.md)
: Whether `ros2 bag play`, `mcap`, `info` and `convert` cope with the sparse channels this
  recorder produces. They do — but `mcap info` averages a channel's rate over the whole bag, which
  misleads in precisely the way this project exists to prevent.

[connecting-a-gui.md](connecting-a-gui.md), [gui-plan.md](gui-plan.md)
: How a GUI reaches the recorder, and what was considered before the browser UI was built.
