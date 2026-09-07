# The Python library

`ros2 dynrec` covers the shell. `dynrec` covers the supervisor — the mission script that changes
what is recorded because the robot changed what it is doing.

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

The full signatures are in the [API reference](api.md). This page is about the four things you
have to understand to use it correctly.

## 1. Failures raise; partial successes do not

A refused call becomes an exception, which is the library's version of the CLI's exit code — a
script reads top to bottom with no return codes to check.

```python
from dynrec import CallFailed

try:
    rec.profile('navigation')
except CallFailed as exc:
    print(exc.error_string)      # the recorder's own words, not ours
```

But a call that subscribed two topics out of three is **success** to the service, so it returns
normally. The result object is the only thing that says otherwise:

```python
change = rec.set_topics(['/scan', '/not_yet_published'])
if not change.complete:
    print('not recording:', change.unavailable)
```

Forgetting that check is how you end up recording less than you think, which is why
`TopicChange.complete` exists rather than leaving you to compare two lists.

## 2. Unknown is not zero

```python
status = rec.status()
if status.messages_missed is None:
    print('this middleware supplies no sequence numbers; missed messages are unknowable')
elif status.messages_missed > 0:
    print('the recorder was blocked long enough to lose data')
```

`Status.messages_missed` is `None` — never `0` — when the middleware supplies no publication
sequence numbers. This is the third place that rule is enforced, after the browser UI and the
CLI's `--json`, and the most consequential of the three: a CLI number is read by a person who
might notice, a library number is compared against a threshold by code that will not.

`messages_lost_reported` is renamed on the way out for the same reason. It counts only what the
transport admitted to, and has read 0 against a bag that was genuinely missing 3–4% of its
messages.

## 3. It can watch, which is the actual reason to prefer it

Shelling out to the CLI can do everything above. What it cannot do is react.

```python
rec.on_event(lambda e: print(e.stamp, e.kind, e.action, e.topic or 'all topics'))
rec.on_status(lambda s: print(s.messages_written))

rec.wait_for(lambda s: not s.recording, timeout=60)
```

Both event streams arrive flattened into one {py:class}`~dynrec.results.Event`, because a channel
that stops and a bag that stops are the same question asked twice. They come in on separate
subscriptions, so **sort by `event.stamp`** — the recorder's clock — rather than trusting the
order they were delivered in.

`wait_for` is written against the latched status topic rather than a polling loop, so the current
state counts: `rec.wait_for(lambda s: not s.recording)` returns at once if the recorder is already
stopped, instead of hanging until something else changes.

Callbacks run on the client's own executor thread. Keep them short, and **do not call a blocking
method of the same client from inside one** — that is the one deadlock the design does not save
you from. A callback that raises is logged and swallowed, so one bad callback cannot take the
stream down with it.

## 4. A `Recorder` is meant to be long-lived

It holds one client per service rather than building one per call, which is both faster and — on
the evidence in [the roadmap](../design/roadmap.md) — the difference between a script that still
works after a few thousand calls and one that quietly stops getting replies. Close it with
`close()`, or use the `with` block.

### Why it brings its own ROS context

The client creates its own rclpy context, node and executor thread rather than taking yours.

That is not tidiness. The obvious alternative — `Recorder(node=my_node)`, spinning the caller's
node — deadlocks in exactly the case this library exists for: a supervisor node reacting from
inside its own callback would be asking an executor to spin a node that executor already owns.
Owning the context means calls block the calling thread and nothing else, whether or not the
caller has ROS of its own:

```python
class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.rec = Recorder()                       # safe: separate context
        self.create_subscription(Mode, '/mode', self._on_mode, 10)

    def _on_mode(self, msg):
        self.rec.profile(msg.name)                  # safe from inside a callback
```

`Recorder(node=...)` is deliberately not offered rather than offered with a warning. The cost is
one extra participant on the graph per client, which is the reason for the long-lived advice
above.

## Several recorders

`Recorder()` refuses to guess between two, as the CLI does and for the same reason — guessing
could stop the wrong recording. Ask who is out there first:

```python
from dynrec import Recorder, discover

for name in discover():
    with Recorder(name) as rec:
        print(name, rec.status().messages_written)
```

`AmbiguousRecorder` carries the names it found, so you can also let it fail and recover:

```python
from dynrec import AmbiguousRecorder

try:
    rec = Recorder()
except AmbiguousRecorder as exc:
    rec = Recorder(exc.recorders[0])
```

## Errors

Everything raised derives from `DynrecError`, so a script that only wants to log and carry on
catches one thing. The distinctions exist because they call for different responses — a recorder
that is not there yet is worth retrying, a call the recorder refused is not.

`RecorderNotFound`
: No recorder on the graph, or none under the requested name.

`AmbiguousRecorder`
: Several are running and none was named. Carries `.recorders`.

`ServiceUnavailable`
: The recorder is known but the service did not appear in time. Usually a typo in the node name;
  the message lists the recorders that *are* running.

`CallTimeout`
: Reached but did not reply in time — a busy or wedged recorder, rather than a name or domain
  mistake.

`CallFailed`
: The recorder refused. Carries `.return_code` and `.error_string`.

`InvalidRequest`
: The request could not be built from what you passed. Also a `ValueError`, since every instance
  is a mistake in the calling code.

## Without ROS installed

Four of the five modules import no ROS at all, and `import dynrec` resolves `Recorder` lazily, so
the pure parts are importable on a machine with no rclpy:

```python
from dynrec.schedule import parse_time     # works anywhere
from dynrec.results import Status          # works anywhere
from dynrec import Recorder                # needs rclpy and the interfaces
```

That is what lets 36 of the package's 58 tests run without a ROS graph, and what lets these docs
build without installing ROS.
