# The command line

`ros2 dynrec` drives a running recorder without service-call syntax, and without a display — which
is the access pattern that matters on a robot.

```bash
ros2 dynrec status            # what is it doing
ros2 dynrec topics            # what is it recording, one per line
ros2 dynrec add /scan         # start recording /scan, leave everything else alone
ros2 dynrec remove /scan      # stop recording it
ros2 dynrec set /tf /odom     # record exactly these
ros2 dynrec profile navigation
ros2 dynrec pause | resume | toggle | split | snapshot | stop | record
```

## Finding the recorder

With one recorder on the graph there is nothing to configure. It is found by looking for a node
offering `~/get_status` **of our type**, so a recorder launched under any name or pushed into any
namespace is found, and a node that merely borrows the name is not.

Pass `--node` when several are running. With more than one it refuses to guess and lists them,
because guessing could stop the wrong recording.

## Exit codes

Every verb **exits non-zero when the recorder refuses**, so a supervisor script can rely on the
exit code alone and never has to parse `return_code` out of a service reply:

```bash
ros2 dynrec add /scan || echo "could not record /scan"
```

Failures go to stderr, where a supervisor's log will not mix them into the output.

## Patterns

```bash
ros2 dynrec add -e '^/camera/'                        # everything under /camera
ros2 dynrec set -e '^/robot/' --exclude-regex depth   # all of /robot but the depth camera
ros2 dynrec remove -e '/image'                        # drop the heavy ones
```

`-e/--regex` selects by pattern rather than by name and `--exclude-regex` filters the result. On
`add` and `set` the pattern searches the graph; on `remove` it searches what is actually being
recorded, since only a recorded topic can be dropped.

A pattern that matches nothing is refused on `add` — adding nothing is a request that was not met,
and the likely cause is a typo. On `set` it is allowed, because an empty result is the documented
way to record nothing without closing the bag.

## Naming a type

```bash
ros2 dynrec add /scan:sensor_msgs/msg/LaserScan
```

That is how you record a topic whose publisher has not started yet. Without it the type is
discovered from the graph, which needs the topic to be there already.

## Scheduling

`resume`, `split` and `record` take `--at`:

```bash
ros2 dynrec resume --at +30s
ros2 dynrec split  --at 14:05
ros2 dynrec record --at 2026-09-03T14:05
```

Accepted forms are `+30s`/`+5m`/`+1h`, `14:05` (today, or tomorrow if it has passed), an ISO 8601
instant, or raw epoch seconds.

`--mode` picks the clock for `resume` and `split`: `node` (default) fires on a timer and works on
a robot that has gone quiet; `publish` and `receive` are evaluated as messages arrive and cannot
fire without traffic. `--topic` restricts those to one topic, which must be one the recorder is
subscribed to.

:::{note}
Relative and wall-clock times are resolved against **your** clock, while the recorder compares
against its node clock. On one machine, or a clock-synced fleet, those agree; across a robot whose
clock has drifted they do not, which is why the absolute forms exist.
:::

## Reading a bag back

```bash
ros2 dynrec info /tmp/mybag
```

The one verb that does not talk to a running recorder. It reads a finished bag and reports what
each channel really did:

```
topic                                 msgs  recorded   active       rate    averaged
/demo/alpha                            590     29.5s    29.5s   20.00 Hz    16.12 Hz
/demo/beta                             218     11.0s    11.0s   19.87 Hz     5.95 Hz
/demo/delta                             80      4.0s     4.0s   20.25 Hz     2.19 Hz
/demo/gamma                            365     18.2s    18.2s   20.10 Hz     9.97 Hz
```

Every one of those topics published at 20 Hz. The `averaged` column is what `ros2 bag info` and
`mcap info` report — count over the whole bag — which describes a channel that stopped early as a
slow sensor. The `rate` column measures each channel over the time it was actually subscribed and
not paused, which the recorder's events make knowable.

It **exits 2** when the bag holds a hole that nothing explains: a gap shared by every live channel
with no pause event behind it, which is what a crash, a stall, or a recording made with
`record_pause_events:=false` all look like. A sparse channel on its own is normal and is not an
error. `--json` gives the same report as data, with unknowable values as `null`.

## Machine-readable status

```bash
ros2 dynrec status --json
```

Fields the recorder cannot vouch for come out as `null`, never as a convenient zero.
`messages_missed` is `null` when the middleware supplies no publication sequence numbers, which is
the difference between "nothing was missed" and "no idea".

```bash
ros2 dynrec topics | xargs -n1 echo recording:
```

`topics` prints bare names and nothing else, so it pipes into `xargs` without any parsing.
`status` is where the decorated version lives.

---

Built before the browser UI on purpose. A service API is only as good as the thinnest client that
can drive it, and proving it against something scriptable first is what keeps the API honest.

For anything longer than a shell one-liner, [the Python library](library/index.md) is the better
tool — it can watch the recorder's event streams, which no amount of shelling out will do.
