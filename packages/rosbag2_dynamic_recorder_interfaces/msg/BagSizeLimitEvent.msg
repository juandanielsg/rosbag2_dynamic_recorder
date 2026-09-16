# Emitted when the recorder stops itself because the bag grew past max_bag_size, and recorded into
# the bag itself by default.
#
# Why this exists: max_bagfile_size bounds one file and then rolls to the next, so a recording left
# running keeps growing across splits without limit. max_bag_size bounds the whole bag directory
# instead, as a hard cap on how much a single recording may ever take up -- a safety net for an
# unattended recorder, alongside the free-space guard that protects the disk as a whole.
#
# Like LowDiskEvent it is written into the bag BEFORE the writer is closed, so a reader can tell a
# deliberate size stop from a crash, a power loss or a killed process. The event is a few hundred
# bytes past a limit already exceeded, which is why the check compares against size on disk rather
# than refusing the write.

uint8 STOPPED=0

# When the stop took effect, on the node clock.
builtin_interfaces/Time stamp

# What was done. Only STOPPED today; kept as a field so a future "delete the oldest split and
# continue" action can be told apart from a full stop.
uint8 action

# Size on disk of the bag directory, across every split, at the check that triggered.
uint64 bag_size_bytes

# The configured limit in bytes that was exceeded.
uint64 max_bag_size

# What caused it, for auditing. e.g. "timer".
string reason

# The fully qualified node name of the event sender.
string node_name
