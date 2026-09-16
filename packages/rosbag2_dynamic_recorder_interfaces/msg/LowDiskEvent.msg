# Emitted when the recorder stops itself because free space on the bag's filesystem fell below the
# configured minimum, and recorded into the bag itself by default.
#
# Why this exists: the existing limits (max_bagfile_size, max_bagfile_duration) bound the bag, not
# the disk. Anything else growing on the same filesystem -- system logs, core dumps, a second
# recorder, a software update -- can still fill it. A full disk on an embedded robot does more than
# stop the recording: logs cannot be written, DDS shared-memory files cannot be created, and
# services start to fail. min_free_space / min_free_space_percent stop before that.
#
# Without this event the bag would simply end at the last message, and a reader could not tell a
# deliberate low-disk stop from a crash, a power loss or a killed process -- the same shape of
# unexplained end the pause and subscription events exist to account for. It is written into the
# bag BEFORE the writer is closed, so the explanation is in the recording it explains. The minimum
# is above zero by construction, so there is room for this one small event even on a nearly full
# disk.

uint8 STOPPED=0

# When the stop took effect, on the node clock.
builtin_interfaces/Time stamp

# What was done. Only STOPPED today; kept as a field so a future "delete old splits and continue"
# action can be told apart from a full stop.
uint8 action

# Bytes available to this process on the bag's filesystem at the check that triggered.
uint64 free_space_bytes

# Total size of that filesystem in bytes, so a percentage can be read back.
uint64 total_space_bytes

# The configured minimum in bytes that was violated. 0 when only a percentage was set.
uint64 min_free_space

# The configured minimum as a percentage of the filesystem that was violated. 0.0 when only bytes
# were set.
float64 min_free_space_percent

# What caused it, for auditing. e.g. "timer".
string reason

# The fully qualified node name of the event sender.
string node_name
