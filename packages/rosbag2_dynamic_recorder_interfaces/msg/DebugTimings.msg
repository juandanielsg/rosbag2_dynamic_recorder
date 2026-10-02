# Where the recorder's own time goes, published on ~/debug/timings when the debug_timings
# parameter is set (it is off by default, and then nothing is measured or published). Written for
# rosbag2_dynamic_recorder_bench; a diagnostic, not a stable interface.
#
# Two kinds of message. One per finished operation (a service call, or an action a schedule
# fired), with the time spent in each of its phases. And one per period with operation
# "periodic", carrying the contention on the bag's write lock since the previous one: how often a
# message had to wait for it, and how long.
#
# All times are CLOCK_MONOTONIC nanoseconds, so a benchmark can line them up with its own.

# The service name ("subscribe_topics", "stop", ...), "schedule:<action>", or "periodic".
string operation

# When the operation started. For "periodic", the start of the period covered.
int64 start_mono_ns

# The operation's phases in the order they ran, each with its duration. A phase repeats when the
# operation handles several topics. A name ending in ":lock_wait" is time spent waiting for the
# bag's lock, which recorded messages also wait on.
string[] phase_names
int64[] phase_ns

# "periodic" only: messages that found the write lock held, their total wait, the longest.
uint64 lock_waits
int64 lock_wait_ns
int64 lock_wait_max_ns

# "periodic" only: messages staged because a slow operation (creating a channel, writing an
# event, splitting, closing) held the writer, rather than made to wait for it; and those dropped
# because the stage, as large as the writer's cache, was full.
uint64 staged
uint64 stage_dropped
