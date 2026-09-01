# Everything a UI or CLI needs to render a status line.
#
# Published periodically and on every change on ~/status, and also returned by ~/get_status for
# one-shot clients. The topic is the one a UI should use: it needs no polling, it keeps multiple
# clients converged, and it is latched so a panel opened mid-recording sees the current state
# immediately.

# Bag directory: the path you would hand to `ros2 bag play`.
string uri

# Storage plugin in use, e.g. "mcap".
string storage_id

# True between startup and ~/stop.
bool recording

# True while arriving messages are discarded rather than written. Subscriptions stay up.
bool paused

# True if the writer is buffering in memory and only flushes on ~/snapshot.
bool snapshot_mode

# When recording started, on the node clock.
builtin_interfaces/Time recording_started

# Seconds since recording started. Provided so a client on a different machine does not have to
# reconcile clocks to show an elapsed time.
float64 elapsed_seconds

# Topics currently subscribed, sorted.
string[] subscribed_topics

# Messages written to the bag since startup, including the recorder's own event messages.
uint64 messages_written

# Messages the transport or the writer REPORTED as lost.
#
# WARNING: this is reported losses, not actual losses. Messages can go missing without either
# layer raising an event -- observed on a TurtleBot 4 run where this field read 0 while roughly
# 3-4% of messages were absent from the bag (see notes/recording-stall.md). Do not present this
# to a user as a guarantee that nothing was lost.
uint64 messages_lost

# Messages DETECTED as missing by watching for gaps in each publisher's sequence numbers.
#
# This is the honest number. Unlike messages_lost it does not depend on the transport raising an
# event, so it catches samples the publisher overwrote in its own history before we ever saw them
# -- which is how messages actually go missing here. Compare the two: a large messages_missed with
# messages_lost at 0 means the recorder was blocked long enough for publisher history to overflow.
#
# Only meaningful when sequence_numbers_available is true.
uint64 messages_missed

# True if the middleware populates publication sequence numbers, i.e. whether messages_missed
# means anything at all. If false, treat messages_missed as unknown rather than zero.
bool sequence_numbers_available

# Number of times the bag file has rolled over, whether by ~/split_bagfile or by a size or
# duration limit.
uint64 bag_splits

# Total size on disk of the bag directory in bytes. 0 if it could not be determined.
#
# NOTE: bytes *flushed to disk*, not bytes captured. The writer caches, so this reads 0 early in a
# recording even though messages are being captured normally. Use messages_written for liveness.
uint64 bag_size_bytes
