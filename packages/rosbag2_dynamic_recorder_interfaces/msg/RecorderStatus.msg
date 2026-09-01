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

# Number of times the bag file has rolled over, whether by ~/split_bagfile or by a size or
# duration limit.
uint64 bag_splits

# Total size on disk of the bag directory in bytes. 0 if it could not be determined.
#
# NOTE: bytes *flushed to disk*, not bytes captured. The writer caches, so this reads 0 early in a
# recording even though messages are being captured normally. Use messages_written for liveness.
uint64 bag_size_bytes
