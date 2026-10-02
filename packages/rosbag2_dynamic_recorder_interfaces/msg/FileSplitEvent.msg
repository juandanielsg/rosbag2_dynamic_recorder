# Emitted whenever the bag rolls over to a new file, and recorded into the new file by default.
#
# Stock rosbag2 reports a rollover as rosbag2_interfaces/msg/WriteSplitEvent, which the recorder
# still publishes on ~/events/write_split for clients written against it. That message carries no
# stamp and no cause, so it cannot be placed on the recording's timeline or say why a file ended:
# this one does both, like the recorder's other events. Closing the bag at ~/stop is not a
# rollover and emits none.

# When the rollover happened, on the node clock.
builtin_interfaces/Time stamp

# The file that was closed, and the file recording continues in.
string closed_file
string opened_file

# What caused it: "service:split_bagfile", "schedule:split", or, for a rollover the writer made on
# its own, "limit:max_bagfile_size", "limit:max_bagfile_duration", or "limit" when both are set
# and either could have been reached.
string reason

# The fully qualified node name of the event sender.
string node_name
