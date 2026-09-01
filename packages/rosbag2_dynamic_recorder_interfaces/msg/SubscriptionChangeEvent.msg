# Emitted whenever the recorder starts or stops recording a topic, and recorded into the bag
# itself by default.
#
# Rationale: a channel that simply stops mid-bag is indistinguishable from a dropout, a crash, or
# a network fault. Without this, a deliberately unsubscribed topic looks exactly like lost data to
# anyone analysing the bag later. These events make the channel self-documenting: they say what
# changed, when, and why, in-stream and timestamped, and they survive bag splitting.

uint8 SUBSCRIBED=0
uint8 UNSUBSCRIBED=1

# When the change took effect.
builtin_interfaces/Time stamp

# Fully qualified topic name.
string topic_name

# Topic type. May be empty for an UNSUBSCRIBED event if the type was never resolved.
string topic_type

# SUBSCRIBED or UNSUBSCRIBED.
uint8 action

# What caused the change, for auditing. e.g. "service:subscribe_topics", "service:set_topics",
# "startup", "shutdown".
string reason

# The fully qualified node name of the event sender.
string node_name
