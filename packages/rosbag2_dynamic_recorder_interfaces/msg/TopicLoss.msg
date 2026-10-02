# What one topic has lost in the current bag, as reported in RecorderStatus.topic_losses.
#
# The three figures are RecorderStatus's totals broken down by topic, with the same meanings and the
# same caveats. They are counted over the bag rather than over a subscription: dropping a topic and
# adding it back keeps its figures, and ~/record starts every topic at zero.

string topic_name

# Detected from gaps in publisher sequence numbers. Only meaningful when
# RecorderStatus.sequence_numbers_available is true; 0 otherwise, which is not the same as none.
uint64 messages_missed

# Reported by the transport as dropped before delivery. A floor, not a total.
uint64 messages_lost_in_transport

# Delivered, then dropped by the recorder: its write cache was full, or a write failed.
uint64 messages_lost_in_recorder
