# Emitted whenever recording is paused or resumed, and recorded into the bag itself by default.
#
# Rationale, and the reason this is separate from SubscriptionChangeEvent: pausing leaves a hole in
# EVERY topic at once. A bag full of simultaneous gaps is indistinguishable from a crash, a network
# fault, or the environmental rosbag2 stall that drops ~1.2s from every topic roughly every 31.2s
# -- all of which show up as a gap across every topic at the same moment. SubscriptionChangeEvent
# explains a channel that stops; this explains a bag that stops.
#
# Without it the one case the provenance mechanism was built for -- an unexplained gap -- was the
# one case it did not cover, because stock rosbag2_interfaces/srv/Pause has an empty request and
# response and so has nothing to carry an explanation.
#
# Note the deliberate asymmetry with pausing itself: while paused NO messages are written, but
# these events are. A pause that is absent from the bag cannot be told apart from the fault it
# resembles, which is the whole problem.

uint8 PAUSED=0
uint8 RESUMED=1

# When the change took effect, on the node clock.
#
# CAVEAT for anyone reading these back: bag readers return messages in timestamp order, so the
# order you see is only as monotonic as the clock was. A backward clock step -- NTP correcting, or
# a virtualised host, where ~120ms steps were measured -- reshuffles the events that fall within
# it, and a PAUSED can then appear after its RESUMED. The recorder's own transition sequence is
# always correct and no event is ever lost; only the recorded order follows the clock. If a
# sequence fails to alternate, suspect the clock before suspecting the recorder.
builtin_interfaces/Time stamp

# PAUSED or RESUMED.
uint8 action

# What caused the change, for auditing. e.g. "service:pause", "service:resume",
# "service:toggle_paused", "schedule:resume", "startup".
#
# "startup" covers both a recorder launched with start_paused and a bag reopened by ~/record while
# still paused: in either case the bag begins with a hole that would otherwise be unexplained.
string reason

# The fully qualified node name of the event sender.
string node_name
