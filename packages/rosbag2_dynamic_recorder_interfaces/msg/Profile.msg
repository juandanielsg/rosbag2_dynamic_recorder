# A named set of topics that can be applied in one call.
#
# The fleet case this exists for: a supervisor switches the robot between operating modes and the
# recorded topic set should follow, without interrupting the topics common to both modes.

# Identifier used with ~/set_profile.
string name

# Fully qualified topic names this profile records.
string[] topics
