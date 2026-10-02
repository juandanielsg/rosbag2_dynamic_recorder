# One operation the recorder has been asked to perform later, as reported in
# RecorderStatus.schedules.
#
# Only the newest schedule of each action is live: scheduling a resume replaces any resume already
# pending, whichever clock either was set on. A schedule leaves the list when it fires, and ~/stop
# clears pending resumes and splits, which must not fire against the next bag. A scheduled record
# survives a stop, since running while stopped is what it is for. There is no service to cancel
# one; scheduling the same action again replaces it.

uint8 RESUME=0
uint8 SPLIT=1
uint8 RECORD=2

# The clock `time` is compared against, numbered as in the Resume and SplitBagfile modes.
# NODE_TIME fires on a timer, so it fires on a robot that has gone quiet. PUBLISH_TIME and
# RECEIVE_TIME are checked as messages arrive and cannot fire without traffic. A RECORD is always
# NODE_TIME: Record has no mode field.
uint8 NODE_TIME=0
uint8 PUBLISH_TIME=1
uint8 RECEIVE_TIME=2

# RESUME, SPLIT or RECORD.
uint8 action

# When it fires, on the clock `mode` names.
builtin_interfaces/Time time

# NODE_TIME, PUBLISH_TIME or RECEIVE_TIME.
uint8 mode

# For PUBLISH_TIME and RECEIVE_TIME: the topic whose messages are checked. Empty means any recorded
# topic. Always empty for NODE_TIME.
string tracking_topic
