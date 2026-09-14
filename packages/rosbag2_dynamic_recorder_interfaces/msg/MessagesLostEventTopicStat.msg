# Field-identical to rosbag2_interfaces/msg/MessagesLostEventTopicStat, deliberately.
#
# Upstream added it, with MessagesLostEvent and the writer-side messages_lost_callback that feeds
# it, on rolling only: rosbag2_interfaces 0.26 (Jazzy) and 0.32 (Kilted) do not carry it. The
# recorder publishes ~/events/messages_lost on every distro, so where the upstream type exists it
# is used, and where it does not this copy stands in. See GetSubscribedTopics.srv for the same
# arrangement and its reasoning.

# The name of the topic on which the messages were lost
string topic_name

# The number of messages lost since the last event on a DDS transport layer
uint64 messages_lost_in_transport

# The number of messages lost since the last event in the Rosbag2 recorder. Always 0 on a rosbag2
# whose writer does not report its own losses (Jazzy, Kilted).
uint64 messages_lost_in_recorder
