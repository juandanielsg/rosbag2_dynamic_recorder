#!/usr/bin/env bash
# M1 done-criterion check: topics can be added and removed from a live recording via service
# calls, into one continuous MCAP, with no gap on untouched topics.
#
# Uses spike_publishers from the throwaway spike package for traffic on /spike/{a,b,c}.
# Run inside the dev container with the workspace overlay sourced.
set -u

BAG=${1:-/tmp/m1_bag}
NODE=/rosbag2_dynamic_recorder
IFACE=rosbag2_dynamic_recorder_interfaces/srv
rm -rf "$BAG"

ros2 run rosbag2_dynamic_spike spike_publishers > /tmp/m1_pub.log 2>&1 &
PUB=$!
sleep 3

# Start recording /spike/a and /spike/b. /spike/a is never touched again: it is the control.
ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:="$BAG" -p "topics:=['/spike/a','/spike/b']" > /tmp/m1_rec.log 2>&1 &
REC=$!
sleep 6

echo "=== 1. subscribe_topics: add /spike/c (type discovered from graph) ==="
ros2 service call "$NODE/subscribe_topics" "$IFACE/SubscribeTopics" "{topics: ['/spike/c']}"
sleep 5

echo "=== 2. unsubscribe_topics: drop /spike/b ==="
ros2 service call "$NODE/unsubscribe_topics" "$IFACE/UnsubscribeTopics" "{topics: ['/spike/b']}"
sleep 5

echo "=== 3. set_topics: back to [a, b] -- drops c, re-adds b, leaves a untouched ==="
ros2 service call "$NODE/set_topics" "$IFACE/SetTopics" "{topics: ['/spike/a','/spike/b']}"
sleep 5

echo "=== 4. get_subscribed_topics ==="
ros2 service call "$NODE/get_subscribed_topics" "$IFACE/GetSubscribedTopics" "{}"

kill -INT "$REC" 2>/dev/null
wait "$REC" 2>/dev/null
kill "$PUB" 2>/dev/null
wait "$PUB" 2>/dev/null
sleep 1

echo "=== recorder log ==="
cat /tmp/m1_rec.log
echo "=== bag ==="
ls -la "$BAG"
