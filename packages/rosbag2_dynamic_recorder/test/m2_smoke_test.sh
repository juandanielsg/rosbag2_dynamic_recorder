#!/usr/bin/env bash
# M2 check: pause/resume, split, snapshot-mode guard, subscription-change events recorded into
# the bag, lost-message accounting, and the stop service.
#
# Run inside the dev container with the workspace overlay sourced.
set -u

BAG=${1:-/tmp/m2_bag}
NODE=/rosbag2_dynamic_recorder
IFACE=rosbag2_dynamic_recorder_interfaces/srv
RB=rosbag2_interfaces/srv
rm -rf "$BAG"

call() { ros2 service call "$NODE/$1" "$2" "${3:-{\}}" 2>&1 | tail -2; }

ros2 run rosbag2_dynamic_spike spike_publishers > /tmp/m2_pub.log 2>&1 &
PUB=$!
sleep 3

ros2 run rosbag2_dynamic_recorder dynamic_recorder --ros-args \
  -p uri:="$BAG" -p "topics:=['/spike/a','/spike/b']" > /tmp/m2_rec.log 2>&1 &
REC=$!
sleep 6

echo "=== 1. add /spike/c (expect a SUBSCRIBED event in the bag) ==="
call subscribe_topics "$IFACE/SubscribeTopics" "{topics: ['/spike/c']}"
sleep 3

echo "=== 2. pause -> is_paused -> resume (expect a gap on ALL topics) ==="
call pause "$RB/Pause"
call is_paused "$RB/IsPaused"
sleep 4
call resume "$RB/Resume"
call is_paused "$RB/IsPaused"
sleep 3

echo "=== 3. toggle_paused twice (should end unpaused) ==="
call toggle_paused "$RB/TogglePaused"
call toggle_paused "$RB/TogglePaused"
call is_paused "$RB/IsPaused"
sleep 2

echo "=== 4. split_bagfile (expect a second .mcap file) ==="
call split_bagfile "$RB/SplitBagfile"
sleep 3

echo "=== 5. snapshot without snapshot_mode (expect success=false, not a crash) ==="
call snapshot "$RB/Snapshot"

echo "=== 6. scheduled split is rejected explicitly, not silently ignored ==="
call split_bagfile "$RB/SplitBagfile" "{split_time: {sec: 9999999999, nanosec: 0}}"

echo "=== 7. drop /spike/b (expect an UNSUBSCRIBED event) ==="
call unsubscribe_topics "$IFACE/UnsubscribeTopics" "{topics: ['/spike/b']}"
sleep 3

echo "=== 8. stop service closes the bag ==="
call stop "$RB/Stop"
sleep 2
call stop "$RB/Stop"

kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null
kill "$PUB" 2>/dev/null; wait "$PUB" 2>/dev/null
sleep 1

echo "=== recorder log ==="
cat /tmp/m2_rec.log
echo "=== bag files ==="
ls -la "$BAG"
