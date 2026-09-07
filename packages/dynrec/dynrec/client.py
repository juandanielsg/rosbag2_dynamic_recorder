# Copyright 2026 juandanielsg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The rclpy plumbing: one object that owns a context, a node and a thread to spin it.

Three decisions shape this file.

**It brings its own rclpy context and its own executor thread.** The obvious alternative -- take
the caller's node and spin it -- deadlocks in the case this library exists for. A supervisor node
that changes what is recorded when the robot changes mode does so from inside a callback, and
spinning a node that the caller's executor already owns, from within that executor's own thread,
waits forever. Owning the context sidesteps that completely: calls block the calling thread and
nothing else, whether or not the caller has ROS of its own. The cost is a second participant on
the graph per client, which is why `Recorder` is meant to be long-lived and is a context manager.

**One client per service, created once and kept.** `ros2 dynrec` creates and destroys a client per
invocation, which is right for a process that exits a moment later and wrong for a script that
runs for a week. It is also the suspect for the ceiling recorded in notes/roadmap.md, where a
single rclpy node stopped receiving service responses after roughly 7,000 calls.

**Failures raise; partial success does not.** A refused call becomes an exception because a script
that ignores one has to work at it. But a call that subscribed two topics of three is reported by
the recorder as success, so it returns normally and says so in `TopicChange.unavailable`.
"""

import os
import threading
import time
from itertools import count

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rosbag2_dynamic_recorder_interfaces.msg import (
    PauseEvent,
    RecorderStatus,
    SubscriptionChangeEvent,
)
from rosbag2_dynamic_recorder_interfaces.srv import (
    GetProfiles,
    GetStatus,
    GetSubscribedTopics,
    SetProfile,
    SetTopics,
    SubscribeTopics,
    UnsubscribeTopics,
)
from rosbag2_interfaces.srv import (
    IsPaused,
    Pause,
    Record,
    Resume,
    Snapshot,
    SplitBagfile,
    Stop,
    TogglePaused,
)

from dynrec.discovery import choose_recorder, recorder_nodes
from dynrec.errors import CallFailed, CallTimeout, ServiceUnavailable
from dynrec.results import Event, Profiles, Status, TopicChange
from dynrec.schedule import apply_schedule, parse_time
from dynrec.topics import parse_topic_specs, require_a_selection

#: Seconds to wait for a service to appear and then to reply. Generous because adding a topic
#: costs ~0.5s inside the recorder and a set_topics over a dozen topics is slower still.
DEFAULT_TIMEOUT = 10.0

#: Seconds to wait for a recorder to show up on the graph before giving up on finding one.
#: Longer than the CLI's one second of spin: a script is often started by the same launch file as
#: the recorder, so a couple of seconds of discovery lag is ordinary rather than exceptional.
DEFAULT_DISCOVERY_TIMEOUT = 5.0

_node_names = count()


def _unique_node_name():
    """A node name that will not collide with a second client in the same process.

    Two clients under one name is legal in ROS and confusing everywhere else -- notably in
    `ros2 node list`, where a script debugging its own recorder would see one name twice.
    """
    return 'dynrec_client_{}_{}'.format(os.getpid(), next(_node_names))


class _Ros:
    """A private context, node and spinning thread. Shut down exactly once."""

    def __init__(self, node_name=None, domain_id=None):
        self.context = rclpy.Context()
        if domain_id is None:
            self.context.init()
        else:
            self.context.init(domain_id=domain_id)
        self.node = Node(node_name or _unique_node_name(), context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._closed = False
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            # The context being shut down under a spinning executor is the normal way this thread
            # ends, and rclpy signals it by raising. Nothing here can act on any other failure
            # either -- close() is the only path out -- so the thread simply ends.
            pass

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()
        # Not joined without a timeout: an executor wedged in a callback would otherwise hang the
        # caller's exit, and the thread is a daemon precisely so it cannot outlive the process.
        self._thread.join(timeout=5.0)


class _Handle:
    """What a subscription registration hands back, so it can be undone."""

    def __init__(self, unregister):
        self._unregister = unregister
        self._live = True

    def cancel(self):
        if self._live:
            self._live = False
            self._unregister()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.cancel()
        return False


def discover(timeout=DEFAULT_DISCOVERY_TIMEOUT, domain_id=None):
    """Every recorder currently visible on the graph, sorted, as node names.

    For the fleet case: a script controlling two recorders needs their names before it can build
    a `Recorder` for each, and `Recorder()` deliberately refuses to guess between them.
    """
    ros = _Ros(domain_id=domain_id)
    try:
        return _poll_for_recorders(ros.node, timeout, None)[0]
    finally:
        ros.close()


def _poll_for_recorders(node, timeout, requested):
    """Wait for the graph to show a recorder. Returns (discovered, deadline_reached).

    Polls rather than sleeping a fixed spin time so a recorder that is already up costs ~50ms
    instead of a flat second -- which matters when a script builds a client per operation.
    """
    deadline = time.monotonic() + max(0.0, timeout)
    discovered = []
    while True:
        discovered = recorder_nodes(node.get_service_names_and_types())
        if discovered and (requested is None or _matches(discovered, requested)):
            return discovered, False
        if time.monotonic() >= deadline:
            return discovered, True
        time.sleep(0.05)


def _matches(discovered, requested):
    return ('/' + requested.strip('/')) in discovered


class Recorder:
    """A connected rosbag2_dynamic_recorder, and every operation it offers.

    With one recorder running there is nothing to configure::

        with Recorder() as rec:
            rec.set_topics(['/scan', '/odom'])
            print(rec.status().subscribed_topics)

    Name one explicitly when several are running, or when the script knows which it wants::

        rec = Recorder('/robot/left_recorder')

    Every method blocks until the recorder has answered and raises on refusal, so a script reads
    top to bottom and needs no return-code checking. Safe to call from any thread, including from
    inside a callback of the caller's own node -- see the module docstring for why that is the
    point rather than a detail.
    """

    def __init__(
        self,
        recorder=None,
        *,
        timeout=DEFAULT_TIMEOUT,
        discovery_timeout=DEFAULT_DISCOVERY_TIMEOUT,
        domain_id=None,
        node_name=None,
    ):
        """Connect to a recorder.

        :param recorder: node name, e.g. '/robot/left_recorder'. Omit to use the only recorder on
            the graph; a name is required as soon as there are two.
        :param timeout: seconds to wait for a service to appear and then to reply.
        :param discovery_timeout: seconds to wait for a recorder to appear on the graph. A named
            recorder is trusted even if discovery does not turn it up, since discovery can lag and
            timing out on the first call says more than refusing a name that is about to exist.
        :param domain_id: ROS domain to join. Defaults to the environment's ROS_DOMAIN_ID.
        :param node_name: name for this client's own node. Defaults to a unique one.
        """
        self.timeout = timeout
        self._ros = _Ros(node_name=node_name, domain_id=domain_id)
        try:
            discovered, _ = _poll_for_recorders(self._ros.node, discovery_timeout, recorder)
            #: Node names of every recorder seen while connecting, for error messages and for a
            #: caller that wants to know what else is out there.
            self.discovered = discovered
            #: The recorder this client talks to, fully qualified.
            self.name = choose_recorder(discovered, recorder)
        except Exception:
            # A failed connect must not leak a context and a live thread. The caller never got an
            # object to close.
            self._ros.close()
            raise

        self._clients = {}
        self._lock = threading.Lock()
        self._status_callbacks = []
        self._event_callbacks = []
        self._status_sub = None
        self._event_subs = []

    # -- lifecycle ------------------------------------------------------------------------

    def close(self):
        """Drop the client's node and stop its thread. Idempotent."""
        self._ros.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- reading state --------------------------------------------------------------------

    def status(self):
        """Everything the recorder knows about itself, right now, as a :class:`Status`.

        A one-shot call. A script that reacts to changes should use :meth:`on_status` instead,
        which needs no polling and delivers the recorder's own view as it changes.
        """
        response = self._call(GetStatus, 'get_status')
        return Status.from_msg(response.status, recorder=self.name)

    def topics(self):
        """The topics being recorded, sorted. Bare names, nothing else."""
        return list(self._call(GetSubscribedTopics, 'get_subscribed_topics').topics)

    def profiles(self):
        """The configured profiles and which one is active, as a :class:`Profiles`."""
        return Profiles.from_msg(self._call(GetProfiles, 'get_profiles'))

    def is_paused(self):
        """True while arriving messages are being discarded. Subscriptions stay up either way."""
        return bool(self._call(IsPaused, 'is_paused').paused)

    # -- changing what is recorded --------------------------------------------------------

    def add(self, topics=(), *, regex='', exclude_regex=''):
        """Start recording these topics, leaving everything else alone.

        Name a type as `'/scan:sensor_msgs/msg/LaserScan'` to record a topic whose publisher has
        not started yet; without one the type is read from the graph, which cannot work before the
        publisher exists. From a script that is the common case, since the script often starts the
        publisher itself.

        `regex` is a search over topic names on the graph, in the dialect of
        `ros2 bag record -e`: 'camera' finds '/robot/camera/image', so anchor it ('^/robot/') when
        a prefix is what you mean. `exclude_regex` is applied after names and matches are
        combined, so it filters explicitly named topics too.
        """
        request = SubscribeTopics.Request()
        request.topics, request.topic_types = parse_topic_specs(topics)
        require_a_selection(request.topics, regex)
        request.regex = regex
        request.exclude_regex = exclude_regex
        return TopicChange.from_msg(self._call(SubscribeTopics, 'subscribe_topics', request))

    def remove(self, topics=(), *, regex='', exclude_regex=''):
        """Stop recording these topics, leaving everything else alone.

        Messages already written are kept; the bag simply stops gaining new ones, leaving a sparse
        channel that the recorder's own events explain.

        `regex` here matches what is *currently being recorded*, not the graph -- only a recorded
        topic can be dropped, and matching the graph would silently do nothing for one that has
        since left it.
        """
        topic_names, _ = parse_topic_specs(topics)
        require_a_selection(topic_names, regex)
        request = UnsubscribeTopics.Request()
        request.topics = topic_names
        request.regex = regex
        request.exclude_regex = exclude_regex
        return TopicChange.from_msg(self._call(UnsubscribeTopics, 'unsubscribe_topics', request))

    def set_topics(self, topics=(), *, regex='', exclude_regex=''):
        """Record exactly these topics and nothing else.

        The one to reach for when switching modes: topics common to the old and the new set are
        never torn down and keep recording without a gap, which is the guarantee this whole
        project is built around.

        Passing nothing is a valid request that stops recording every topic without closing the
        bag -- unlike :meth:`add` and :meth:`remove`, which refuse an empty selection because for
        them it could only ever be a mistake.
        """
        request = SetTopics.Request()
        request.topics, request.topic_types = parse_topic_specs(topics)
        request.regex = regex
        request.exclude_regex = exclude_regex
        return TopicChange.from_msg(self._call(SetTopics, 'set_topics', request))

    def profile(self, name):
        """Apply a named profile: record exactly the topics it lists.

        Routed through the same code as :meth:`set_topics` inside the recorder, so it inherits the
        same guarantee about topics common to both sets.
        """
        request = SetProfile.Request()
        request.name = name
        return TopicChange.from_msg(self._call(SetProfile, 'set_profile', request))

    # -- controlling the recording --------------------------------------------------------

    def pause(self):
        """Stop writing messages. Subscriptions stay up and arriving messages are discarded.

        The recorder writes a PauseEvent into the bag, so the hole this leaves across every topic
        is explained from inside the bag rather than looking like the crash it otherwise resembles.
        """
        self._call(Pause, 'pause')

    def resume(self, at=None, *, mode='node', topic=''):
        """Start writing messages again, now or at a scheduled time.

        Returns the epoch time it was scheduled for, or None when it happened immediately.

        `mode` picks the clock the time is compared against. 'node' fires on a timer and works on a
        robot that has gone quiet; 'publish' and 'receive' are evaluated as messages arrive and
        therefore cannot fire without traffic -- which is a feature when you mean "when data
        resumes" and a trap when you mean "in five minutes".
        """
        request = Resume.Request()
        scheduled = apply_schedule(request, at, mode, topic, 'resume_time', 'resume_mode')
        self._call(Resume, 'resume', request)
        return scheduled

    def toggle(self):
        """Flip between paused and recording. Returns True if the recorder is now paused.

        Costs a second round trip to ask, because a toggle whose outcome you have to guess is not
        much use to a script.
        """
        self._call(TogglePaused, 'toggle_paused')
        return self.is_paused()

    def split(self, at=None, *, mode='node', topic=''):
        """Roll over to a new bag file, now or at a scheduled time.

        Returns the epoch time it was scheduled for, or None when it happened immediately. A
        schedule is cleared by :meth:`stop`, so one queued here cannot fire against a later bag.
        """
        request = SplitBagfile.Request()
        scheduled = apply_schedule(request, at, mode, topic, 'split_time', 'split_mode')
        self._call(SplitBagfile, 'split_bagfile', request)
        return scheduled

    def snapshot(self):
        """Flush the in-memory buffer to disk. Only meaningful in snapshot mode.

        Raises CallFailed otherwise, since the service reports a bare false: the near-certain cause
        is a recorder not started with `snapshot_mode:=true`, in which case there is no buffer to
        flush because messages are already being written as they arrive.
        """
        if not self._call(Snapshot, 'snapshot').success:
            raise CallFailed(
                'snapshot failed. The recorder logs the reason; the usual one is that it was not '
                'started with snapshot_mode:=true, so there is no buffer to flush.')

    def stop(self):
        """Close the bag. The node keeps running, and :meth:`record` opens a new one."""
        self._call(Stop, 'stop')

    def record(self, uri='', at=None):
        """Open a new bag after a stop, restoring the previous topic selection.

        Returns the epoch time it was scheduled for, or None when it started immediately. `uri`
        defaults to the one the recorder was started with. Unlike :meth:`resume` and :meth:`split`
        there is no mode to choose: Record carries no mode field, so a scheduled start is node time
        by definition and fires on a timer whether or not messages are arriving.
        """
        request = Record.Request()
        request.uri = uri
        if at is None:
            self._call(Record, 'record', request)
            return None
        seconds, nanoseconds = parse_time(at)
        request.start_time.sec = seconds
        request.start_time.nanosec = nanoseconds
        self._call(Record, 'record', request)
        return seconds + nanoseconds / 1e9

    # -- watching -------------------------------------------------------------------------

    def on_status(self, callback):
        """Call `callback(status)` whenever the recorder publishes its status.

        The reason to prefer this library over shelling out to `ros2 dynrec`: no polling, and the
        first call arrives immediately even mid-recording, because the recorder latches the topic.

        Callbacks run on this client's own executor thread. Keep them short, and do not call back
        into a blocking method of this same client from inside one -- that would be the deadlock
        the whole design avoids everywhere else. Raise anything and it is logged and swallowed, so
        one bad callback cannot stop the others or kill the thread.

        Returns a handle with `.cancel()`, usable as a context manager.
        """
        with self._lock:
            self._status_callbacks.append(callback)
            if self._status_sub is None:
                # Matching the recorder's latched profile is what makes a client started
                # mid-recording see current state at once instead of waiting for the next tick.
                self._status_sub = self._ros.node.create_subscription(
                    RecorderStatus, '{}/status'.format(self.name), self._deliver_status,
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST))
        return _Handle(lambda: self._drop(self._status_callbacks, callback))

    def on_event(self, callback):
        """Call `callback(event)` for every subscription change and every pause.

        Both streams, flattened into one :class:`Event`, because they are only meaningful together:
        one explains a channel that stops, the other a bag that stops. They arrive on separate
        subscriptions, so if order matters, sort by `event.stamp` rather than trusting the order
        they were delivered in.

        Same callback rules and same handle as :meth:`on_status`.
        """
        with self._lock:
            self._event_callbacks.append(callback)
            if not self._event_subs:
                events_qos = QoSProfile(
                    depth=50,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST)
                self._event_subs = [
                    self._ros.node.create_subscription(
                        SubscriptionChangeEvent,
                        '{}/events/subscription_change'.format(self.name),
                        lambda msg: self._deliver_event(Event.from_subscription_msg(msg)),
                        events_qos),
                    self._ros.node.create_subscription(
                        PauseEvent,
                        '{}/events/pause'.format(self.name),
                        lambda msg: self._deliver_event(Event.from_pause_msg(msg)),
                        events_qos),
                ]
        return _Handle(lambda: self._drop(self._event_callbacks, callback))

    def wait_for(self, predicate, timeout=None):
        """Block until `predicate(status)` is true, and return that :class:`Status`.

        Written against the latched status topic rather than a polling loop, so the current state
        is evaluated immediately: `rec.wait_for(lambda s: not s.recording)` returns at once if the
        recorder is already stopped, instead of hanging until something else changes.

        Raises CallTimeout if `timeout` passes first.
        """
        matched = []
        satisfied = threading.Event()

        def check(status):
            if not satisfied.is_set() and predicate(status):
                matched.append(status)
                satisfied.set()

        with self.on_status(check):
            if not satisfied.wait(timeout):
                raise CallTimeout(
                    'the recorder did not reach the requested state within {:g}s'.format(timeout))
        return matched[0]

    def _drop(self, callbacks, callback):
        with self._lock:
            if callback in callbacks:
                callbacks.remove(callback)

    def _deliver_status(self, msg):
        status = Status.from_msg(msg, recorder=self.name)
        with self._lock:
            callbacks = list(self._status_callbacks)
        self._fan_out(callbacks, status)

    def _deliver_event(self, event):
        with self._lock:
            callbacks = list(self._event_callbacks)
        self._fan_out(callbacks, event)

    def _fan_out(self, callbacks, payload):
        for callback in callbacks:
            try:
                callback(payload)
            except Exception as exc:
                # Logged rather than raised: this runs on the executor thread, where an escaping
                # exception would take down every other callback and the status stream with it.
                self._ros.node.get_logger().error(
                    'dynrec callback raised {}: {}'.format(type(exc).__name__, exc))

    # -- the one path to the recorder -----------------------------------------------------

    def _call(self, srv_type, verb, request=None):
        """Call `<recorder>/<verb>`, returning the response or raising.

        Clients are cached: the same service called a thousand times reuses one client, which is
        both faster and, on the evidence in notes/roadmap.md, the difference between a script that
        keeps working and one that quietly stops getting replies.
        """
        service = '{}/{}'.format(self.name, verb)
        client = self._client_for(srv_type, service)
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise ServiceUnavailable(self._unreachable_message(service))

        future = client.call_async(request if request is not None else srv_type.Request())
        answered = threading.Event()
        future.add_done_callback(lambda _: answered.set())
        if not answered.wait(self.timeout):
            # Dropped from the client's pending table, or a long-lived script leaks one entry per
            # timed-out call.
            if hasattr(client, 'remove_pending_request'):
                client.remove_pending_request(future)
            raise CallTimeout(
                '{} did not reply within {:g}s. The recorder is up but busy or wedged.'.format(
                    service, self.timeout))

        response = future.result()
        if response is None:
            raise CallTimeout('{} returned no response'.format(service))
        return self._checked(response, service)

    def _client_for(self, srv_type, service):
        with self._lock:
            client = self._clients.get(service)
            if client is None:
                client = self._ros.node.create_client(srv_type, service)
                self._clients[service] = client
            return client

    @staticmethod
    def _checked(response, service):
        """Turn the `return_code`/`error_string` pair most services share into an exception.

        Services with no return code -- Pause, TogglePaused, Snapshot -- pass through untouched;
        their callers handle what little there is to handle.
        """
        code = getattr(response, 'return_code', 0)
        if code != 0:
            detail = getattr(response, 'error_string', '') or 'return_code {}'.format(code)
            raise CallFailed(
                '{} refused the call: {}'.format(service, detail),
                return_code=code, error_string=getattr(response, 'error_string', ''))
        return response

    def _unreachable_message(self, service):
        """Explain a missing service, naming the recorders that *are* there.

        A typo in the node name is the common cause, so listing what was found turns a bare
        timeout into an answer.
        """
        message = '{} is not available after {:g}s.'.format(service, self.timeout)
        others = [name for name in self.discovered if name != self.name]
        if others:
            return message + ' Recorders that are running: ' + ', '.join(others)
        if not self.discovered:
            return message + ' No recorder was found on the graph at all.'
        return message

    def __repr__(self):
        return '<dynrec.Recorder {}>'.format(self.name)


__all__ = ['Recorder', 'discover', 'DEFAULT_TIMEOUT', 'DEFAULT_DISCOVERY_TIMEOUT']
