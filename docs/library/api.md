# API reference

Generated from the source. The [guide](index.md) is the place to start; this is the page to come
back to for a signature.

## The client

```{eval-rst}
.. autoclass:: dynrec.client.Recorder
   :members:
   :member-order: bysource

.. autofunction:: dynrec.client.discover

.. autodata:: dynrec.client.DEFAULT_TIMEOUT

.. autodata:: dynrec.client.DEFAULT_DISCOVERY_TIMEOUT
```

## Reading a bag back

The rate every other tool reports is averaged over the whole bag, which describes a channel that
stopped early as a slow sensor. These read the recorder's own events to measure each channel over
the time it was actually being recorded.

```{eval-rst}
.. automodule:: dynrec.bag
   :members:
   :member-order: bysource
```

## Results

What the calls hand back: plain data, not ROS messages. The rule these types exist to enforce is
that a number the recorder cannot vouch for is `None`, never a convenient zero.

```{eval-rst}
.. automodule:: dynrec.results
   :members:
   :member-order: bysource
```

## Errors

```{eval-rst}
.. automodule:: dynrec.errors
   :members:
   :show-inheritance:
   :member-order: bysource
```

## Discovery

How a recorder is identified on the graph. Takes a graph listing as an argument rather than
fetching one, which is what makes the rule testable without a ROS graph.

```{eval-rst}
.. automodule:: dynrec.discovery
   :members:
   :member-order: bysource
```

## Scheduling

```{eval-rst}
.. automodule:: dynrec.schedule
   :members:
   :member-order: bysource
```

## Topic specifications

```{eval-rst}
.. automodule:: dynrec.topics
   :members:
   :member-order: bysource
```
