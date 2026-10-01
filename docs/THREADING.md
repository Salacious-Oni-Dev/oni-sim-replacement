# Threading

The simulation frame runs on a worker thread of its own, overlapped with the game's managed
code and rendering. This document describes how that works, which calls wait for the worker
and which do not, and why none of it changes what the game sees.

Implementation: `sim/simdll.cpp` (`FrameWorker`, `PublishedFrame`, the `PrepareGameData` case
of `HandleImmediateMessage`, and `SIM_HandleMessage`).

## The shape

The game's own SimDLL runs its frame on a separate thread and meets the game once per frame:
the game waits for the finished frame, reads it, and hands over; the sim then starts the next
frame while the game renders. The two sides alternate strictly. Neither ever reads a
half-written frame, and the sim never runs more than one frame ahead.

This library does the same thing:

1. The game calls `PrepareGameData` once per tick.
2. `PrepareGameData` returns the frame the worker finished during the previous tick.
3. It then rotates the message queues (below), releases the conduit handles freed during the
   tick, and kicks the worker to start the next frame.
4. The worker runs the frame and publishes it into the other of two alternating buffers.

So while the game reads frame N, the worker is filling frame N + 1. The overlap is with the
game's rendering and managed code, not with its call into the sim.

## Message latency is unchanged

A message the game sends during tick N is drained by the frame that is kicked at the end of
tick N, and the game sees its result in the frame returned at tick N + 1. That is one frame of
latency, which is what the game's own library has too: its frame also starts at the hand-over
and sees only the messages queued before it.

Two queues implement this. `SIM_HandleMessage` pushes deferred messages onto `queue`, on the
game thread. At `PrepareGameData`, after the previous frame has been collected and before the
next one is kicked, `queue` is swapped into `active`. The worker drains `active`. The swap has
to happen on the game thread, because the game is pushing onto `queue` at the time; only the
drain runs on the worker.

As a result, the threaded sim and the synchronous one publish the same bytes in the same
frame order, and the test suite is byte-identical either way.

Three inputs are the exception, and they move toward the game's behaviour rather than away
from it: the elapsed time, the active regions and the visibility mask are read at the
`PrepareGameData` that kicks the frame, one tick earlier than a synchronous sim would read
them. The game's own library carries the same one-frame delay on these inputs.

## The published frame

The game reads the arrays a `GameDataUpdate` points at for the whole of its tick, while the
worker is already filling the next frame. So the update cannot point at the sim's own
storage. `Project` and `FillPropertyTextures` are incremental and keep their buffers across
frames on purpose (see `PROJECTION.md`), which is exactly what makes those buffers unsafe to
hand out.

`PublishedFrame` is one byte arena per frame. Every array the update names is copied into it,
and each pointer in the update is patched to its copy once the whole frame has been written.
The patching has to wait until the end, because a later copy can grow the arena and move it.
Two of these alternate. The arena is never shrunk or cleared, so after the first frame a
publication is one pass of `memcpy` into a buffer that is already the right size.

That copy is the cost of threading: every published array, once per frame, paid on the
worker. The game's own library pays the same copy.

The published-property table and the event-stream table hang off the same `PublishedFrame`,
because `GameDataUpdate` is the game's struct and its layout cannot be extended. The two
exports that read them take the frame pointer the game is holding as a parameter; see below.

## Priming the pipeline

The first `PrepareGameData` after a `Start`, an `Alloc` or a `Load` has no frame in flight. It
runs its frame on the calling thread, returns it, and sets `pipelined`. Every call after that
returns what the worker finished. Without this, the first frame would be kicked before any
`NewGameFrame` had arrived, with no active regions at all, and the game would receive an empty
frame as if it were real. `pipelined` is cleared wherever a new world arrives.

## Which calls wait for the worker

`WaitIdle()` blocks until the frame in flight has been published. With no frame in flight it
is a single branch.

- **Deferred messages do not wait.** `QueueDeferredMessage` handles every message that is
  queued for the next frame, and it touches only `queue`, which belongs to the game thread.
  The game sends thousands of these per frame, and waiting on each would give back all of the
  overlap.
- **Every other message waits.** `HandleImmediateMessage` handles the rest, and
  `SIM_HandleMessage` calls `WaitIdle()` before it. Splitting the dispatch into these two
  functions keeps each message id in exactly one place.
- **Most added exports wait.** An export that reads or writes world state opens with
  `WaitIdle()`.

The exports that deliberately do not wait, and why:

| export | why it is safe without waiting |
|---|---|
| `SIM_ExtPublishedProperties`, `SIM_ExtPublishedEvents` | They take the frame the game is holding as a parameter. That frame is not the one the worker is filling, so it is intact. Reading "the latest frame" instead would race the worker. |
| `SIM_ExtEventStreamIndex` and the other stream lookups | The set of streams is declared once, in `SIM_Initialize`, before the worker exists, and never changes. |
| the element-attribute and molar-mass reads, `SIM_ComputeGasPressure` | Element data is written only through immediate messages, which have already waited for the worker, and is read-only while a frame runs. |
| the `ConduitTemperatureManager_*` exports | See the next section. |

Only add an export to this table if it reads nothing the worker writes during a frame.
Anything else needs `WaitIdle()`.

## Conduits run on the game thread

The game calls the `ConduitTemperatureManager_*` exports for every conduit on every tick, and
calls `_Set` from its own job threads. Those exports do not wait for the worker, because
waiting would stall every conduit call on the frame in flight. Instead, the worker never
touches conduit state at all.

The one piece of conduit work that belongs at the start of a frame is releasing the handles
the game freed during the previous tick. That runs on the game thread, in
`ReleaseConduitHandlesForNextFrame()`, immediately before the frame starts: before the
synchronous `RunFrame` of a primed call, and before each `Kick`. Nothing the game does can land
between that call and the frame starting, so a handle freed during tick N is released at the
start of frame N + 1, as in the game's own library.

This matters more than it might seem. `ConduitFlow.RebuildConnections` frees every handle of a
conduit type and allocates new ones, so the release queue is non-empty after every pipe that
is placed, broken or removed.

The CONVECTION conduit policy reads cell temperatures from the frame the game is holding
(`game_frame`), which is safe for the same reason as the published-property table.

## Turning it off

A file named `sim_nothread.on` in the **working directory** of the process disables the worker
and the pipeline together, and every frame then runs synchronously inside `PrepareGameData`.
It is an escape hatch for bisecting a divergence.

The marker is opened with a bare relative `fopen`, so it is looked for in the working
directory, not beside the DLL. A marker placed beside the DLL has no effect and prints
nothing. The tools in `driver/` run from `driver/`, so their marker goes there.

## Testing threading

Because both modes are built to publish the same bytes, an identical test suite does not prove
the worker ran: byte-identity is equally consistent with the thread never starting. To verify
threading, use a witness that is not the output, such as a log line from `WorkerMain` with the
thread id.

The suite also exercises neither of the two things that differ between the modes. Every
scenario sends a constant elapsed time, so the earlier read of the elapsed time is invisible
to it. No scenario uses conduits, so the game-thread conduit release is tested only in the
live game.

The worker is plain Win32 (`CreateThread` and two auto-reset events) rather than `<thread>`.
The toolchain is mingw-w64 with the win32 thread model, where the standard threading headers
are not dependable.
