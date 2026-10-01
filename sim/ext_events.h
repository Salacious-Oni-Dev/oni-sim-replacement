// The per-frame event-stream registry.
//
// `GameDataUpdate` carries ten event lists as count+pointer pairs, filled by kernels and
// cleared once a frame. That mechanism is Klei's, it is already correct, and its one limit is
// that it cannot grow: an eleventh event type would need a field in a 496-byte struct that is
// a layout contract with the game. This registry is the same mechanism with the list of ten
// unhardcoded -- a name, a record stride, and a published count+pointer pair.
//
// DECLARATION IS NATIVE-ONLY, and `abi/sim_abi_ext.h`'s block at `kSubscribeEventStream` is
// where the reasoning lives. Short version: an event is something the simulation OBSERVED, so
// it is declared by whatever produces it, and every producer is a kernel in this DLL. A mod
// discovers, subscribes and reads. There is no registration message.
//
// SUBSCRIPTION IS THE COLLECTION GATE, not a publish filter. `Emit` on an unsubscribed stream
// does not allocate, does not copy and does not grow a buffer -- it is a load and a branch.
// That is deliberately stronger than stage 3's opt-in publish, where the data exists whether
// or not anyone subscribed and only the per-frame copy is optional. It is what makes it
// defensible to put an `Emit` on a path that runs often, which is the whole point of a
// mechanism meant to absorb every future event type.
//
// THE PER-FRAME CAP IS NOT A NICETY. A stream is fed by kernels, and a runaway producer (a mod
// that spams a malformed message every frame, a kernel with a bug) would otherwise grow a
// vector without bound inside the frame that has to publish it. Past `kExtStreamBytesPerFrame`
// the stream counts drops instead of growing, and `dropped` is published beside `count` so a
// truncated list can never be mistaken for a complete one.
//
// CLEARED AFTER THE PUBLISH, NOT AT THE TOP OF THE FRAME -- the one place this deliberately
// differs from Klei's ten. See `ClearFrame`.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "../abi/sim_abi_ext.h"
#include "ext_registry.h"

namespace oni_sim::ext {

// Streams and cell properties share one name rule -- `<owner>.<property>`, lowercase, one dot
// -- and share the reserved `sim.`/`oni.` owners. Not an accident and not worth a second
// validator: a mod reading a blob or a descriptor table should see one namespace, not two with
// slightly different spelling rules. `ValidPropertyName` in `ext_registry.h` is that rule.
//
// Unlike a cell property, a bad stream name here is a bug in THIS DLL rather than a mod's
// mistake, because only native code declares one. It is still checked, and still refused, so
// the mistake surfaces at the declaration rather than as a name a mod cannot resolve.
inline bool ValidStreamName(const std::string& name, std::string* why) {
  return ValidPropertyName(name, why);
}

class EventStreamRegistry {
 public:
  struct Stream {
    std::string name;
    // Bytes per record, fixed at declaration. Every `Emit` must hand over exactly this many.
    int32_t stride = 0;
    // The collection gate. Survives an `Allocate` and a load for the same reason a stage 3
    // publish subscription does: it is a statement about what the caller wants to watch, not
    // about one allocation of the world.
    bool subscribed = false;
    // This frame's records, and the records this frame refused to hold. Both reset by
    // `ClearFrame`; `dropped` is published alongside `count` rather than accumulated, because
    // "the list you are holding is short" is a fact about one frame.
    std::vector<uint8_t> bytes;
    int32_t dropped = 0;
  };

  // Declares a stream. Native callers only -- there is no message that reaches this. Returns
  // the index (>= 0), or -1 with `error` filled. The index is stable for the life of the
  // registry: nothing removes or reorders a stream.
  int32_t Register(const std::string& name, int32_t stride, std::string* error) {
    auto refuse = [&](const std::string& msg) {
      if (error) *error = msg;
      return -1;
    };
    std::string why;
    if (!ValidStreamName(name, &why)) {
      return refuse("event stream \"" + name + "\": " + why);
    }
    if (stride <= 0 || stride > 4096) {
      return refuse("event stream \"" + name + "\": stride must be 1..4096, not " +
                    std::to_string(stride));
    }
    if (Find(name) >= 0) {
      return refuse("event stream \"" + name + "\" is already declared");
    }
    if (static_cast<int32_t>(streams_.size()) >= kExtMaxEventStreams) {
      return refuse("event stream \"" + name + "\": registry is full (" +
                    std::to_string(kExtMaxEventStreams) + " streams)");
    }
    Stream s;
    s.name = name;
    s.stride = stride;
    streams_.push_back(s);
    return static_cast<int32_t>(streams_.size()) - 1;
  }

  int32_t Find(const std::string& name) const {
    for (size_t i = 0; i < streams_.size(); ++i) {
      if (streams_[i].name == name) return static_cast<int32_t>(i);
    }
    return -1;
  }

  int32_t Count() const { return static_cast<int32_t>(streams_.size()); }
  bool Valid(int32_t idx) const {
    return idx >= 0 && static_cast<size_t>(idx) < streams_.size();
  }
  const Stream* At(int32_t idx) const {
    return Valid(idx) ? &streams_[static_cast<size_t>(idx)] : nullptr;
  }

  // Unsubscribing RELEASES the buffer rather than just clearing it. A stream that collected a
  // megabyte and was then switched off should not keep holding it: the memory belongs to the
  // subscription, and nothing can read those records any more anyway.
  bool Subscribe(int32_t idx, bool on) {
    if (!Valid(idx)) return false;
    Stream& s = streams_[static_cast<size_t>(idx)];
    s.subscribed = on;
    if (!on) {
      std::vector<uint8_t>().swap(s.bytes);
      s.dropped = 0;
    }
    return true;
  }

  bool Subscribed(int32_t idx) const {
    const Stream* s = At(idx);
    return s != nullptr && s->subscribed;
  }

  // A COUNT, not a list. `BuildUpdate` walks this every frame, and stage 3 nearly shipped a
  // per-frame heap allocation by returning a vector from exactly this shape of accessor.
  int32_t SubscribedCount() const {
    int32_t n = 0;
    for (const Stream& s : streams_) {
      if (s.subscribed) ++n;
    }
    return n;
  }

  // Appends one record. Returns true if it was stored -- false for an unknown stream, a size
  // that is not the declared stride, no subscriber, or a stream that has hit its cap.
  //
  // FALSE IS THE COMMON CASE AND IS NOT AN ERROR: nobody is subscribed. Call sites must not
  // log or branch on it, or the diagnostic becomes the thing that needs a diagnostic.
  bool Emit(int32_t idx, const void* record, size_t size) {
    if (!Valid(idx)) return false;
    Stream& s = streams_[static_cast<size_t>(idx)];
    if (!s.subscribed) return false;
    if (record == nullptr || size != static_cast<size_t>(s.stride)) return false;
    if (s.bytes.size() + size > static_cast<size_t>(kExtStreamBytesPerFrame)) {
      ++s.dropped;
      return false;
    }
    const size_t off = s.bytes.size();
    s.bytes.resize(off + size);
    memcpy(s.bytes.data() + off, record, size);
    return true;
  }

  const std::vector<uint8_t>& Bytes(int32_t idx) const {
    static const std::vector<uint8_t> kEmpty;
    return Valid(idx) ? streams_[static_cast<size_t>(idx)].bytes : kEmpty;
  }

  int32_t Records(int32_t idx) const {
    const Stream* s = At(idx);
    if (s == nullptr || s->stride <= 0) return 0;
    return static_cast<int32_t>(s->bytes.size() / static_cast<size_t>(s->stride));
  }

  int32_t Dropped(int32_t idx) const {
    const Stream* s = At(idx);
    return s == nullptr ? 0 : s->dropped;
  }

  // Called at the END of `BuildUpdate`, once the frame's copy has been taken -- NOT from
  // `ClearFrameEvents` at the top of the frame, where Klei's ten vectors are cleared.
  //
  // The difference is load-bearing rather than stylistic. This DLL can refuse a message on the
  // GAME thread, between frames, on the immediate-message path. A record produced there would
  // be wiped by the next frame's opening clear before any publish could carry it, and the one
  // diagnostic whose whole job is to stop a silent drop would silently drop it. Clearing after
  // the copy means every record reaches exactly one published frame, whichever side of the
  // frame boundary produced it.
  //
  // `capacity` is kept: a subscribed stream is a stream somebody expects to fill again next
  // frame, and handing the allocation back every frame would be a per-frame malloc on the
  // publish path. `Subscribe(idx, false)` is what actually releases it.
  void ClearFrame() {
    for (Stream& s : streams_) {
      s.bytes.clear();
      s.dropped = 0;
    }
  }

 private:
  std::vector<Stream> streams_;
};

}  // namespace oni_sim::ext
