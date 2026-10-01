// The offline suites' reference values, and the machinery that makes them check themselves.
//
// WHY THIS FILE EXISTS. A golden kept as a number in prose, checked by a person running a
// pipeline and comparing by eye, drifts silently: nothing fails when the scenario set grows
// and the number is not re-recorded, and a suite that prints "413 PASS" with nothing saying
// it should be 413 reads as success when an arm stops running.
//
// A golden nobody re-runs is a comment. This header is the build-time half.
//
// THE DIGEST IS TAKEN INSIDE THE SUITE, NOT BY A PIPELINE. `diffsim` loads two SimDLLs into
// its own process, and Klei's prints one
//
//     THREAD - started 'SimThread' (55156)
//
// per sim it starts, carrying an OS thread id that changes every run. That is the ONLY
// non-deterministic byte anywhere in the output, and the recorded grep pipelines existed to
// strip it. Rather than keep a pattern list, `diffsim` hashes the bytes IT emitted: the DLL's
// line goes out through the DLL's own C runtime and never passes through our wrapper, so it
// is excluded structurally rather than lexically. There is no filter left to get wrong.
//
// The DLL turns out to write that line to STDERR, which nobody had noticed, and which is why
// the recorded pipelines only ever needed it when the capture used `2>&1`. Two of the three
// clauses in the last recorded filter were inert for reasons nobody could see either: the
// output is CRLF, so `ms$` can never match, and `--scenario all` prints no `[`-prefixed line
// at all. A filter whose clauses are silently doing nothing is the argument for this file in
// miniature.
//
// NEWLINES ARE HASHED AS CRLF ON PURPOSE. These tools are mingw-built Windows binaries, so
// the C runtime turns every '\n' into "\r\n" on the way to a file or a pipe. Expanding it
// here too keeps the value equal to what a reviewer gets from
//
//     diffsim.exe --corpus <pinned> --scenario all > out.txt   # note: no 2>&1
//     md5sum out.txt
//
// which is the whole point of choosing md5 over something cheaper: the assertion the suite
// makes and the check a human can make by hand are the same arithmetic over the same bytes,
// so neither one has to be taken on faith. Verified equal to the digit -- see
// driver/README.md, "The goldens check themselves", for that check and for the nine probes
// that proved this machinery fails on purpose.
//
// WHAT A KEY IS. One line of `driver/GOLDENS.txt`, `key value`, `#` comments and blank lines
// ignored. The file is the single tracked home for every one of these values; prose keeps a
// POINTER to it and never a copy, because a copy is what drifted last time.

#pragma once

#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace goldens {

// ---------------------------------------------------------------------------------------
// MD5. Self-contained because these are mingw cross-builds with no crypto library, and
// because a digest a reviewer cannot reproduce with `md5sum` is worth less than no digest.
// ---------------------------------------------------------------------------------------
class Md5 {
 public:
  Md5() { Reset(); }

  void Reset() {
    a_ = 0x67452301u; b_ = 0xefcdab89u; c_ = 0x98badcfeu; d_ = 0x10325476u;
    length_ = 0; buffered_ = 0;
  }

  void Update(const void* data, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    length_ += n;
    while (n > 0) {
      const size_t take = (64 - buffered_ < n) ? (64 - buffered_) : n;
      memcpy(block_ + buffered_, p, take);
      buffered_ += take; p += take; n -= take;
      if (buffered_ == 64) { Transform(block_); buffered_ = 0; }
    }
  }

  // Hex digest. Finalising does not disturb the running state, so a caller may take an
  // interim digest and keep hashing -- which is what `diffsim` needs: it has to know the
  // value BEFORE it prints the verdict line, and the verdict must not be inside its own hash.
  std::string HexDigest() const {
    Md5 copy = *this;
    const uint64_t bits = copy.length_ * 8;
    static const uint8_t kPad[64] = { 0x80 };
    const size_t pad = (copy.buffered_ < 56) ? (56 - copy.buffered_) : (120 - copy.buffered_);
    copy.length_ = 0;  // Update() must not re-count the padding
    copy.Update(kPad, pad);
    uint8_t tail[8];
    for (int i = 0; i < 8; ++i) tail[i] = static_cast<uint8_t>((bits >> (8 * i)) & 0xff);
    copy.Update(tail, 8);
    const uint32_t words[4] = { copy.a_, copy.b_, copy.c_, copy.d_ };
    static const char* kHex = "0123456789abcdef";
    std::string out;
    out.reserve(32);
    for (int w = 0; w < 4; ++w) {
      for (int i = 0; i < 4; ++i) {
        const uint8_t byte = static_cast<uint8_t>((words[w] >> (8 * i)) & 0xff);
        out.push_back(kHex[byte >> 4]);
        out.push_back(kHex[byte & 0x0f]);
      }
    }
    return out;
  }

 private:
  static uint32_t Rol(uint32_t x, int c) { return (x << c) | (x >> (32 - c)); }

  void Transform(const uint8_t block[64]) {
    static const uint32_t K[64] = {
      0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu, 0xf57c0fafu, 0x4787c62au,
      0xa8304613u, 0xfd469501u, 0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu,
      0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u, 0xf61e2562u, 0xc040b340u,
      0x265e5a51u, 0xe9b6c7aau, 0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
      0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu, 0xa9e3e905u, 0xfcefa3f8u,
      0x676f02d9u, 0x8d2a4c8au, 0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu,
      0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u, 0x289b7ec6u, 0xeaa127fau,
      0xd4ef3085u, 0x04881d05u, 0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
      0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u, 0x655b59c3u, 0x8f0ccc92u,
      0xffeff47du, 0x85845dd1u, 0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
      0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u,
    };
    static const int S[64] = {
      7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
      5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
      4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
      6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
    };
    uint32_t m[16];
    for (int i = 0; i < 16; ++i) {
      m[i] = static_cast<uint32_t>(block[i * 4]) |
             (static_cast<uint32_t>(block[i * 4 + 1]) << 8) |
             (static_cast<uint32_t>(block[i * 4 + 2]) << 16) |
             (static_cast<uint32_t>(block[i * 4 + 3]) << 24);
    }
    uint32_t a = a_, b = b_, c = c_, d = d_;
    for (int i = 0; i < 64; ++i) {
      uint32_t f;
      int g;
      if (i < 16)      { f = (b & c) | (~b & d);          g = i; }
      else if (i < 32) { f = (d & b) | (~d & c);          g = (5 * i + 1) & 15; }
      else if (i < 48) { f = b ^ c ^ d;                   g = (3 * i + 5) & 15; }
      else             { f = c ^ (b | ~d);                g = (7 * i) & 15; }
      const uint32_t tmp = d;
      d = c;
      c = b;
      b = b + Rol(a + f + K[i] + m[g], S[i]);
      a = tmp;
    }
    a_ += a; b_ += b; c_ += c; d_ += d;
  }

  uint32_t a_, b_, c_, d_;
  uint64_t length_;
  size_t buffered_;
  uint8_t block_[64];
};

// ---------------------------------------------------------------------------------------
// The output sink. Everything a suite prints goes through Emit(); everything a DLL prints
// does not, which is the whole mechanism.
// ---------------------------------------------------------------------------------------
class Digest {
 public:
  // `text` is the bytes as the program wrote them, with bare '\n'. The C runtime will expand
  // those to "\r\n" on the way to the terminal, so the hash expands them too -- see the file
  // header for why the two have to agree.
  void Emit(const char* text, size_t n) {
    // Whole runs between newlines, not a byte at a time: the suites this feeds sit in a
    // spin-wait handshake with a SimDLL worker thread, and slowing the main thread's printf
    // is not a free thing to do -- see driver/README.md on the intermittent hang.
    size_t start = 0;
    for (size_t i = 0; i < n; ++i) {
      if (text[i] == '\n') {
        static const char kCrLf[2] = { '\r', '\n' };
        md5_.Update(text + start, i - start);
        md5_.Update(kCrLf, 2);
        ++lines_;
        start = i + 1;
      }
    }
    if (start < n) md5_.Update(text + start, n - start);
  }

  std::string HexDigest() const { return md5_.HexDigest(); }
  long Lines() const { return lines_; }

 private:
  Md5 md5_;
  long lines_ = 0;
};

// ---------------------------------------------------------------------------------------
// driver/GOLDENS.txt
// ---------------------------------------------------------------------------------------

// A missing or unreadable goldens file is a FAILURE, never a skip. The failure mode this
// whole item exists to close is a check that quietly does nothing, and "the file wasn't
// there so we passed" is that failure mode wearing a different hat.
class File {
 public:
  explicit File(const char* path) : path_(path) {
    // THE OPEN IS RETRIED, AND THAT IS NOT DEFENSIVE PROGRAMMING. Run many times in a row
    // under WSL, the open fails about **1 time in 40**, always transiently, and the run then reports
    // "cannot read GOLDENS.txt" and exits 1 with nothing wrong with the file. These are PE
    // binaries run from WSL through binfmt interop against a Linux filesystem, and a
    // transient open failure is a property of that path rather than of the tooling.
    //
    // A suite that fails 2.5 % of the time for a reason nobody can act on is a suite people
    // learn to re-run rather than read, which is exactly the value a golden has. Four attempts
    // with a short backoff; `errno` from the LAST one is reported when it still will not open,
    // so the next person sees the cause instead of the symptom. A genuinely missing file
    // returns ENOENT four times and costs three milliseconds.
    FILE* f = nullptr;
    for (int attempt = 0; attempt < 4; ++attempt) {
      errno = 0;
      f = fopen(path, "rb");
      if (f) break;
      open_errno_ = errno;
      if (attempt < 3) std::this_thread::sleep_for(std::chrono::milliseconds(1 << attempt));
      ++retries_;
    }
    if (!f) return;
    char line[1024];
    while (fgets(line, sizeof(line), f)) lines_.push_back(line);
    fclose(f);
    loaded_ = true;
  }

  // 0 unless the first open failed; how many attempts it took, and what the last failure was.
  int retries() const { return retries_; }
  int open_errno() const { return open_errno_; }

  bool loaded() const { return loaded_; }
  const char* path() const { return path_; }

  // Returns false if the key is absent.
  bool Get(const char* key, std::string* out) const {
    for (size_t i = 0; i < lines_.size(); ++i) {
      std::string k, v;
      if (Split(lines_[i], &k, &v) && k == key) { *out = v; return true; }
    }
    return false;
  }

  void Set(const char* key, const std::string& value) {
    for (size_t i = 0; i < lines_.size(); ++i) {
      std::string k, v;
      if (Split(lines_[i], &k, &v) && k == key) {
        lines_[i] = Format(key, value);
        return;
      }
    }
    lines_.push_back(Format(key, value));
  }

  // Rewrites the file, keeping every comment and blank line exactly where it was: a
  // re-record has to produce a diff a reviewer can read, and a command that flattens the
  // explanations around the values is not that.
  bool Write() const {
    FILE* f = fopen(path_, "wb");
    if (!f) return false;
    for (size_t i = 0; i < lines_.size(); ++i) fputs(lines_[i].c_str(), f);
    fclose(f);
    return true;
  }

 private:
  // Values are written in a fixed column so a re-record produces a one-token diff rather
  // than a reflowed line. The diff is the review; it should be readable.
  static std::string Format(const char* key, const std::string& value) {
    std::string line(key);
    while (line.size() < 20) line.push_back(' ');
    // A key of exactly the column width, or wider, would otherwise be written hard against
    // its value with no separator at all -- and `Split` would then read the pair as one long
    // key, so the suite that had just re-recorded a value could not find it again on the next
    // run. It fails as "GOLDENS.txt has no key ...", which points at the reader rather than at
    // the writer; found by adding the first key 20 characters long. Existing keys are all
    // shorter, so this changes no line that is already in the file.
    if (line.size() >= 20) line.push_back(' ');
    return line + value + "\n";
  }

  static bool Split(const std::string& raw, std::string* key, std::string* value) {
    size_t b = 0;
    while (b < raw.size() && (raw[b] == ' ' || raw[b] == '\t')) ++b;
    if (b >= raw.size() || raw[b] == '#' || raw[b] == '\n' || raw[b] == '\r') return false;
    size_t e = b;
    while (e < raw.size() && raw[e] != ' ' && raw[e] != '\t') ++e;
    *key = raw.substr(b, e - b);
    while (e < raw.size() && (raw[e] == ' ' || raw[e] == '\t')) ++e;
    size_t z = raw.size();
    while (z > e && (raw[z - 1] == '\n' || raw[z - 1] == '\r' ||
                     raw[z - 1] == ' ' || raw[z - 1] == '\t')) --z;
    *value = raw.substr(e, z - e);
    return true;
  }

  const char* path_;
  std::vector<std::string> lines_;
  bool loaded_ = false;
  int retries_ = 0;
  int open_errno_ = 0;
};

// ---------------------------------------------------------------------------------------
// The count assertion. `vftest` and `gastest` do not need a digest -- their output carries
// per-arm text that changes whenever a description is reworded -- but they do need to be
// unable to lose an arm silently. The number recorded is the number of Check() calls that
// RAN, not the number that passed: a failing arm already fails the suite by itself, and the
// hole a pass/fail count cannot see is an arm that stops executing at all.
//
// Reports on stderr for the same reason the digest does: a verdict about the output does not
// belong inside the output.
// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------
// The digest assertion. Same file, same key syntax, same re-record discipline as the count
// above -- what differs is only that the value is a hex md5 rather than a number, and that a
// failure needs a sentence saying what a move MEANS, because "this hash changed" is not by
// itself something a reader can act on.
//
// IT LIVES HERE RATHER THAN IN ONE SUITE because two suites now assert digests and they must
// not drift apart in how they do it. `bench` pins what its kernels looked at and left behind;
// `vftest` pins what the DELIBERATE DEVIATIONS produce, which `diffsim` cannot see by
// construction -- diffsim asks "do we match Klei", and everything this project adds on
// purpose is a difference from Klei. Two oracles, two questions.
// ---------------------------------------------------------------------------------------
inline bool CheckDigest(const char* path, const char* key, const std::string& actual,
                        bool record, const char* noncanonical, const char* what_moved) {
  if (noncanonical && !record) {
    fprintf(stderr,
            "\ngoldens: NOT CHECKED -- %s makes this a non-canonical run; %s describes the\n"
            "goldens: default invocation only. This run: %s\n",
            noncanonical, key, actual.c_str());
    return true;
  }
  File file(path);
  if (!file.loaded() && !record) {
    fprintf(stderr,
            "\ngoldens: FAILED -- cannot read %s after %d attempts (errno %d: %s). The\n"
            "goldens: reference values are a tracked file, not an optional one; run from\n"
            "goldens: driver/ or pass --goldens <path>.\n",
            path, file.retries(), file.open_errno(), strerror(file.open_errno()));
    return false;
  }
  if (record) {
    if (noncanonical) {
      fprintf(stderr,
              "\ngoldens: REFUSED to re-record -- %s makes this a non-canonical run, and a\n"
              "goldens: reference taken from an invocation nobody else uses is worse than\n"
              "goldens: none.\n",
              noncanonical);
      return false;
    }
    file.Set(key, actual);
    if (!file.Write()) {
      fprintf(stderr, "\ngoldens: FAILED -- could not write %s\n", path);
      return false;
    }
    fprintf(stderr, "\ngoldens: RE-RECORDED  %s  %s\n", key, actual.c_str());
    return true;
  }
  std::string expected;
  if (!file.Get(key, &expected)) {
    fprintf(stderr, "\ngoldens: FAILED -- %s has no key %s\n", path, key);
    return false;
  }
  if (expected != actual) {
    fprintf(stderr,
            "\ngoldens: FAILED -- %s\n"
            "goldens:   expected %s\n"
            "goldens:   actual   %s\n"
            "goldens: %s\n"
            "goldens: If the change is intended, re-record with --record-goldens and commit\n"
            "goldens: the diff -- that diff is the review.\n",
            key, expected.c_str(), actual.c_str(), what_moved);
    return false;
  }
  fprintf(stderr, "\ngoldens: PASS -- %s %s (%s)\n", key, actual.c_str(), path);
  return true;
}

inline bool CheckCount(const char* path, const char* key, long actual, bool record,
                       const char* noncanonical) {
  char actual_text[32];
  snprintf(actual_text, sizeof(actual_text), "%ld", actual);

  if (noncanonical && !record) {
    fprintf(stderr,
            "\ngoldens: NOT CHECKED -- %s makes this a non-canonical run; %s describes the\n"
            "goldens: default invocation only. This run: %s arms.\n",
            noncanonical, key, actual_text);
    return true;
  }

  File file(path);
  if (!file.loaded() && !record) {
    fprintf(stderr,
            "\ngoldens: FAILED -- cannot read %s after %d attempts (errno %d: %s). The\n"
            "goldens: reference values are a tracked file, not an optional one; run from\n"
            "goldens: driver/ or pass --goldens <path>.\n",
            path, file.retries(), file.open_errno(), strerror(file.open_errno()));
    return false;
  }

  if (record) {
    if (noncanonical) {
      fprintf(stderr,
              "\ngoldens: REFUSED to re-record -- %s makes this a non-canonical run, and a\n"
              "goldens: reference taken from an invocation nobody else uses is worse than\n"
              "goldens: none.\n",
              noncanonical);
      return false;
    }
    file.Set(key, actual_text);
    if (!file.Write()) {
      fprintf(stderr, "\ngoldens: FAILED -- could not write %s\n", path);
      return false;
    }
    fprintf(stderr, "\ngoldens: RE-RECORDED into %s\n"
                    "goldens:   %s  %s\n"
                    "goldens: commit the diff -- that diff is the review.\n",
            path, key, actual_text);
    return true;
  }

  std::string expected;
  if (!file.Get(key, &expected)) {
    fprintf(stderr, "\ngoldens: FAILED -- %s has no key %s\n", path, key);
    return false;
  }
  if (expected != actual_text) {
    fprintf(stderr,
            "\ngoldens: FAILED -- %s\n"
            "goldens:   expected %s arms\n"
            "goldens:   actual   %s arms\n"
            "goldens: an arm was added, removed, or stopped running. If the change is\n"
            "goldens: intended, re-record with --record-goldens and commit the diff.\n",
            key, expected.c_str(), actual_text);
    return false;
  }
  fprintf(stderr, "\ngoldens: PASS -- %s %s arms (%s)\n", key, actual_text, path);
  return true;
}

}  // namespace goldens
