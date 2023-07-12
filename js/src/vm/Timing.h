/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef vm_Timing_h
#define vm_Timing_h

namespace js {
namespace timing {

static inline uint64_t rdtsc() {
  uint32_t hi, lo;
  asm volatile("rdtsc" : "=a"(lo), "=d"(hi));
  return uint64_t(hi) << 32 | lo;
}

struct Bin {
    uint64_t cycles;
    uint64_t counts;

    void accum(uint64_t cycleDelta) {
        cycles += cycleDelta;
        counts++;
    }
};

struct Timer {
    Bin& bin;
    uint64_t start;

    Timer(Bin& bin_) : bin(bin_), start(rdtsc()) {}
    ~Timer() {
        bin.cycles += rdtsc() - start;
        bin.counts++;
    }
};

extern Bin OpcodeTimers[257];

}  // namespace timing
}  // namespace js

#endif /* vm_Timing_h */
