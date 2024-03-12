/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/Architecture-wasm32.h"
#include "jit/RegisterSets.h"

namespace js::jit {

FloatRegisterSet FloatRegister::ReduceSetForPush(const FloatRegisterSet& s) {
#ifdef ENABLE_WASM_SIMD
#  error "Needs more careful logic if SIMD is enabled"
#endif

  LiveFloatRegisterSet ret;
  for (FloatRegisterIterator iter(s); iter.more(); ++iter) {
    ret.addUnchecked(FromCode((*iter).encoding()));
  }
  return ret.set();
}

}  // namespace js::jit
