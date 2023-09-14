/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jsapi.h"

#include "jit/BaselineIC.h"
#include "jit/CacheIR.h"
#include "jit/CacheIRCompiler.h"
#include "jit/JitZone.h"
#include "vm/PortableBaselineInterpret.h"

using namespace js;
using namespace js::jit;

#ifdef ENABLE_JS_PBL_WEVAL
void js::PreloadCommonICs(JSContext* cx) {
  if (!IsPortableBaselineInterpreterEnabled()) {
    return;
  }
  if (!cx->zone()) {
    return;
  }
  JitZone* jitZone = cx->zone()->getJitZone(cx);

#  define _(kind, body)                                                       \
    {                                                                         \
      CacheIRWriter writer(cx);                                               \
      body;                                                                   \
      CacheIRStubKey::Lookup lookup(kind, ICStubEngine::Baseline,             \
                                    writer.codeStart(), writer.codeLength()); \
      CacheIRStubInfo* stubInfo = nullptr;                                    \
      jitZone->getBaselineCacheIRStubCode(lookup, &stubInfo);                 \
      if (!stubInfo) {                                                        \
        stubInfo = CacheIRStubInfo::New(kind, ICStubEngine::Baseline, true,   \
                                        sizeof(ICCacheIRStub), writer);       \
        EnqueuePortableBaselineICSpecialization(stubInfo);                    \
        CacheIRStubKey key(stubInfo);                                         \
        (void)jitZone->putBaselineCacheIRStubCode(lookup, key, nullptr);      \
      }                                                                       \
    }
#  include "vm/PortableBaselineICsCollected.h"
#  undef _
}
#endif
