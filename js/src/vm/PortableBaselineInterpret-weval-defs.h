/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef PortableBaselineInerpret_weval_defs_h
#define PortableBaselineInerpret_weval_defs_h

/* Basic definitions for PBL's internals that can be swapped out as
 * needed to handle interpreter details differently.
 *
 * Meant to be included only from PortableBaselineInterpret.cpp. */

#define PBL_HYBRID_ICS_DEFAULT false

#define PBL_CALL_IC(jitcode, ctx, stubvalue, result, arg0, arg1, arg2value, \
                    hasarg2)                                                \
  do {                                                                      \
    if (hasarg2) {                                                          \
      ctx.arg2 = arg2value;                                                 \
    }                                                                       \
    ICStubFunc func = reinterpret_cast<ICStubFunc>(jitcode);                \
    result = func(arg0, arg1, stubvalue, ctx);                              \
  } while (0)

#define PBL_CALL_INTERP(result, script, interp, ...) \
  result = interp(__VA_ARGS__);

#define PBL_ESTABLISH_STUBINFO_CODE(Specialized, stubInfo, code)            \
  if (!Specialized) {                                                       \
    stubInfo = cstub->stubInfo();                                           \
    code = stubInfo->code();                                                \
  } else {                                                                  \
    stubInfo = reinterpret_cast<const CacheIRStubInfo*>(                    \
        weval_read_specialization_global(0));                               \
    code = reinterpret_cast<uint8_t*>(weval_read_specialization_global(1)); \
  }

#define READ_REG(reg) \
  (Specialized ? weval_read_reg((reg)) : ctx.icregs.icVals[(reg)])
#define WRITE_REG(reg, value, tagtype)                                        \
  if (Specialized) {                                                          \
    weval_write_reg((reg), (value));                                          \
    weval_write_reg((reg) + ICRegs::kMaxICVals, uint64_t(JSVAL_TAG_##tagtype) \
                                                    << JSVAL_TAG_SHIFT);      \
  } else {                                                                    \
    ctx.icregs.icVals[(reg)] = (value);                                       \
    ctx.icregs.icTags[(reg)] = uint64_t(JSVAL_TAG_##tagtype)                  \
                               << JSVAL_TAG_SHIFT;                            \
  }

#define READ_VALUE_REG(reg)                                       \
  Value::fromRawBits(                                             \
      Specialized ? (weval_read_reg((reg) + ICRegs::kMaxICVals) | \
                     weval_read_reg((reg)))                       \
                  : (ctx.icregs.icTags[(reg)] | ctx.icregs.icVals[(reg)]))
#define WRITE_VALUE_REG(reg, value)                 \
  if (Specialized) {                                \
    weval_write_reg((reg), (value).asRawBits());    \
    weval_write_reg((reg) + ICRegs::kMaxICVals, 0); \
  } else {                                          \
    ctx.icregs.icVals[(reg)] = (value).asRawBits(); \
    ctx.icregs.icTags[(reg)] = 0;                   \
  }

#define PBL_PUSH_IC_CTX() \
  weval::push_context(    \
      reinterpret_cast<uint32_t>(cacheIRReader.currentPosition()));

#define PBL_UPDATE_IC_CTX() \
  weval::update_context(    \
      reinterpret_cast<uint32_t>(cacheIRReader.currentPosition()));

#define PBL_POP_IC_CTX() \
    weval::pop_context();

#endif /* PortableBaselineInerpret_defs_h */
