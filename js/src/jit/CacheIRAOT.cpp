/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifdef ENABLE_JS_AOT_ICS

#  include "jit/CacheIRAOT.h"

#  include "jsmath.h"
#  include "jstypes.h"

#  include "gc/AllocKind.h"
#  include "jit/CacheIR.h"
#  include "jit/JitZone.h"
#  include "js/ScalarType.h"
#  include "js/Value.h"
#  include "vm/CompletionKind.h"
#  include "vm/Opcodes.h"
#  include "wasm/WasmValType.h"

// Include null pointers for "native" pointers: an AOT-loaded stub
// should not bake in an arbitrary pointer observed from a previous
// execution. In any case, the AOT corpus should not include any ICs
// that bake in pointers (baseline does not generate any).

#  if JS_BITS_PER_WORD == 32
#    define NATIVE_NULLPTR 0, 0, 0, 0
#  elif JS_BITS_PER_WORD == 64
#    define NATIVE_NULLPTR 0, 0, 0, 0, 0, 0, 0, 0
#  else
#    error Please add a case for a non-32/64-bit system here.
#  endif

// These correspond to the CacheIRWriter definitions of the serialized
// CacheIR format.

#  define OP(op) uint8_t(JSOp::op),
#  define ID(id) id,
#  define OFFSET(off) off,
#  define BOOL(x) x,
#  define BYTE(x) x,
#  define JSOP(op) uint8_t(JSOp::op),
#  define STATIC_STRING(p) NATIVE_NULLPTR,
#  define INT32(i)                                                             \
    uint32_t(i) & 0xff, (uint32_t(i) >> 8) & 0xff, (uint32_t(i) >> 16) & 0xff, \
        (uint32_t(i) >> 24) & 0xff
#  define UINT32(i) \
    (i) & 0xff, ((i) >> 8) & 0xff, ((i) >> 16) & 0xff, ((i) >> 24) & 0xff
#  define CALLFLAGS(f) f,
#  define WHYMAGIC(m) m,
#  define SCALARTYPE(name) uint8_t(Scalar::Type::name),
#  define UNARYMATHFUNC(name) uint8_t(UnaryMathFunction::name),
#  define VALUETYPE(name) uint8_t(ValueType::name),
#  define NATIVEIMM(p) NATIVE_NULLPTR,
#  define GUARDCLASSKIND(name) uint8_t(GuardClassKind::name),
#  define WASMVALTYPE(name) uint8_t(wasm::ValType::Kind::name),
#  define ALLOCKIND(name) uint8_t(gc::AllocKind::name),
#  define COMPLETIONKIND(name) uint8_t(CompletionKind::name),
#  define REALMFUSE(i) i,

// Other macros used to serialize parts of the CacheIRWriter.
#  define STUBFIELD(data, ty) \
    internal::StubField { StubField::Type::ty, data }
#  define LASTUSED(n) n

// First, generate individual IC bodies.

#  define IC(idx, _kind, _num_operand_ids, _num_input_operands,        \
             _num_instructions, _typedata, _stubdatasize, _stubfields, \
             _lastused, ops)                                           \
    static const uint8_t IC##idx = {ops};

#  include "jit/CacheIRAOTGenerated.h"

#  undef IC

// Generate the stubfield lists.

#  define IC(idx, _kind, _num_operand_ids, _num_input_operands,       \
             _num_instructions, _typedata, _stubdatasize, stubfields, \
             _lastused, _ops)                                         \
    static const internal::StubField IC##idx##StubFields = {stubfields};

#  include "jit/CacheIRAOTGenerated.h"

#  undef IC

// Generate the operand-last-used lists.

#  define IC(idx, _kind, _num_operand_ids, _num_input_operands,        \
             _num_instructions, _typedata, _stubdatasize, _stubfields, \
             lastused, _ops)                                           \
    static const AOTStubFieldData IC##idx##LastUsed = {lastused};

#  include "jit/CacheIRAOTGenerated.h"

#  undef IC

// Now, generate the toplevel list of AOT structs from which we can
// reconstitute a CacheIRWriter.

#  define IC(idx, kind, num_operand_ids, num_input_operands, num_instructions, \
             typedata, stubdatasize, _stubfields, _lastused, _ops)             \
    CacheIRAOTStub{                                                            \
        CacheKind::kind,                                                       \
        num_operand_ids,                                                       \
        num_input_operands,                                                    \
        num_instructions,                                                      \
        typedata,                                                              \
        stubdatasize,                                                          \
        &IC##idx##StubFields[0],                                               \
        sizeof(IC##idx##StubFields) / sizeof(IC##idx##StubFields[0]),          \
        &IC##idx##LastUsed[0],                                                 \
        &IC##idx[0],                                                           \
        sizeof(IC##idx)},

static const CacheIRAOTStub stubs[] = {
#  include "jit/CacheIRAOTGenerated.h"
};

mozilla::Span<const CacheIRAOTStub> js::jit::GetAOTStubs() {
  return mozilla::Span(stubs, sizeof(stubs) / sizeof(stubs[0]));
}

#endif /* ENABLE_JS_AOT_ICS */
