/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*
 * Methods to dump CacheIR to a string of C++ code that can then be
 * invoked to recreate the CacheIR, in a way that works across changes
 * to enum values.
 */

#include "jit/CacheIRPreInit.h"

#include "jit/CacheIR.h"
#include "jit/CacheIRReader.h"
#include "js/TypeDecls.h"

using namespace js;
using namespace jit;

class MOZ_RAII CacheIRPreInitDumper {
  GenericPrinter& out_;

  CACHE_IR_SPEWER_GENERATED

  void spewOp(CacheOp op) {
    const char* opName = CacheIROpNames[size_t(op)];
    out_.printf("%-30s", opName);
  }
  void spewOpEnd() { out_.printf("\n"); }

  void spewArgSeparator() { out_.printf(", "); }

  void spewOperandId(const char* name, OperandId id) {
    spewRawOperandId(name, id.id());
  }
  void spewRawOperandId(const char* name, uint32_t id) {
    out_.printf("%s %u", name, id);
  }
  void spewField(const char* name, uint32_t offset) {
    out_.printf("%s %u", name, offset);
  }
  void spewBoolImm(const char* name, bool b) {
    out_.printf("%s %s", name, b ? "true" : "false");
  }
  void spewByteImm(const char* name, uint8_t val) {
    out_.printf("%s %u", name, val);
  }
  void spewJSOpImm(const char* name, JSOp op) {
    out_.printf("%s JSOp::%s", name, CodeName(op));
  }
  void spewStaticStringImm(const char* name, const char* str) {
    out_.printf("%s \"%s\"", name, str);
  }
  void spewInt32Imm(const char* name, int32_t val) {
    out_.printf("%s %d", name, val);
  }
  void spewUInt32Imm(const char* name, uint32_t val) {
    out_.printf("%s %u", name, val);
  }
  void spewCallFlagsImm(const char* name, CallFlags flags) {
    out_.printf(
        "%s (format %u%s%s%s)", name, flags.getArgFormat(),
        flags.isConstructing() ? ", isConstructing" : "",
        flags.isSameRealm() ? ", isSameRealm" : "",
        flags.needsUninitializedThis() ? ", needsUninitializedThis" : "");
  }
  void spewJSWhyMagicImm(const char* name, JSWhyMagic magic) {
    out_.printf("%s JSWhyMagic(%u)", name, unsigned(magic));
  }
  void spewScalarTypeImm(const char* name, Scalar::Type type) {
    out_.printf("%s Scalar::Type(%u)", name, unsigned(type));
  }
  void spewUnaryMathFunctionImm(const char* name, UnaryMathFunction fun) {
    const char* funName = GetUnaryMathFunctionName(fun);
    out_.printf("%s UnaryMathFunction::%s", name, funName);
  }
  void spewValueTypeImm(const char* name, ValueType type) {
    out_.printf("%s ValueType(%u)", name, unsigned(type));
  }
  void spewJSNativeImm(const char* name, JSNative native) {
    out_.printf("%s %p", name, native);
  }
  void spewGuardClassKindImm(const char* name, GuardClassKind kind) {
    out_.printf("%s GuardClassKind(%u)", name, unsigned(kind));
  }
  void spewWasmValTypeImm(const char* name, wasm::ValType::Kind kind) {
    out_.printf("%s WasmValTypeKind(%u)", name, unsigned(kind));
  }
  void spewAllocKindImm(const char* name, gc::AllocKind kind) {
    out_.printf("%s AllocKind(%u)", name, unsigned(kind));
  }
  void spewCompletionKindImm(const char* name, CompletionKind kind) {
    out_.printf("%s CompletionKind(%u)", name, unsigned(kind));
  }

 public:
  CacheIRPreInitDumper(GenericPrinter& out) : out_(out) {}

  void spew(CacheIRReader& reader) {
    do {
      switch (reader.readOp()) {
#define SPEW_OP(op, ...) \
  case CacheOp::op:      \
    spew##op(reader);    \
    break;
        CACHE_IR_OPS(SPEW_OP)
#undef SPEW_OP

        default:
          MOZ_CRASH("Invalid op");
      }
    } while (reader.more());
  }
};

void js::jit::DumpCacheIRPreInit(GenericPrinter& out, const uint8_t* code) {
  CacheIRReader reader(code);
  CacheIRPreInitDumper dumper(out);
  dumper.spew(reader);
}

void js::jit::JitZone::dumpCacheIRPreInit(GenericPrinter& out) {
    for (auto it = baselineCacheIRStubCodes_.iter(); !it.done(); it.next()) {
        const CacheIRStubInfo* stubInfo = it.get().key().stubInfo.get();
        DumpCacheIRPreInit(out, stubInfo->code());
    }
}
