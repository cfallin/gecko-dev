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
    out_.printf("  writer.writeOp(CacheOp::%s);\n", opName);
  }
  void spewOpEnd() {}

  void spewArgSeparator() {}

  void spewOperandId(const char* name, OperandId id) {
    spewRawOperandId(name, id.id());
  }
  void spewRawOperandId(const char* name, uint32_t id) {
    out_.printf("  writer.writeOperandId(OperandId(%d));\n", id);
  }
  void spewField(const char* name, uint32_t offset, StubField::Type type) {
    const char* ty;
    switch (type) {
      case StubField::Type::RawInt32:
        ty = "RawInt32";
        break;
      case StubField::Type::RawPointer:
        ty = "RawPointer";
        break;
      case StubField::Type::Shape:
        ty = "Shape";
        break;
      case StubField::Type::GetterSetter:
        ty = "GetterSetter";
        break;
      case StubField::Type::JSObject:
        ty = "JSObject";
        break;
      case StubField::Type::Symbol:
        ty = "Symbol";
        break;
      case StubField::Type::String:
        ty = "String";
        break;
      case StubField::Type::BaseScript:
        ty = "BaseScript";
        break;
      case StubField::Type::JitCode:
        ty = "JitCode";
        break;
      case StubField::Type::Id:
        ty = "Id";
        break;
      case StubField::Type::AllocSite:
        ty = "AllocSite";
        break;
      case StubField::Type::RawInt64:
        ty = "RawInt64";
        break;
      case StubField::Type::Value:
        ty = "Value";
        break;
      case StubField::Type::Double:
        ty = "Double";
        break;
      default:
        MOZ_CRASH("Unknown StubField type");
    }
    out_.printf("  writer.addStubField(0, StubField::Type::%s);\n", ty);
  }
  void spewBoolImm(const char* name, bool b) {
    out_.printf("  writer.writeBoolImm(%s);\n", b ? "true" : "false");
  }
  void spewByteImm(const char* name, uint8_t val) {
    out_.printf("  writer.writeByteImm(%d);\n", val);
  }
  void spewJSOpImm(const char* name, JSOp op) {
    out_.printf("  writer.writeJSOpImm(JSOp::%s);\n", CodeName(op));
  }
  void spewStaticStringImm(const char* name, const char* str) {
    out_.printf("  writer.addStubField(0, StubField::Type::RawPointer);\n");
  }
  void spewInt32Imm(const char* name, int32_t val) {
    out_.printf("  writer.writeInt32Imm(%d);\n", val);
  }
  void spewUInt32Imm(const char* name, uint32_t val) {
    out_.printf("  writer.writeUInt32Imm(%u);\n", val);
  }
  void spewCallFlagsImm(const char* name, CallFlags flags) {
    out_.printf("  writer.writeByteImm(%d);\n", flags.toByte());
  }
  void spewJSWhyMagicImm(const char* name, JSWhyMagic magic) {
    out_.printf("  writer.writeByteImm(%d);\n", int(magic));
  }
  void spewScalarTypeImm(const char* name, Scalar::Type type) {
    // See comment on Scalar::Type: existing types are guaranteed
    // not to be renumbered, so we can embed the integer value here.
    out_.printf("  writer.writeByteImm(%d);\n", int(type));
  }
  void spewUnaryMathFunctionImm(const char* name, UnaryMathFunction fun) {
    out_.printf("  writer.writeByteImm(%d);\n", int(fun));
  }
  void spewValueTypeImm(const char* name, ValueType type) {
    out_.printf("  writer.writeByteImm(%d);\n", int(type));
  }
  void spewJSNativeImm(const char* name, JSNative native) {
    out_.printf("  writer.addStubField(0, StubField::Type::RawPointer);\n");
  }
  void spewGuardClassKindImm(const char* name, GuardClassKind kind) {
    out_.printf("  writer.writeByteImm(%d);\n", int(kind));
  }
  void spewWasmValTypeImm(const char* name, wasm::ValType::Kind kind) {
    out_.printf("  writer.writeByteImm(%d);\n", int(kind));
  }
  void spewAllocKindImm(const char* name, gc::AllocKind kind) {
    out_.printf("  writer.writeByteImm(%d);\n", int(kind));
  }
  void spewCompletionKindImm(const char* name, CompletionKind kind) {
    out_.printf("  writer.writeByteImm(%d);\n", int(kind));
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

void js::jit::DumpCacheIRPreInit(GenericPrinter& out,
                                 const CacheIRStubInfo* stubInfo) {
  CacheIRReader reader(stubInfo);
  CacheIRPreInitDumper dumper(out);
  dumper.spew(reader);
}

void js::jit::JitZone::dumpCacheIRPreInit(GenericPrinter& out) {
  for (auto it = baselineCacheIRStubCodes_.iter(); !it.done(); it.next()) {
    const CacheIRStubInfo* stubInfo = it.get().key().stubInfo.get();
    out.printf("_(CacheKind::%s, {\n", CacheKindNames[int(stubInfo->kind())]);
    DumpCacheIRPreInit(out, stubInfo);
    out.printf("});\n");
  }
}
