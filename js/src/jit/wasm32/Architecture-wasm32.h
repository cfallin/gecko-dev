/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_Architecture_wasm32_h
#define jit_wasm32_Architecture_wasm32_h

#include "mozilla/MathAlgorithms.h"

// JitSpewer.h is included through MacroAssembler implementations for other
// platforms, so include it here to avoid inadvertent build bustage.
#include "jit/JitSpewer.h"

#include "jit/shared/Architecture-shared.h"

namespace js::jit {

static const uint32_t SimdMemoryAlignment = 16;
static const uint32_t WasmStackAlignment = SimdMemoryAlignment;
static const uint32_t WasmTrapInstructionLength = 2;

// See comments in wasm::GenerateFunctionPrologue.
static constexpr uint32_t WasmCheckedCallEntryOffset = 0u;

class Registers {
 public:
  enum RegisterID {
    wsp = 0,  // corresponds to global __stack_pointer which is mapped into
              // global[0]
    first_local_id = 1,
    wbp = first_local_id,
    wcx_64 = 2,
    wax_64 = 3,
    wdi_64 = 4,
    wbx_64 = 5,
    wdx_64 = 6,
    wsi_64 = 7,
    w0 = 8,
    w1 = 9,
    w2 = 10,
    w3 = 11,
    w4 = 12,
    w5 = 13,
    w6 = 14,
    w7 = 15,
    invalid_reg = 9,
    invalid_reg2 = 10,  // To avoid silly static_assert failures.
  };

  typedef uint16_t Code;

  typedef RegisterID Encoding;
  union RegisterContent {
    uintptr_t r;
  };

  using SetType = uint16_t;

  static uint32_t SetSize(SetType set) {
    static_assert(sizeof(SetType) <= 4, "SetType must be, at most, 32 bits");
    return mozilla::CountPopulation32(set);
  }

  static uint32_t FirstBit(SetType set) {
    return mozilla::CountTrailingZeroes32(set);
  }

  static uint32_t LastBit(SetType set) {
    return 31 - mozilla::CountLeadingZeroes32(set);
  }

  static const char* GetName(Code) { MOZ_CRASH(); }
  static Code FromName(const char*) { MOZ_CRASH(); }

  static const Encoding Invalid = invalid_reg;
  static const uint32_t Total = 16;
  static const uint32_t TotalPhys = Total;
  static const uint32_t Allocatable = 14;

  static const uint32_t NumLocals = Total - static_cast<int>(first_local_id);

  static const SetType AllMask = (1 << Total) - 1;

  static const SetType VolatileMask =
      (1 << RegisterID::wax_64) | (1 << RegisterID::wcx_64) |
      (1 << RegisterID::wdx_64) | (1 << RegisterID::w0);

  static const SetType WrapperMask = VolatileMask | (1 << RegisterID::wbx_64);

  static const SetType NonVolatileMask =
      AllMask & ~VolatileMask & ~(1 << RegisterID::wsp);

  static const SetType NonAllocatableMask =
      (1 << RegisterID::wsp) | (1 << RegisterID::wbp) |
      (1 << RegisterID::w0);  // This is ScratchReg

  static const SetType AllocatableMask = AllMask & ~NonAllocatableMask;

  // Registers returned from a JS -> JS call.
  static const SetType JSCallMask = (1 << RegisterID::wcx_64);

  // Registers returned from a JS -> C call.
  static const SetType CallMask = (1 << RegisterID::wax_64);

  static bool IsLocal(uint8_t id) { return id >= RegisterID::first_local_id; }

  static uint32_t ToLocalIndex(uint8_t id) {
    MOZ_RELEASE_ASSERT(IsLocal(id));
    return id - RegisterID::first_local_id;
  }

  static bool IsGlobal(uint8_t id) { return id < RegisterID::first_local_id; }

  static bool Is64Bit(uint8_t id) {
    return id != RegisterID::wsp && id != RegisterID::wbp;
  }
};

typedef uint8_t PackedRegisterMask;

class FloatRegisters {
 public:
  enum FPRegisterID {
    first_local_id = 0,
    f0 = first_local_id,
    f1 = 1,
    f2 = 2,
    f3 = 3,
    f4 = 4,
    f5 = 5,
    f6 = 6,
    f7 = 7,
    first_double_reg_id = 8,
    d0 = first_double_reg_id,
    d1 = 9,
    d2 = 10,
    d3 = 11,
    d4 = 12,
    d5 = 13,
    d6 = 14,
    d7 = 15,
    invalid_reg
  };

  using Encoding = FPRegisterID;
  using Code = FPRegisterID;

  // Content spilled during bailouts.
  union RegisterContent {
    float s;
    double d;
  };

  using SetType = uint32_t;

  static const char* GetName(Code) { MOZ_CRASH(); }
  static Code FromName(const char*) { MOZ_CRASH(); }

  static const uint32_t Total = 16;
  static const uint32_t TotalPhys = Total;
  static const uint32_t Allocatable =
      Total - 2;  // Without f7 and d7, they are scratch registers.

  static_assert(sizeof(SetType) * 8 >= Total,
                "SetType should be large enough to enumerate all registers.");

  static const SetType AllMask = ((SetType(1) << TotalPhys) - 1);

  static const SetType AllSingleMask =
      (SetType(1) << FPRegisterID::f0) | (SetType(1) << FPRegisterID::f1) |
      (SetType(1) << FPRegisterID::f2) | (SetType(1) << FPRegisterID::f3) |
      (SetType(1) << FPRegisterID::f4) | (SetType(1) << FPRegisterID::f5) |
      (SetType(1) << FPRegisterID::f6) | (SetType(1) << FPRegisterID::f7);

  static const SetType AllDoubleMask =
      (SetType(1) << FPRegisterID::d0) | (SetType(1) << FPRegisterID::d1) |
      (SetType(1) << FPRegisterID::d2) | (SetType(1) << FPRegisterID::d3) |
      (SetType(1) << FPRegisterID::d4) | (SetType(1) << FPRegisterID::d5) |
      (SetType(1) << FPRegisterID::d6) | (SetType(1) << FPRegisterID::d7);

  static const SetType NonAllocatableMask =
      (1 << FPRegisterID::f7) | (1 << FPRegisterID::d7);

  static const SetType VolatileMask = AllMask;
  static const SetType NonVolatileMask = AllMask & ~VolatileMask;
  static const SetType AllocatableMask = AllMask & ~NonAllocatableMask;

  static const Code Invalid = invalid_reg;

  static bool IsLocal(uint8_t id) {
    return id >= FloatRegisters::first_local_id;
  }

  static uint32_t ToLocalIndex(uint8_t id) {
    MOZ_RELEASE_ASSERT(IsLocal(id));
    return id - FloatRegisters::first_local_id;
  }

  static bool IsGlobal(uint8_t id) {
    return id < FloatRegisters::first_local_id;
  }

  static bool Is64Bit(uint8_t id) {
    return id >= FloatRegisters::first_double_reg_id;
  }
};

template <typename T>
class TypedRegisterSet;

struct FloatRegister {
  using Codes = FloatRegisters;
  using Code = size_t;
  using Encoding = Codes::Encoding;
  using SetType = Codes::SetType;

  constexpr FloatRegister() : reg_(Codes::Encoding::invalid_reg) {}

  constexpr FloatRegister(Codes::Encoding r) : reg_(r) {}

  static uint32_t FirstBit(SetType set) {
    return mozilla::CountTrailingZeroes32(set);
  }

  static uint32_t LastBit(SetType set) {
    return 31 - mozilla::CountLeadingZeroes32(set);
  }

  static FloatRegister FromCode(uint32_t i) {
    MOZ_ASSERT(i < Codes::Total);
    return FloatRegister(static_cast<Codes::Encoding>(i & 15));
  }

  bool isSingle() const { return reg_ < Codes::Encoding::first_double_reg_id; }

  bool isDouble() const {
    return reg_ >= Codes::Encoding::first_double_reg_id &&
           reg_ != reg_ < Codes::Encoding::invalid_reg;
  }

  bool isSimd128() const { return false; }

  FloatRegister asSingle() const {
    MOZ_ASSERT(isSingle());
    return *this;
  }

  FloatRegister asDouble() const {
    MOZ_ASSERT(isDouble());
    return *this;
  }

  FloatRegister asSimd128() const {
    return FloatRegister(Codes::Encoding::invalid_reg);
  }

  bool isInvalid() const { return reg_ == Codes::Encoding::invalid_reg; }

  Encoding encoding() const {
    MOZ_ASSERT(!isInvalid());
    MOZ_ASSERT(uint32_t(reg_) < Codes::TotalPhys);
    return reg_;
  }

  Code code() const { return Code(encoding()); }

  const char* name() const { MOZ_CRASH(); }

  bool volatile_() const {
    return !!((SetType(1) << code()) & FloatRegisters::VolatileMask);
  }

  bool operator!=(FloatRegister other) const { return other.reg_ != reg_; }

  bool operator==(FloatRegister other) const { return other.reg_ == reg_; }

  bool aliases(FloatRegister other) const { return other.reg_ == reg_; }

  uint32_t numAliased() const { return 1; }

  uint32_t numAlignedAliased() const { return numAliased(); }

  FloatRegister aliased(uint32_t aliasIdx) const {
    MOZ_ASSERT(aliasIdx == 0);
    return *this;
  }

  bool equiv(FloatRegister other) const { return other.code() == code(); }

  uint32_t size() const {
    MOZ_ASSERT(!isInvalid());
    if (isSingle()) {
      return sizeof(float);
    }
    if (isDouble()) {
      return sizeof(double);
    }

    MOZ_CRASH("Unknown float register type");
    return 0;
  }

  FloatRegister alignedAliased(uint32_t aliasIdx) const {
    return aliased(aliasIdx);
  }

  SetType alignedOrDominatedAliasedSet() const { return SetType(1) << code(); }

  static constexpr RegTypeName DefaultType = RegTypeName::Float64;

  template <RegTypeName = DefaultType>
  static SetType LiveAsIndexableSet(SetType s) {
    return SetType(0);
  }

  template <RegTypeName Name = DefaultType>
  static SetType AllocatableAsIndexableSet(SetType s) {
    static_assert(Name != RegTypeName::Any, "Allocatable set are not iterable");
    return LiveAsIndexableSet<Name>(s);
  }

  static TypedRegisterSet<FloatRegister> ReduceSetForPush(
      const TypedRegisterSet<FloatRegister>& s);

  uint32_t getRegisterDumpOffsetInBytes() { MOZ_CRASH(); }
  static uint32_t SetSize(SetType x) { return mozilla::CountPopulation32(x); }

  static Code FromName(const char* name) { MOZ_CRASH(); }

  static uint32_t GetPushSizeInBytes(const TypedRegisterSet<FloatRegister>&) {
    MOZ_CRASH();
    return 0;
  }

 private:
  Codes::Encoding reg_;
};

template <>
inline FloatRegister::SetType
FloatRegister::LiveAsIndexableSet<RegTypeName::Any>(SetType set) {
  return set;
}

inline bool hasUnaliasedDouble() {
  return true;
}  // All float registers are distinct.
inline bool hasMultiAlias() { return false; }

static const uint32_t ShadowStackSpace = 0;
static const uint32_t JumpImmediateRange = INT32_MAX;

#ifdef JS_NUNBOX32
static const int32_t NUNBOX32_TYPE_OFFSET = 4;
static const int32_t NUNBOX32_PAYLOAD_OFFSET = 0;
#endif

}  // namespace js::jit

#endif /* jit_wasm32_Architecture_wasm32_h */
