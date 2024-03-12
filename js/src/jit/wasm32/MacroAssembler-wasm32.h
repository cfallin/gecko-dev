/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_MacroAssembler_wasm32_h
#define jit_wasm32_MacroAssembler_wasm32_h

#include "mozilla/Maybe.h"

#include <algorithm>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "jit/JitOptions.h"
#include "jit/wasm32/Assembler-wasm32.h"

using js::wasm::FaultingCodeOffsetPair;

namespace {

// Primary template for function types
template <typename Func>
struct FunctionTraits;

// Specialization for function pointers
template <typename Ret, typename... Args>
struct FunctionTraits<Ret (*)(Args...)> {
  using ReturnType = Ret;
  using ArgTypes = std::tuple<Args...>;
};

}  // namespace

namespace js::jit {

class CompactBufferReader;

// See documentation for ScratchTagScope and ScratchTagScopeRelease in
// MacroAssembler-x64.h.

class ScratchTagScope : public ScratchRegisterScope {
 public:
  ScratchTagScope(MacroAssembler& masm, const ValueOperand&)
      : ScratchRegisterScope(masm) {}
};

class ScratchTagScopeRelease {
  ScratchTagScope* ts_;

 public:
  explicit ScratchTagScopeRelease(ScratchTagScope* ts) : ts_(ts) {
    ts_->release();
  }
  ~ScratchTagScopeRelease() { ts_->reacquire(); }
};

struct SwitchInfo {
  // These two data structures are for builing a proper CFG for table switch.
  // The second could be derived from the first one, but we held it for
  // convinience.
  std::map<wasm32::BasicBlock*, std::vector<Label*>>
      switchDispatchBlockToCaseLabels_;
  std::map<Label*, wasm32::BasicBlock*> labelToSwitchHead_;

  void recordLabelForSwitchBlock(Label* label, wasm32::BasicBlock* block);
  bool contains(Label* label) const;
  wasm32::BasicBlock* switchBlockByLabel(Label* label);
  void clear();
};

class MacroAssemblerWasm32 : public Assembler {
 public:
  constexpr static uint32_t STACK_SLOT_SIZE = sizeof(void*);

  // Perform a downcast. Should be removed by Bug 996602.
  MacroAssembler& asMasm();
  const MacroAssembler& asMasm() const;

  MacroAssemblerWasm32() { resetCurrentCFG(); }

  size_t size() const { return Assembler::size(); }

  size_t bytesNeeded() const { return size(); }

  size_t jumpRelocationTableBytes() const { return 0u; }

  size_t dataRelocationTableBytes() const { return 0u; }

  size_t numCodeLabels() const { MOZ_CRASH(); }
  CodeLabel codeLabel(size_t) { MOZ_CRASH(); }

  bool reserve(size_t size) { MOZ_CRASH(); }
  bool appendRawCode(const uint8_t* code, size_t numBytes) { MOZ_CRASH(); }
  bool swapBuffer(wasm::Bytes& bytes) { MOZ_CRASH(); }

  void assertNoGCThings() const { MOZ_CRASH(); }

  static void TraceJumpRelocations(JSTracer*, JitCode*, CompactBufferReader&) {
    MOZ_CRASH();
  }
  static void TraceDataRelocations(JSTracer*, JitCode*, CompactBufferReader&) {
    MOZ_CRASH();
  }

  static bool SupportsFloatingPoint() { return true; }
  static bool SupportsUnalignedAccesses() { return false; }
  static bool SupportsFastUnalignedFPAccesses() { return false; }

  void executableCopy(void* buffer) { Assembler::executableCopy(buffer); }

  void copyJumpRelocationTable(uint8_t*) {}

  void copyDataRelocationTable(uint8_t*) {}

  void copyPreBarrierTable(uint8_t*) {}

  void processCodeLabels(uint8_t*) {}

  void flushBuffer();

  void bind(Label* label);

  void bind(CodeLabel* label) { MOZ_CRASH(); }

  template <typename T>
  void j(Condition, T) {
    MOZ_CRASH();
  }

  void j(Label* target);

  void jump(Label* label);

  void jump(JitCode* code) { MOZ_CRASH(); }

  void jump(Register reg);

  void jump(const Address& address);

  void jump(ImmPtr ptr);

  void jump(TrampolinePtr code) {
    jump(ImmPtr(code.value, ImmPtr::NoCheckToken()));
  }

  void callPreBarrier(TrampolinePtr ptr);

  void writeCodePointer(CodeLabel* label);

  void haltingAlign(size_t);

  void nopAlign(size_t);
  void checkStackAlignment();

  uint32_t currentOffset() { return Assembler::size(); }

  void nop();

  void breakpoint();
  void unreachable();

  void abiret();
  void ret();

  CodeOffset toggledJump(Label*);
  CodeOffset toggledCall(JitCode*, bool);
  static size_t ToggledCallSize(uint8_t*);

  void finish();

  template <typename T, typename S>
  void moveValue(T, S) {
    MOZ_CRASH();
  }

  template <typename T, typename S, typename U>
  void moveValue(T, S, U) {
    MOZ_CRASH();
  }

  void storeValue(ValueOperand val, const Address& dest);

  void storeValue(ValueOperand val, const BaseIndex& dest);

  void storeValue(JSValueType type, Register payload, const Address& dest);

  void storeValue(JSValueType type, Register reg, BaseIndex dest) {
    MOZ_CRASH();
  }

  void storeValue(const Value& val, Address dest);

  void storeValue(const Value& val, BaseIndex dest) { MOZ_CRASH(); }

  void storeValue(const Address& src, const Address& dest, Register temp);

  template <typename T, typename S>
  void storePrivateValue(const T&, const S&) {
    MOZ_CRASH();
  }

  void loadValue(Address src, ValueOperand val);

  void loadValue(const BaseIndex& src, ValueOperand val);

  template <typename T, typename S>
  void loadUnalignedValue(T, S) {
    MOZ_CRASH();
  }

  void pushValue(const Address& addr);

  void pushValue(ValueOperand val);

  void pushValue(const Value& val);

  void pushValue(JSValueType type, Register payload);

  void pushValue(const BaseIndex& addr, Register);

  void popValue(ValueOperand val);

  void tagValue(JSValueType type, Register payload, ValueOperand dest);
  void retn(Imm32 n);

  void push(const Address& src);
  void push(Register src);
  void push64(Register src);
  void Push64(Register src);
  void push(Imm32 imm);
  void push(ImmPtr imm);
  void push(ImmGCPtr imm);
  void push(ImmWord imm);
  void push(FloatRegister fr);

  template <typename T>
  void Push(T) {
    MOZ_CRASH();
  }

  template <typename T>
  void pop(T) {
    MOZ_CRASH();
  }

  void pop(Register reg);
  void pop64(Register reg);
  void Pop64(Register reg);
  void pop(FloatRegister reg);
  void pop(const ValueOperand& v);
  void PopStackSlots(const std::size_t numSlots);

  template <typename T>
  CodeOffset pushWithPatch(T) {
    MOZ_CRASH();
  }

  void testNullSet(Condition cond, const ValueOperand& value, Register dest);

  void testObjectSet(Condition cond, const ValueOperand& value, Register dest);
  void testUndefinedSet(Assembler::Condition cond, const ValueOperand& value,
                        Register dest);

  template <typename T, typename S>
  void cmpPtrSet(Condition, T, S, Register) {
    MOZ_CRASH();
  }

  template <typename T>
  void mov(T, Register) {
    MOZ_CRASH();
  }

  void mov(Register src, Register dst) { movePtr(src, dst); }

  void movePtr(ImmWord imm, Register dest) { MOZ_CRASH(); }
  void movePtr(wasm::SymbolicAddress imm, Register dest) { MOZ_CRASH(); }

  void movePtr(ImmGCPtr imm, Register dest);

  void movePtr(ImmPtr ptr, Register dst);

  void movePtr(Register src, Register dst);

  void move32(Imm32 imm, Register dest);

  void move32(Register src, Register dest);

  template <typename T, typename S>
  void moveFloat32(T, S) {
    MOZ_CRASH();
  }

  void moveDouble(FloatRegister src, FloatRegister dst);

  template <typename T, typename S>
  void move64(const T& src, const S& dst) {
    wasmGetValue64(src);
    register_set64(dst);
  }

  template <typename T>
  CodeOffset movWithPatch(T, Register) {
    MOZ_CRASH();
  }

  void loadPtr(AbsoluteAddress address, Register dest) {
    load32(address, dest);
  }
  void loadPtr(wasm::SymbolicAddress address, Register dest) { MOZ_CRASH(); }

  FaultingCodeOffset loadPtr(const Address& address, Register dest) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    load32(address, dest);
    return fco;
  }

  FaultingCodeOffset loadPtr(const BaseIndex& address, Register dest) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    load32(address, dest);
    return fco;
  }

  template <class SrcType>
  FaultingCodeOffset load32(const SrcType& src, Register dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmGetValue32(src);
    register_set32(dst);
    return fco;
  }

  template <typename S>
  void load32Unaligned(const S& src, Register dest) {
    load32(src, dest);
  }

  template <typename AddrType>
  FaultingCodeOffset loadFloat32(const AddrType& addr, FloatRegister dest) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(addr);
    register_getf32(dest);
    f32_store();
    return fco;
  }

  template <typename AddrType>
  FaultingCodeOffset loadDouble(const AddrType& addr, FloatRegister dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(addr);
    f64_load();
    register_setf64(dst);
    return fco;
  }

  void loadPrivate(const Address& src, Register dst) { loadPtr(src, dst); }

  template <class SrcType>
  FaultingCodeOffset load8SignExtend(const SrcType& src, Register dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(src);
    i32_load8_s();
    register_set32(dst);
    return fco;
  }

  template <class SrcType>
  FaultingCodeOffset load8ZeroExtend(const SrcType& src, Register dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(src);
    i32_load8_u();
    register_set32(dst);
    return fco;
  }

  template <class SrcType>
  FaultingCodeOffset load16SignExtend(const SrcType& src, Register dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(src);
    i32_load16_s();
    register_set32(dst);
    return fco;
  }

  template <class SrcType>
  void load16UnalignedSignExtend(const SrcType& src, Register dst) {
    load16SignExtend(src, dst);
  }

  template <class SrcType>
  FaultingCodeOffset load16ZeroExtend(const SrcType& src, Register dst) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(src);
    i32_load16_u();
    register_set32(dst);
    return fco;
  }

  template <class SrcType>
  void load16UnalignedZeroExtend(const SrcType& src, Register dst) {
    load16ZeroExtend(src, dst);
  }

  template <class SrcType>
  FaultingCodeOffsetPair load64(const SrcType& src, Register64 dst) {
    FaultingCodeOffset fco1 = FaultingCodeOffset(currentOffset());
    FaultingCodeOffset fco2 = FaultingCodeOffset(currentOffset());
    wasmGetValue64(src);
    register_set64(dst.low);
    return FaultingCodeOffsetPair(fco1, fco2);
  }

  template <typename SrcType>
  void load64Unaligned(const SrcType& src, Register64 dst) {
    load64(src, dst);
  }

  void zeroDouble(FloatRegister dst);

  FaultingCodeOffset storePtr(Register src, const Address& address) {
    return store32(src, address);
  }

  FaultingCodeOffset storePtr(Register src, const BaseIndex& address) {
    return store32(src, address);
  }

  void storePtr(ImmPtr src, const Address& address) {
    store32(Imm32(reinterpret_cast<int32_t>(src.value)), address);
  }
  void storePtr(ImmWord src, const Address& address) {
    store32(Imm32(static_cast<int32_t>(src.value)), address);
  }
  void storePtr(ImmGCPtr src, const Address& address);
  void storePtr(Register src, AbsoluteAddress address) {
    store32(src, address);
  }

  template <typename SrcType, typename AddrType>
  FaultingCodeOffset store32(SrcType src, const AddrType& address) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(address);
    wasmGetValue32(src);
    i32_store();
    return fco;
  }

  template <typename S, typename T>
  void store32Unaligned(const S& src, const T& dst) {
    store32(src, dst);
  }

  template <typename SrcType, typename AddrType>
  FaultingCodeOffset store8(SrcType src, const AddrType& address) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(address);
    wasmGetValue32(src);
    i32_store8();
    return fco;
  }

  template <typename SrcType, typename AddrType>
  FaultingCodeOffset store16(SrcType src, const AddrType& address) {
    FaultingCodeOffset fco = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(address);
    wasmGetValue32(src);
    i32_store16();
    return fco;
  }

  template <typename S, typename T>
  void store16Unaligned(const S& src, const T& dest) {
    store16(src, dest);
  }

  template <typename T>
  FaultingCodeOffsetPair store64(Register64 src, const T& dst) {
    FaultingCodeOffset fco1 = FaultingCodeOffset(currentOffset());
    FaultingCodeOffset fco2 = FaultingCodeOffset(currentOffset());
    wasmEvalOperand32(dst);
    register_get64(src.low);
    i64_store();
    return FaultingCodeOffsetPair(fco1, fco2);
  }

  void store64(Imm64 src, const Address& dst) {
    wasmEvalOperand32(dst);
    i64_const(src.value);
    i64_store();
  }

  template <typename T>
  void store64Unaligned(Register64 src, const T& dst) {
    store64(src, dst);
  }

  void computeEffectiveAddress(const Address& address, Register dest);

  void computeEffectiveAddress(const BaseIndex& address, Register dest);

  void splitTag(Register src, Register dest) {
    extractTypeFromi64(src);
    register_set32(dest);
  }

  void splitTagForTest(const ValueOperand& value, ScratchTagScope& tag) {
    splitTag(value.scratchReg(), tag);
  }

  void boxDouble(FloatRegister src, const ValueOperand& dest, FloatRegister) {
    register_getf64(src);
    i64_reinterpret_f64();
    register_set64(dest.scratchReg());
  }

  void boxNonDouble(JSValueType type, Register src, const ValueOperand& dest) {
    register_get32(src);
    i64_extend_i32_u();
    i64_const(static_cast<uint64_t>(ImmType(type).value) << 32);
    i64_or();
    register_set64(dest.scratchReg());
  }

  template <typename T>
  void unboxInt32(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_INT32);
  }

  template <typename T>
  void unboxBoolean(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_BOOLEAN);
  }

  template <typename T>
  void unboxString(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_STRING);
  }

  template <typename T>
  void unboxSymbol(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_SYMBOL);
  }

  template <typename T>
  void unboxBigInt(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_BIGINT);
  }

  template <typename T>
  void unboxObject(const T& src, Register dst) {
    unboxNonDouble(src, dst, JSVAL_TYPE_OBJECT);
  }

  template <typename T>
  void unboxDouble(T, FloatRegister) {
    MOZ_CRASH();
  }

  void unboxDouble(const Address& src, FloatRegister dest) {
    wasmEvalOperand32(src);
    f64_load();
    register_setf64(dest);
  }

  void unboxDouble(const ValueOperand& src, FloatRegister dest) {
    register_get64(src.scratchReg());
    f64_reinterpret_i64();
    register_setf64(dest);
  }

  void unboxValue(const ValueOperand&, AnyRegister, JSValueType) {
    MOZ_CRASH();
  }

  template <typename T>
  void unboxNonDouble(const T& src, Register dest, JSValueType type) {
    MOZ_RELEASE_ASSERT(type != JSVAL_TYPE_DOUBLE);
    MOZ_ASSERT(!JitOptions.spectreValueMasking);

    extractPayloadHelper32(src);
    register_set32(dest);
  }

  void unboxGCThingForGCBarrier(const Address& src, Register dest) {
    load32(ToPayload(src), dest);
  }

  template <typename T>
  void unboxWasmAnyRefGCThingForGCBarrier(const T&, Register) {
    MOZ_CRASH();
  }

  void getWasmAnyRefGCThingChunk(Register, Register) { MOZ_CRASH(); }

  template <typename T>
  void unboxObjectOrNull(const T& src, Register dest) {
    MOZ_CRASH();
  }

  void notBoolean(const ValueOperand& val);

  [[nodiscard]] Register extractObject(Address, Register) { MOZ_CRASH(); }
  [[nodiscard]] Register extractObject(ValueOperand, Register) { MOZ_CRASH(); }
  [[nodiscard]] Register extractInt32(ValueOperand, Register) { MOZ_CRASH(); }

  template <typename T>
  [[nodiscard]] Register extractTag(const T& src, Register dst) {
    extractTagHelper32(src);
    register_set32(dst);
    return dst;
  }

  void convertFloat32ToInt32(FloatRegister, Register, Label*, bool v = true) {
    MOZ_CRASH();
  }

  void convertDoubleToInt32(FloatRegister, Register, Label*, bool v = true) {
    wasmPrintMsg("convertDoubleToInt32 freg, reg, lbl NYI");
    unreachable();
  }

  void convertDoubleToPtr(FloatRegister, Register, Label*, bool v = true) {
    MOZ_CRASH();
  }

  void convertBoolToInt32(Register source, Register dest);
  void convertDoubleToFloat32(FloatRegister, FloatRegister) { MOZ_CRASH(); }
  void convertInt32ToFloat32(Register, FloatRegister) { MOZ_CRASH(); }

  void convertInt32ToDouble(Register src, FloatRegister dest) {
    register_get32(src);
    f64_convert_i32_s();
    register_setf64(dest);
  }

  void convertInt32ToDouble(const Address& src, FloatRegister dest) {
    wasmPrintMsg("convertInt32ToDouble addr, freg NYI");
    unreachable_instr();
  }

  void convertInt32ToDouble(const BaseIndex& src, FloatRegister dest) {
    MOZ_CRASH();
  }

  void convertInt32ToDouble(const Operand& src, FloatRegister dest) {
    MOZ_CRASH();
  }

  void convertFloat32ToDouble(FloatRegister, FloatRegister) { MOZ_CRASH(); }

  void boolValueToDouble(ValueOperand, FloatRegister) { MOZ_CRASH(); }
  void boolValueToFloat32(ValueOperand, FloatRegister) { MOZ_CRASH(); }
  void int32ValueToDouble(ValueOperand, FloatRegister) { MOZ_CRASH(); }
  void int32ValueToFloat32(ValueOperand, FloatRegister) { MOZ_CRASH(); }

  void loadConstantDouble(double d, FloatRegister dst);
  void loadConstantFloat32(float, FloatRegister) { MOZ_CRASH(); }
  Condition testInt32Truthy(bool, ValueOperand) { MOZ_CRASH(); }
  Condition testStringTruthy(bool, ValueOperand) { MOZ_CRASH(); }
  Condition testBigIntTruthy(bool, ValueOperand) { MOZ_CRASH(); }

  template <typename T>
  void loadUnboxedValue(T, MIRType, AnyRegister) {
    MOZ_CRASH();
  }
  template <typename T>
  void storeUnboxedPayload(ValueOperand value, T, size_t, JSValueType) {
    MOZ_CRASH();
  }

  void convertUInt32ToDouble(Register, FloatRegister) { MOZ_CRASH(); }
  void convertUInt32ToFloat32(Register, FloatRegister) { MOZ_CRASH(); }
  void incrementInt32Value(const Address& addr);
  void ensureDouble(const ValueOperand& source, FloatRegister dest,
                    Label* failure);
  void handleFailureWithHandlerTail(Label* profilerExitTail,
                                    Label* bailoutTail) {
    MOZ_CRASH();
  }

  void buildFakeExitFrame(Register, uint32_t*) { MOZ_CRASH(); }
  bool buildOOLFakeExitFrame(void*) { MOZ_CRASH(); }

  void setPrinter(Sprinter*) { MOZ_CRASH(); }
  Operand ToPayload(Operand base) { MOZ_CRASH(); }
  Address ToPayload(const Address& address) const;
  BaseIndex ToPayload(const BaseIndex& address) const;
  BaseValueIndex ToPayload(const BaseValueIndex& address) const;

  Register getStackPointer() const { return StackPointer; }

  // Instrumentation for entering and leaving the profiler.
  void profilerEnterFrame(Register framePtr, Register scratch);
  void profilerExitFrame();

  Address ToType(const Address& address);
  BaseIndex ToType(const BaseIndex& address);

  void wasmSub32(Register src, Imm32 offset, Register dst);
  void wasmPushI32(Register reg);
  void wasmPushI32(const Address& address);
  void wasmPushAddress(const Address& address);
  void wasmPopI32(Register reg);
  void wasmPopI64(Register reg);
  void wasmPushI64(Register reg);
  void wasmPushF32(FloatRegister reg);
  void wasmPushF64(FloatRegister reg);
  void wasmPushF64(const Address& address);
  void wasmPopF64(FloatRegister reg);
  void wasmMovLocal32ToReg(uint32_t localIndex, Register reg);
  void wasmPrepareCall();
  void lea(const BaseIndex& src, Register dst);
  void lshift32(Register src, Register dst, Register shift);
  void truncateDoubleToInt32(FloatRegister src, Register dst, Label* fail);
  void branchDoubleNegativeZero(FloatRegister src, Label* fail,
                                bool maybeNonZero = true);
  void alignSP(uint32_t alignment);
  void recordLabelForSwitch(Label* caseLabel);
  void setSignatureForRuntimeCall(wasm32::SignatureIndex signatureIndex);

  template <typename Sig>
  constexpr wasm32::SignatureIndex getSignatureForRuntimeCall() {
    using FunctionSignature = FunctionTraits<Sig>;
    using ResultType = typename FunctionSignature::ReturnType;
    using ParametersTypes = typename FunctionSignature::ArgTypes;

    if constexpr (std::is_same_v<ResultType, void>) {
      if constexpr (std::tuple_size_v<ParametersTypes> == 1) {
        if constexpr (std::is_same_v<std::tuple_element_t<0, ParametersTypes>,
                                     uint64_t>) {
          return wasm32::SignatureIndex::SIGNATURE_1_i64_TO_VOID;
        } else if constexpr (std::is_same_v<
                                 std::tuple_element_t<0, ParametersTypes>,
                                 double>) {
          return wasm32::SignatureIndex::SIGNATURE_1_F64_TO_VOID;
        }
      } else if constexpr (std::tuple_size_v<ParametersTypes> == 2) {
        return wasm32::SignatureIndex::SIGNATURE_2_I32_TO_VOID;
      } else if constexpr (std::tuple_size_v<ParametersTypes> == 3) {
        return wasm32::SignatureIndex::SIGNATURE_3_I32_TO_VOID;
      }
      return wasm32::SignatureIndex::SIGNATURE_8_I32_TO_VOID;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 1) {
      if (std::is_same_v<std::tuple_element_t<0, ParametersTypes>, double>) {
        return wasm32::SignatureIndex::SIGNATURE_1_F64_TO_1_I32;
      }
      return wasm32::SignatureIndex::SIGNATURE_1_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 2) {
      if constexpr (std::is_same_v<std::tuple_element_t<0, ParametersTypes>,
                                   double> &&
                    std::is_same_v<std::tuple_element_t<1, ParametersTypes>,
                                   double> &&
                    std::is_same_v<ResultType, double>) {
        return wasm32::SignatureIndex::SIGNATURE_2_F64_TO_1_F64;
      } else if constexpr (std::is_same_v<
                               std::tuple_element_t<1, ParametersTypes>,
                               double>) {
        return wasm32::SignatureIndex::SIGNATURE_1_I32_1_F64_TO_1_I32;
      }
      return wasm32::SignatureIndex::SIGNATURE_2_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 3) {
      if constexpr (std::is_same_v<ResultType, double>) {
        return wasm32::SignatureIndex::SIGNATURE_3_F64_TO_1_F64;
      }
      return wasm32::SignatureIndex::SIGNATURE_3_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 4) {
      if constexpr (std::is_same_v<ResultType, double>) {
        return wasm32::SignatureIndex::SIGNATURE_4_F64_TO_1_F64;
      } else if constexpr (std::is_same_v<ResultType, uint64_t>) {
        return wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64;
      }
      return wasm32::SignatureIndex::SIGNATURE_4_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 5) {
      if constexpr (std::is_same_v<ResultType, uint64_t>) {
        return wasm32::SignatureIndex::SIGNATURE_1_i32_4_I64_TO_I64;
      }
      return wasm32::SignatureIndex::SIGNATURE_5_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 6) {
      return wasm32::SignatureIndex::SIGNATURE_6_I32_TO_1_I32;
    } else if constexpr (std::tuple_size_v<ParametersTypes> == 7) {
      return wasm32::SignatureIndex::SIGNATURE_7_I32_TO_1_I32;
    }

    return wasm32::SignatureIndex::SIGNATURE_VOID_TO_VOID;
  }

  void wasmCallByIndexInLocal(uint32_t localIndex, uint32_t typeIndex);
  void wasmPushLocal(uint32_t localIndex);
  void wasmCallByIndex(uint32_t index, uint32_t typeIndex);

  // Debug instructions.
  void wasmPrintMsg(const char* message);
  void wasmPrintReg32(Register reg);
  void wasmPrintReg64(Register reg);
  void wasmPrintRegF64(FloatRegister reg);
  void wasmPrintMemory(Register base, uint32_t numValues);

  void fillPendingsBranches();

 protected:
  enum class BinOperation {
    PLUS,
    SUB,
  };

  template <typename T>
  void branch32Overflow(BinOperation op, T src, Register dest, Label* label) {
    wasmGetValue32(dest);
    wasmGetValue32(src);
    switch (op) {
      case BinOperation::PLUS:
        i32_add();
        break;
      case BinOperation::SUB:
        i32_sub();
        break;
    }
    i64_extend_i32_s();

    wasmGetValue32(dest);
    i64_extend_i32_s();

    wasmGetValue32(src);
    i64_extend_i32_s();

    switch (op) {
      case BinOperation::PLUS:
        i64_add();
        break;
      case BinOperation::SUB:
        i64_sub();
        break;
    }

    i64_neq();
    j(label);

    wasmGetValue32(dest);
    wasmGetValue32(src);
    switch (op) {
      case BinOperation::PLUS:
        i32_add();
        break;
      case BinOperation::SUB:
        i32_sub();
        break;
    }
    register_set32(dest);
  }

  void extractTypeFromi64(const ValueOperand& value);
  void extractTypeFromi64(Register src);
  void extractPayloadFromi64(const ValueOperand& value);

  template <typename T>
  void wasmEvalOperand32(T operand) {
    if constexpr (std::is_same_v<Register, T>) {
      register_get32(operand);
    } else if constexpr (std::is_same_v<Imm32, T>) {
      i32_const(operand.value);
    } else if constexpr (std::is_same_v<ImmPtr, T>) {
      i32_const(reinterpret_cast<int32_t>(operand.value));
    } else if constexpr (std::is_same_v<Address, T>) {
      register_get32(operand.base);
      i32_const(operand.offset);
      i32_add();
    } else if constexpr (std::is_same_v<BaseIndex, T> ||
                         std::is_same_v<BaseValueIndex, T>) {
      register_get32(operand.base);
      register_get32(operand.index);
      i32_const(ScaleToShift(operand.scale));
      i32_shl();
      i32_add();
      i32_const(operand.offset);
      i32_add();
    } else if constexpr (std::is_same_v<AbsoluteAddress, T>) {
      i32_const(reinterpret_cast<int32_t>(operand.addr));
    } else {
      MOZ_CRASH();
    }
  }

  template <typename T>
  void wasmEvalOperand64(T operand) {
    if constexpr (std::is_same_v<Register, T>) {
      register_get64(operand);
    } else if constexpr (std::is_same_v<Imm32, T>) {
      i64_const(operand.value);
    } else if constexpr (std::is_same_v<ImmPtr, T>) {
      i64_const(reinterpret_cast<int32_t>(operand.value));
    } else if constexpr (std::is_same_v<Address, T>) {
      register_get32(operand.base);
      i32_const(operand.offset);
      i32_add();
    } else if constexpr (std::is_same_v<BaseIndex, T> ||
                         std::is_same_v<BaseValueIndex, T>) {
      register_get32(operand.base);
      register_get32(operand.index);
      i32_const(ScaleToShift(operand.scale));
      i32_shl();
      i32_add();
      i32_const(operand.offset);
      i32_add();
    } else if constexpr (std::is_same_v<AbsoluteAddress, T>) {
      i32_const(reinterpret_cast<int32_t>(operand.addr));
    } else {
      MOZ_CRASH();
    }
  }

  template <typename T>
  void wasmGetValue32(T operand) {
    wasmEvalOperand32(operand);
    if constexpr (std::is_same_v<Address, T> || std::is_same_v<BaseIndex, T> ||
                  std::is_same_v<BaseValueIndex, T> ||
                  std::is_same_v<AbsoluteAddress, T>) {
      i32_load();
    }
  }

  template <typename T>
  void wasmGetValue64(T operand) {
    wasmEvalOperand64(operand);
    if constexpr (std::is_same_v<Address, T> || std::is_same_v<BaseIndex, T> ||
                  std::is_same_v<BaseValueIndex, T> ||
                  std::is_same_v<AbsoluteAddress, T>) {
      i64_load();
    }
  }

  template <typename T>
  void branchTagNonDouble(JSValueTag tag, const T& operand, Condition cond,
                          Label* label) {
    MOZ_RELEASE_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

    extractTagHelper32(operand);
    i32_const(ImmTag(tag).value);
    emitCondition32(cond);
    j(label);
  }

  template <typename T>
  void extractTagHelper32(const T& operand) {
    if constexpr (std::is_same_v<Address, T> || std::is_same_v<BaseIndex, T> ||
                  std::is_same_v<AbsoluteAddress, T>) {
      wasmGetValue32(ToType(operand));
    } else if constexpr (std::is_same_v<Register, T>) {
      wasmGetValue32(operand);
    } else if constexpr (std::is_same_v<ValueOperand, T>) {
      extractTypeFromi64(operand);
    } else {
      MOZ_CRASH();
    }
  }

  template <typename T>
  void extractPayloadHelper32(const T& operand) {
    if constexpr (std::is_same_v<Address, T> || std::is_same_v<BaseIndex, T> ||
                  std::is_same_v<BaseValueIndex, T> ||
                  std::is_same_v<AbsoluteAddress, T>) {
      wasmGetValue32(ToPayload(operand));
    } else if constexpr (std::is_same_v<Register, T>) {
      wasmGetValue32(operand);
    } else if constexpr (std::is_same_v<ValueOperand, T>) {
      extractPayloadFromi64(operand);
    } else {
      MOZ_CRASH();
    }
  }

  template <typename LhsType, typename RhsType>
  void branch32Impl(Condition cond, const LhsType& lhs, const RhsType& rhs,
                    Label* label) {
    wasmGetValue32(lhs);
    wasmGetValue32(rhs);
    emitCondition32(cond);
    j(label);
  }

  template <typename T>
  void fallibleUnboxPtrImpl(const T& src, Register dest, JSValueType type,
                            Label* fail);

  void testSetHelper(Condition cond, JSValueTag tag, const ValueOperand& value,
                     Register dest);

  template <typename T>
  void cmp32Move32Impl(Condition cond, Register lhs, const T& rhs, Register src,
                       Register dest) {
    wasmGetValue32(src);   // select true case
    wasmGetValue32(dest);  // select false case

    wasmGetValue32(lhs);
    wasmGetValue32(rhs);
    emitCondition32(cond);
    select_instr();

    register_set32(dest);
  }

  // Used for callABI to pass signature index.
  mozilla::Maybe<wasm32::SignatureIndex> lastRuntimeCallFunctionSignatureIndex_;
  SwitchInfo switchInfo_;
};

typedef MacroAssemblerWasm32 MacroAssemblerSpecific;

static inline bool GetTempRegForIntArg(uint32_t, uint32_t, Register*) {
  MOZ_CRASH();
}

}  // namespace js::jit

#endif /* jit_wasm32_MacroAssembler_wasm32_h */
