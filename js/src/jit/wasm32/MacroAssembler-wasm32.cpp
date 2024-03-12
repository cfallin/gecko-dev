/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/MacroAssembler-wasm32.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>
#include <stack>
#include <string>

#include "jsmath.h"
#include "jit/JitRuntime.h"
#include "jit/MacroAssembler.h"
#include "vm/JitActivation.h"  // jit::JitActivation

#include "jit/MacroAssembler-inl.h"

namespace js::jit {

constexpr int32_t PAYLOAD_OFFSET = NUNBOX32_PAYLOAD_OFFSET;
constexpr int32_t TAG_OFFSET = NUNBOX32_TYPE_OFFSET;

namespace {

bool hasReturnValue(wasm32::SignatureIndex index) {
  if (index == wasm32::SignatureIndex::SIGNATURE_VOID_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_8_I32_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_1_I32_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_2_I32_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_1_i64_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_3_I32_TO_VOID ||
      index == wasm32::SignatureIndex::SIGNATURE_1_F64_TO_VOID) {
    return false;
  }

  return true;
}

}  // namespace

// Note: this function clobbers the input register.
void MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output) {
  MOZ_CRASH();
}

void MacroAssembler::subFromStackPtr(Imm32 imm32) {
  register_get32(StackPointer);
  i32_const(imm32.value);
  i32_sub();
  register_set32(StackPointer);
}

//{{{ check_macroassembler_style

void MacroAssembler::PushBoxed(FloatRegister reg) { MOZ_CRASH(); }

void MacroAssembler::branchPtrInNurseryChunk(Condition cond, Register ptr,
                                             Register temp, Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
  MOZ_ASSERT(temp != InvalidReg);
  MOZ_ASSERT(ptr != temp);
  movePtr(ptr, temp);
  branchPtrInNurseryChunkImpl(cond, temp, label);
}

void MacroAssembler::branchPtrInNurseryChunk(Condition cond,
                                             const Address& address,
                                             Register temp, Label* label) {
  MOZ_ASSERT(temp != InvalidReg);
  loadPtr(address, temp);
  branchPtrInNurseryChunkImpl(cond, temp, label);
}

void MacroAssembler::branchPtrInNurseryChunkImpl(Condition cond, Register ptr,
                                                 Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  andPtr(Imm32(~gc::ChunkMask), ptr);
  branchPtr(InvertCondition(cond), Address(ptr, gc::ChunkStoreBufferOffset),
            ImmWord(0), label);
}

void MacroAssembler::pushReturnAddress() {
  // TODO(dbezhetskov): Wasm don't have access to return addresses so we push
  // const here.
  push(Imm32(0xbeef));
}

void MacroAssembler::popReturnAddress() {
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);
}

CodeOffset MacroAssembler::moveNearAddressWithPatch(Register dest) {
  MOZ_CRASH();
  return CodeOffset(0u);
}

void MacroAssembler::patchNearAddressMove(CodeLocationLabel loc,
                                          CodeLocationLabel target) {
  MOZ_CRASH();
}

size_t MacroAssembler::PushRegsInMaskSizeInBytes(LiveRegisterSet set) {
  return set.gprs().size() * sizeof(intptr_t) + set.fpus().getPushSizeInBytes();
}

void MacroAssembler::PushRegsInMask(LiveRegisterSet set) {
  for (GeneralRegisterIterator iter(set.gprs()); iter.more(); ++iter) {
    if (is64BitReg(*iter)) {
      Push64(*iter);
    } else {
      Push(*iter);
    }
  }

  FloatRegisterSet fpuSet(set.fpus().reduceSetForPush());
  for (FloatRegisterIterator iter(fpuSet); iter.more(); ++iter) {
    Push(*iter);
  }
}

void MacroAssembler::PopRegsInMaskIgnore(LiveRegisterSet set,
                                         LiveRegisterSet ignore) {
  FloatRegisterSet fpuSet(set.fpus().reduceSetForPush());
  for (FloatRegisterBackwardIterator iter(fpuSet); iter.more(); ++iter) {
    if (!ignore.has(*iter)) {
      Pop(*iter);
    } else {
      PopStackSlots(sizeof(double) / sizeof(intptr_t));
    }
  }

  for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); ++iter) {
    if (is64BitReg(*iter)) {
      if (!ignore.has(*iter)) {
        Pop64(*iter);
      } else {
        PopStackSlots(sizeof(int64_t) / sizeof(intptr_t));
      }
    } else {
      if (!ignore.has(*iter)) {
        Pop(*iter);
      } else {
        PopStackSlots(sizeof(int32_t) / sizeof(intptr_t));
      }
    }
  }
}

void MacroAssembler::PopStackPtr() { MOZ_CRASH(); }

void MacroAssembler::freeStackTo(uint32_t framePushed) { MOZ_CRASH(); }

void MacroAssembler::flexibleDivMod32(Register rhs, Register srcDest,
                                      Register remOutput, bool isUnsigned,
                                      const LiveRegisterSet&) {
  register_get32(srcDest);
  register_get32(rhs);

  if (isUnsigned) {
    i32_div_u();
  } else {
    i32_div_s();
  }

  register_get32(srcDest);
  register_get32(rhs);

  if (isUnsigned) {
    i32_rem_u();
  } else {
    i32_rem_s();
  }

  register_set32(remOutput);
  register_set32(srcDest);
}

void MacroAssembler::flexibleRemainder32(Register rhs, Register srcDest,
                                         bool isUnsigned,
                                         const LiveRegisterSet&) {
  register_get32(srcDest);
  register_get32(rhs);

  if (isUnsigned) {
    i32_rem_u();
  } else {
    i32_rem_s();
  }

  register_set32(srcDest);
}

void MacroAssembler::storeRegsInMask(LiveRegisterSet set, Address dest,
                                     Register scratch) {
  MOZ_CRASH();
}

void MacroAssembler::wasmBoundsCheck32(Condition cond, Register index,
                                       Register boundsCheckLimit, Label* ok) {
  MOZ_CRASH();
}

void MacroAssembler::wasmBoundsCheck32(Condition cond, Register index,
                                       Address boundsCheckLimit, Label* ok) {
  MOZ_CRASH();
}

void MacroAssembler::wasmBoundsCheck64(Condition cond, Register64 index,
                                       Register64 boundsCheckLimit, Label* ok) {
  MOZ_CRASH();
}

void MacroAssembler::wasmBoundsCheck64(Condition cond, Register64 index,
                                       Address boundsCheckLimit, Label* ok) {
  MOZ_CRASH();
}

void MacroAssembler::oolWasmTruncateCheckF32ToI32(FloatRegister input,
                                                  Register output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateDoubleToInt64(
    FloatRegister input, Register64 output, bool isSaturating, Label* oolEntry,
    Label* oolRejoin, FloatRegister tempDouble) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateDoubleToUInt64(
    FloatRegister input, Register64 output, bool isSaturating, Label* oolEntry,
    Label* oolRejoin, FloatRegister tempDouble) {
  MOZ_CRASH();
}

void MacroAssembler::oolWasmTruncateCheckF64ToI64(FloatRegister input,
                                                  Register64 output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateFloat32ToInt64(
    FloatRegister input, Register64 output, bool isSaturating, Label* oolEntry,
    Label* oolRejoin, FloatRegister tempDouble) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateFloat32ToUInt64(
    FloatRegister input, Register64 output, bool isSaturating, Label* oolEntry,
    Label* oolRejoin, FloatRegister tempDouble) {
  MOZ_CRASH();
}

void MacroAssembler::oolWasmTruncateCheckF32ToI64(FloatRegister input,
                                                  Register64 output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  MOZ_CRASH();
}

void MacroAssembler::oolWasmTruncateCheckF64ToI32(FloatRegister input,
                                                  Register output,
                                                  TruncFlags flags,
                                                  wasm::BytecodeOffset off,
                                                  Label* rejoin) {
  MOZ_CRASH();
}

void MacroAssembler::convertUInt64ToFloat32(Register64 src, FloatRegister dest,
                                            Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::convertInt64ToFloat32(Register64 src, FloatRegister dest) {
  MOZ_CRASH();
}

bool MacroAssembler::convertUInt64ToDoubleNeedsTemp() { MOZ_CRASH(); }

void MacroAssembler::convertUInt64ToDouble(Register64 src, FloatRegister dest,
                                           Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::convertInt64ToDouble(Register64 src, FloatRegister dst) {
  register_get64(src.low);
  f64_convert_i64_s();
  register_setf64(dst);
}

void MacroAssembler::convertIntPtrToDouble(Register src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::wasmAtomicLoad64(const wasm::MemoryAccessDesc& access,
                                      const Address& mem, Register64 temp,
                                      Register64 output) {
  MOZ_CRASH();
}

void MacroAssembler::wasmAtomicLoad64(const wasm::MemoryAccessDesc& access,
                                      const BaseIndex& mem, Register64 temp,
                                      Register64 output) {
  MOZ_CRASH();
}

void MacroAssembler::patchNopToCall(uint8_t* call, uint8_t* target) {
  MOZ_CRASH();
}

void MacroAssembler::patchCallToNop(uint8_t* call) { MOZ_CRASH(); }

void MacroAssembler::patchCall(uint32_t callerOffset, uint32_t calleeOffset) {
  MOZ_CRASH();
}

CodeOffset MacroAssembler::farJumpWithPatch() {
  MOZ_CRASH();
  return CodeOffset(0);
}

void MacroAssembler::patchFarJump(CodeOffset farJump, uint32_t targetOffset) {
  MOZ_CRASH();
}

CodeOffset MacroAssembler::call(Register reg) {
  wasmPrepareCall();

  wasmGetValue32(reg);
  i32_load();

  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));

  wasmPopI64(ReturnReg);

  register_get64(ReturnReg);
  register_set64(JSReturnOperand.scratchReg());

  return CodeOffset(0);
}

CodeOffset MacroAssembler::call(Label* label) {
  wasmPrepareCall();
  wasmPushI64(R2.scratchReg());  // Additionally pass object in R2.

  i32_const(
      reinterpret_cast<uint32_t>(runtime()->jitRuntime()->postBarrier().value));
  i32_load();

  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_4_I64_TO_I64));

  drop_instr();
  return CodeOffset(0);
}

CodeOffset MacroAssembler::call(wasm::SymbolicAddress imm) {
  MOZ_CRASH();
  return CodeOffset(0);
}

CodeOffset MacroAssembler::callWithPatch() {
  MOZ_CRASH();
  return CodeOffset(0);
}

CodeOffset MacroAssembler::nopPatchableToCall() {
  MOZ_CRASH();
  return CodeOffset(0);
}

FaultingCodeOffset MacroAssembler::wasmTrapInstruction() {
  MOZ_CRASH();
  return FaultingCodeOffset();
}

template <typename T>
void MacroAssembler::storeUnboxedValue(const ConstantOrRegister& value,
                                       MIRType valueType, const T& dest) {
  MOZ_CRASH();
}

template void MacroAssembler::storeUnboxedValue(const ConstantOrRegister& value,
                                                MIRType valueType,
                                                const Address& dest);

template void MacroAssembler::storeUnboxedValue(
    const ConstantOrRegister& value, MIRType valueType,
    const BaseObjectElementIndex& dest);

uint32_t MacroAssembler::pushFakeReturnAddress(Register scratch) {
  MOZ_CRASH();
}

void MacroAssembler::Pop(Register reg) {
  pop(reg);
  implicitPop(sizeof(intptr_t));
}

void MacroAssemblerWasm32::Pop64(Register reg) {
  pop64(reg);
  asMasm().implicitPop(2 * sizeof(intptr_t));
}

void MacroAssembler::Pop(FloatRegister reg) {
  pop(reg);
  implicitPop(reg.isDouble() ? sizeof(double) : sizeof(float));
}

void MacroAssembler::Pop(const ValueOperand& val) {
  popValue(val);
  implicitPop(sizeof(Value));
}

void MacroAssembler::Push(Register reg) {
  push(reg);
  adjustFrame(sizeof(intptr_t));
}

void MacroAssembler::Push(const Imm32 imm) {
  push(imm);
  adjustFrame(sizeof(intptr_t));
}

void MacroAssembler::Push(const ImmWord imm) {
  push(imm);
  adjustFrame(sizeof(intptr_t));
}

void MacroAssembler::Push(const ImmPtr imm) {
  push(imm);
  adjustFrame(int32_t(sizeof(intptr_t)));
}

void MacroAssembler::Push(const ImmGCPtr ptr) {
  push(ptr);
  adjustFrame(sizeof(intptr_t));
}

void MacroAssembler::Push(FloatRegister reg) {
  push(reg);
  adjustFrame(reg.isDouble() ? sizeof(double) : sizeof(float));
}

void MacroAssembler::wasmTruncateFloat32ToInt32(FloatRegister input,
                                                Register output,
                                                bool isSaturating,
                                                Label* oolEntry) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateFloat32ToUInt32(FloatRegister input,
                                                 Register output,
                                                 bool isSaturating,
                                                 Label* oolEntry) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateDoubleToUInt32(FloatRegister input,
                                                Register output,
                                                bool isSaturating,
                                                Label* oolEntry) {
  MOZ_CRASH();
}

void MacroAssembler::wasmTruncateDoubleToInt32(FloatRegister input,
                                               Register output,
                                               bool isSaturating,
                                               Label* oolEntry) {
  MOZ_CRASH();
}

void MacroAssembler::wasmAtomicExchange64(const wasm::MemoryAccessDesc& access,
                                          const Address& mem, Register64 value,
                                          Register64 output) {
  MOZ_CRASH();
}

void MacroAssembler::wasmAtomicExchange64(const wasm::MemoryAccessDesc& access,
                                          const BaseIndex& mem,
                                          Register64 value, Register64 output) {
  MOZ_CRASH();
}

void MacroAssembler::speculationBarrier() { MOZ_CRASH(); }

void MacroAssembler::shiftIndex32AndAdd(Register indexTemp32, int shift,
                                        Register pointer) {
  MOZ_CRASH();
}

void MacroAssembler::setupUnalignedABICall(Register) {
  setupNativeABICall();
  dynamicAlignment_ = true;

  // Preserve SP and align it.
  wasmPushI32(StackPointer);
  alignSP(ABIStackAlignment);
}

void MacroAssembler::enterFakeExitFrameForWasm(Register cxreg, Register scratch,
                                               ExitFrameType type) {
  MOZ_CRASH();
}

void MacroAssembler::floorFloat32ToInt32(FloatRegister src, Register dest,
                                         Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::floorDoubleToInt32(FloatRegister src, Register dest,
                                        Label* fail) {
  register_getf64(src);
  f64_floor();
  f64_const(INT32_MAX);
  f64_gt();

  register_getf64(src);
  f64_floor();
  f64_const(INT32_MIN);
  f64_lt();

  i32_or();
  j(fail);

  register_getf64(src);
  f64_floor();
  i32_trunc_f64_s();
  register_set32(dest);
}

void MacroAssembler::ceilFloat32ToInt32(FloatRegister src, Register dest,
                                        Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::ceilDoubleToInt32(FloatRegister src, Register dest,
                                       Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::roundFloat32ToInt32(FloatRegister src, Register dest,
                                         FloatRegister temp, Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::roundDoubleToInt32(FloatRegister src, Register dest,
                                        FloatRegister temp, Label* fail) {
  ScratchDoubleScope scratch(*this);

  Label negativeOrZero, negative, end;

  // Branch to a slow path for non-positive inputs. Doesn't catch NaN.
  zeroDouble(scratch);
  loadConstantDouble(GetBiggestNumberLessThan(0.5), temp);
  branchDouble(Assembler::DoubleLessThanOrEqual, src, scratch, &negativeOrZero);
  {
    // Input is strictly positive or NaN. Add the biggest double less than 0.5
    // and truncate, rounding down (because if the input is the biggest double
    // less than 0.5, adding 0.5 would undesirably round up to 1). Note that we
    // have to add the input to the temp register because we're not allowed to
    // modify the input register.
    addDouble(src, temp);
    truncateDoubleToInt32(temp, dest, fail);
    jump(&end);
  }

  // Input is negative, +0 or -0.
  bind(&negativeOrZero);
  {
    // Branch on negative input.
    branchDouble(Assembler::DoubleNotEqual, src, scratch, &negative);

    // Fail on negative-zero.
    branchDoubleNegativeZero(src, fail, /* maybeNonZero = */ false);

    // Input is +0.
    xor32(dest, dest);
    jump(&end);
  }

  // Input is negative.
  bind(&negative);
  {
    // Inputs in [-0.5, 0) are rounded to -0. Fail.
    loadConstantDouble(-0.5, scratch);
    branchDouble(Assembler::DoubleGreaterThanOrEqual, src, scratch, fail);

    addDouble(src, temp);
    truncateDoubleToInt32(scratch, dest, fail);
  }

  bind(&end);
}

void MacroAssembler::truncFloat32ToInt32(FloatRegister src, Register dest,
                                         Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::truncDoubleToInt32(FloatRegister src, Register dest,
                                        Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::nearbyIntDouble(RoundingMode mode, FloatRegister src,
                                     FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::nearbyIntFloat32(RoundingMode mode, FloatRegister src,
                                      FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::copySignDouble(FloatRegister lhs, FloatRegister rhs,
                                    FloatRegister output) {
  MOZ_CRASH();
}

void MacroAssembler::branchTestValue(Condition cond, const ValueOperand& lhs,
                                     const Value& rhs, Label* label) {
  MOZ_RELEASE_ASSERT(cond == Equal || cond == NotEqual);

  register_get64(lhs.scratchReg());
  i64_const(rhs.asRawBits());

  if (cond == Equal) {
    i64_eq();
  } else {
    i64_neq();
  }

  j(label);
}

void MacroAssembler::branchValueIsNurseryCell(Condition cond,
                                              const Address& address,
                                              Register temp, Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  Label done;
  branchTestGCThing(Assembler::NotEqual, address,
                    cond == Assembler::Equal ? &done : label);
  branchPtrInNurseryChunk(cond, ToPayload(address), temp, label);
  bind(&done);
}

void MacroAssembler::branchValueIsNurseryCell(Condition cond,
                                              ValueOperand value, Register temp,
                                              Label* label) {
  MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  Label done;

  branchTestGCThing(Assembler::NotEqual, value,
                    cond == Assembler::Equal ? &done : label);
  branchPtrInNurseryChunk(cond, value.payloadReg(), temp, label);

  bind(&done);
}

void MacroAssembler::callWithABINoProfiler(Register fun, ABIType result) {
  MOZ_CRASH();
}

void MacroAssembler::callWithABINoProfiler(const Address& fun, ABIType result) {
  uint32_t stackAdjust;
  callWithABIPre(&stackAdjust);
  lastRuntimeCallFunctionSignatureIndex_ =
      mozilla::Some(wasm32::SignatureIndex::SIGNATURE_3_I32_TO_1_I32);
  wasmGetValue32(fun);
  call_indirect(static_cast<uint32_t>(*lastRuntimeCallFunctionSignatureIndex_));
  callWithABIPost(stackAdjust, result);
}

void MacroAssembler::call(const Address& addr) {
  wasmGetValue32(addr);
  i32_load();
  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));
}

void MacroAssembler::call(ImmWord imm) { MOZ_CRASH(); }

void MacroAssembler::call(ImmPtr imm) {
  if (!lastRuntimeCallFunctionSignatureIndex_) {
    // We are calling VM from Baseline.
    return jump(imm);
  }

  i32_const(reinterpret_cast<uint32_t>(imm.value));
  call_indirect(static_cast<uint32_t>(*lastRuntimeCallFunctionSignatureIndex_));
}

void MacroAssembler::call(JitCode* c) { MOZ_CRASH(); }

void MacroAssembler::callWithABIPre(uint32_t*, bool) {
  MOZ_RELEASE_ASSERT(moveResolver_.resolve());

  MOZ_RELEASE_ASSERT(moveResolver_.numCycles() == 0);
  std::vector<MoveOp> moveOps;
  for (int i = moveResolver_.numMoves() - 1; i >= 0; --i) {
    moveOps.push_back(moveResolver_.getMove(i));
  }

  std::sort(moveOps.begin(), moveOps.end(),
            [](const MoveOp& lhs, const MoveOp& rhs) {
              MOZ_RELEASE_ASSERT(lhs.to().isMemory());
              MOZ_RELEASE_ASSERT(rhs.to().isMemory());

              return lhs.to().disp() < rhs.to().disp();
            });

  for (const auto& moveOp : moveOps) {
    switch (moveOp.type()) {
      case MoveOp::INT32:
      case MoveOp::GENERAL: {
        if (moveOp.from().isGeneralReg()) {
          wasmPushI32(moveOp.from().reg());
        } else if (moveOp.from().isEffectiveAddress()) {
          wasmPushAddress(Address(moveOp.from().base(), moveOp.from().disp()));
        } else {
          MOZ_RELEASE_ASSERT(moveOp.from().isMemory());
          wasmPushI32(Address(moveOp.from().base(), moveOp.from().disp()));
        }
        break;
      }
      case MoveOp::FLOAT32: {
        if (moveOp.from().isFloatReg()) {
          wasmPushF32(moveOp.from().floatReg());
        } else {
          MOZ_CRASH();
        }
        break;
      }
      case MoveOp::DOUBLE: {
        if (moveOp.from().isFloatReg()) {
          wasmPushF64(moveOp.from().floatReg());
        } else if (moveOp.from().isEffectiveAddress()) {
          wasmPushAddress(Address(moveOp.from().base(), moveOp.from().disp()));
        } else {
          MOZ_RELEASE_ASSERT(moveOp.from().isMemory());
          wasmPushF64(Address(moveOp.from().base(), moveOp.from().disp()));
        }
        break;
      }
      default:
        MOZ_CRASH("Unexpected move type");
    }
  }
}

void MacroAssembler::callWithABIPost(uint32_t stackAdjust, ABIType result,
                                     bool callFromWasm) {
  MOZ_RELEASE_ASSERT(!callFromWasm);  // wasm32 target doesn't support Wasm.

  if (hasReturnValue(*lastRuntimeCallFunctionSignatureIndex_)) {
    switch (result) {
      case ABIType::General:
      case ABIType::Int32:
        wasmPopI32(ReturnReg);

        register_get32(ReturnReg);
        register_set32(JSReturnOperand.scratchReg());
        break;
      case ABIType::Float32:
        MOZ_CRASH("NYI");
        break;
      case ABIType::Float64:
        wasmPopF64(ReturnDoubleReg);
        break;
      default:
        MOZ_CRASH("Invalid argument type");
    }
  }

  if (dynamicAlignment_) {
    // Restore previous value of SP that we push in setupUnalignedABICall.
    wasmPopI32(StackPointer);
  }

  lastRuntimeCallFunctionSignatureIndex_.reset();
}

void MacroAssembler::comment(const char* msg) { MOZ_CRASH(); }

void MacroAssembler::flush() { MOZ_CRASH(); }

void MacroAssembler::loadStoreBuffer(Register ptr, Register buffer) {
  MOZ_CRASH();
}

void MacroAssembler::moveValue(const TypedOrValueRegister& src,
                               const ValueOperand& dest) {
  MOZ_CRASH();
}

void MacroAssembler::moveValue(const ValueOperand& src,
                               const ValueOperand& dest) {
  register_get64(src.scratchReg());
  register_set64(dest.scratchReg());
}

void MacroAssembler::moveValue(const Value& src, const ValueOperand& dest) {
  if (src.isGCThing()) {
    writeDataRelocation(ImmGCPtr(src.toGCThing()));
  }

  i64_const(src.asRawBits());
  register_set64(dest.scratchReg());
}

void MacroAssembler::wasmCompareExchange64(const wasm::MemoryAccessDesc& access,
                                           const Address& mem,
                                           Register64 expected,
                                           Register64 replacement,
                                           Register64 output) {
  MOZ_CRASH();
}

void MacroAssembler::wasmCompareExchange64(const wasm::MemoryAccessDesc& access,
                                           const BaseIndex& mem,
                                           Register64 expected,
                                           Register64 replacement,
                                           Register64 output) {
  MOZ_CRASH();
}

//}}} check_macroassembler_style

MacroAssembler& MacroAssemblerWasm32::asMasm() {
  return *static_cast<MacroAssembler*>(this);
}

const MacroAssembler& MacroAssemblerWasm32::asMasm() const {
  return *static_cast<const MacroAssembler*>(this);
}

void MacroAssemblerWasm32::flushBuffer() {
  // Compile the IR into code and flush it into the assembler's buffer.
  finish();
}

void MacroAssemblerWasm32::bind(Label* label) {
  MOZ_RELEASE_ASSERT(!label->bound());

  br_instr(nextBlockId_);
  ensureNewBlockWithLink();

  if (label->used()) {
    auto nextTarget = label->offset();
    while (nextTarget != LabelBase::INVALID_OFFSET) {
      const auto blockIndex = nextTarget;
      wasm32::BasicBlock* targetBlock = blocks_.at(blockIndex).get();
      targetBlock->addSuccessor(currentBlock_);

      MOZ_RELEASE_ASSERT(
          targetBlock->instructions.back().is<wasm32::BR_INSTR>());
      wasm32::BR_INSTR& brInstruction =
          targetBlock->instructions.back().as<wasm32::BR_INSTR>();
      nextTarget = brInstruction.blockIndex;
      brInstruction.blockIndex = currentBlock_->id;
    }
  }

  label->bind(currentBlock_->id);

  // Make an edge to the switch head.
  if (switchInfo_.contains(label)) {
    switchInfo_.switchBlockByLabel(label)->addSuccessor(
        blocks_[currentBlock_->id].get());
  }
}

void MacroAssemblerWasm32::j(Label* target) {
  // The result is on the wasm stack, so we don't need condition here.

  if (target->bound()) {
    const auto targetBlockId = target->offset();
    br_if(targetBlockId, nextBlockId_);
    currentBlock_->addSuccessor(blocks_.at(targetBlockId).get());
  } else {
    br_if(target->offset(), nextBlockId_);
    target->use(currentBlock_->id);
  }

  ensureNewBlockWithLink();
}

void MacroAssemblerWasm32::jump(Label* label) {
  if (label->bound()) {
    const auto targetBlockId = label->offset();
    br_instr(targetBlockId);
    currentBlock_->addSuccessor(blocks_.at(targetBlockId).get());
  } else {
    br_instr(label->offset());
    label->use(currentBlock_->id);
  }

  ensureNewBlock();
}

void MacroAssemblerWasm32::jump(Register reg) {
  register_get32(reg);
  // Will patch it later because right now we haven't known the targets yet.
  br_table({});

  ensureNewBlock();
}

void MacroAssemblerWasm32::jump(const Address& address) {
  wasmGetValue32(address);
  i32_load();
  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));
}

void MacroAssemblerWasm32::jump(ImmPtr ptr) {
  wasmPrepareCall();

  i32_const(reinterpret_cast<uint32_t>(ptr.value));
  i32_load();

  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));

  wasmPopI64(ReturnReg);

  register_get64(ReturnReg);
  register_set64(JSReturnOperand.scratchReg());
}

void MacroAssemblerWasm32::callPreBarrier(TrampolinePtr ptr) {
  wasmPrepareCall();

  i32_const(reinterpret_cast<uint32_t>(ptr.value));
  i32_load();

  call_indirect(static_cast<uint32_t>(
      wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));

  drop_instr();
}

void MacroAssemblerWasm32::writeCodePointer(CodeLabel* label) { MOZ_CRASH(); }

void MacroAssemblerWasm32::haltingAlign(size_t) {}

void MacroAssemblerWasm32::nopAlign(size_t) { MOZ_CRASH(); }

void MacroAssemblerWasm32::checkStackAlignment() {
#ifdef DEBUG
  Label done;
  register_get32(StackPointer);

  register_get32(StackPointer);
  i32_const(~(ABIStackAlignment - 1));
  i32_and();

  i32_eq();
  j(&done);

  unreachable();

  bind(&done);
#endif
}

void MacroAssemblerWasm32::nop() { wasm_nop(); }

void MacroAssemblerWasm32::unreachable() { unreachable_instr(); }

void MacroAssemblerWasm32::breakpoint() {}

void MacroAssemblerWasm32::abiret() {
  wasmPushI64(wcx);  // by wasm abi
  return_instr();
}

void MacroAssemblerWasm32::ret() {
  // Pop fake ret pc.
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);

  wasmPushI64(wcx);  // by wasm abi

  return_instr();
}

CodeOffset MacroAssemblerWasm32::toggledJump(Label* target) {
  CodeOffset offset(size());
  jump(target);
  return offset;
}

CodeOffset MacroAssemblerWasm32::toggledCall(JitCode*, bool) { MOZ_CRASH(); }

size_t MacroAssemblerWasm32::ToggledCallSize(uint8_t*) { MOZ_CRASH(); }

void MacroAssemblerWasm32::finish() {
  fillPendingsBranches();
  Assembler::finish();
}

void MacroAssemblerWasm32::storeValue(ValueOperand val, const Address& dest) {
  wasmEvalOperand32(dest);
  register_get64(val.scratchReg());
  i64_store();
}

void MacroAssemblerWasm32::storeValue(ValueOperand val, const BaseIndex& dest) {
  wasmEvalOperand32(dest);
  register_get64(val.scratchReg());
  i64_store();
}

void MacroAssemblerWasm32::storeValue(JSValueType type, Register payload,
                                      const Address& dest) {
  wasmEvalOperand32(dest);

  i64_const(static_cast<uint64_t>(ImmType(type).value) << 32);
  register_get32(payload);
  i64_extend_i32_u();
  i64_or();

  i64_store();
}

void MacroAssemblerWasm32::storeValue(const Value& val, Address dest) {
  wasmEvalOperand32(dest);

  i64_const(val.asRawBits());
  i64_store();

  if (val.isGCThing()) {
    writeDataRelocation(ImmGCPtr(val.toGCThing()));
  }
}

void MacroAssemblerWasm32::storeValue(const Address& src, const Address& dest,
                                      Register temp) {
  MOZ_CRASH();
}

void MacroAssemblerWasm32::loadValue(Address src, ValueOperand val) {
  wasmGetValue64(src);
  register_set64(val.scratchReg());
}

void MacroAssemblerWasm32::loadValue(const BaseIndex& src, ValueOperand val) {
  wasmGetValue64(src);
  register_set64(val.scratchReg());
}

void MacroAssemblerWasm32::pushValue(const Address& addr) {
  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);

  register_get32(addr.base);
  i32_const(addr.offset + (addr.base == StackPointer ? sizeof(Value) : 0));
  i32_add();
  i64_load();

  i64_store();
}

void MacroAssemblerWasm32::pushValue(ValueOperand val) {
  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  register_get64(val.scratchReg());
  i64_store();
}

void MacroAssemblerWasm32::pushValue(const Value& val) {
  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  i64_const(val.asRawBits());
  i64_store();

  if (val.isGCThing()) {
    writeDataRelocation(ImmGCPtr(val.toGCThing()));
  }
}

void MacroAssemblerWasm32::pushValue(JSValueType type, Register payload) {
  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  i64_const(static_cast<uint64_t>(ImmType(type).value) << 32);
  register_get32(payload);
  i64_extend_i32_u();
  i64_or();
  i64_store();
}

void MacroAssemblerWasm32::pushValue(const BaseIndex& addr, Register) {
  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  wasmEvalOperand32(
      BaseIndex(addr.base, addr.index, addr.scale,
                addr.offset + (addr.base == StackPointer ? sizeof(Value) : 0)));
  i64_load();

  i64_store();
}

void MacroAssemblerWasm32::popValue(ValueOperand val) {
  register_get32(StackPointer);
  i64_load();
  register_set64(val.scratchReg());

  register_get32(StackPointer);
  i32_const(sizeof(Value));
  i32_add();
  register_set32(StackPointer);
}

void MacroAssemblerWasm32::tagValue(JSValueType type, Register payload,
                                    ValueOperand dest) {
  i64_const(static_cast<uint64_t>(ImmType(type).value) << 32);
  register_get32(payload);
  i64_extend_i32_u();
  i64_or();
  register_set64(dest.scratchReg());
}

void MacroAssemblerWasm32::retn(Imm32 n) {
  if (n.value) {
    register_get32(StackPointer);
    i32_const(n.value);
    i32_add();
    register_set32(StackPointer);
  }

  return_instr();
}

void MacroAssemblerWasm32::push(const Address& src) {
  // increment $sp
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  register_get32(src.base);
  i32_const(src.offset + (src.base == StackPointer ? STACK_SLOT_SIZE : 0));
  i32_add();
  i32_load();

  i32_store();
}

void MacroAssemblerWasm32::push(Register src) {
  // increment $sp
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  // store value of reg
  register_get32(StackPointer);
  register_get32(src);
  if (src == StackPointer) {
    i32_const(STACK_SLOT_SIZE);
    i32_add();
  }
  i32_store();
}

void MacroAssemblerWasm32::push64(Register src) {
  MOZ_RELEASE_ASSERT(src != StackPointer);

  // increment $sp
  register_get32(StackPointer);
  i32_const(2 * STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  // store value of reg
  register_get32(StackPointer);
  register_get64(src);
  i64_store();
}

void MacroAssemblerWasm32::Push64(Register src) {
  push64(src);
  asMasm().adjustFrame(2 * sizeof(intptr_t));
}

void MacroAssemblerWasm32::push(Imm32 imm) {
  // increment $sp
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  // store value of reg
  register_get32(StackPointer);
  i32_const(imm.value);
  i32_store();
}

void MacroAssemblerWasm32::push(ImmPtr imm) {
  push(Imm32(reinterpret_cast<int32_t>(imm.value)));
}

void MacroAssemblerWasm32::push(ImmGCPtr imm) {
  writeDataRelocation(imm);
  push(ImmPtr(imm.value));
}

void MacroAssemblerWasm32::push(ImmWord imm) {
  push(Imm32(static_cast<int32_t>(imm.value)));
}

void MacroAssemblerWasm32::push(FloatRegister fr) {
  // increment $sp
  register_get32(StackPointer);
  i32_const(fr.isSingle() ? STACK_SLOT_SIZE : 2 * STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  // store value of reg
  register_get32(StackPointer);
  if (fr.isDouble()) {
    register_getf64(fr);
    f64_store();
  } else if (fr.isSingle()) {
    register_getf32(fr);
    f32_store();
  } else {
    MOZ_CRASH();
  }
}

void MacroAssemblerWasm32::pop(Register reg) {
  MOZ_RELEASE_ASSERT(reg != StackPointer);

  // load value
  register_get32(StackPointer);
  i32_load();
  register_set32(reg);

  // decrement $sp
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);
}

void MacroAssemblerWasm32::pop64(Register reg) {
  MOZ_RELEASE_ASSERT(reg != StackPointer);

  // load value
  register_get32(StackPointer);
  i64_load();
  register_set64(reg);

  // decrement $sp
  register_get32(StackPointer);
  i32_const(2 * STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);
}

void MacroAssemblerWasm32::pop(FloatRegister reg) {
  // load value
  register_get32(StackPointer);
  if (reg.isSingle()) {
    f32_load();
    register_setf32(reg);
  } else if (reg.isDouble()) {
    f64_load();
    register_setf64(reg);
  } else {
    MOZ_CRASH();
  }

  // decrement $sp
  register_get32(StackPointer);
  i32_const(reg.isSingle() ? STACK_SLOT_SIZE : 2 * STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);
}

void MacroAssemblerWasm32::pop(const ValueOperand& val) { popValue(val); }

void MacroAssemblerWasm32::PopStackSlots(const std::size_t numSlots) {
  register_get32(StackPointer);
  i32_const(numSlots * STACK_SLOT_SIZE);
  i32_add();
  register_set32(StackPointer);

  asMasm().implicitPop(numSlots * STACK_SLOT_SIZE);
}

void MacroAssemblerWasm32::testNullSet(Assembler::Condition cond,
                                       const ValueOperand& value,
                                       Register dest) {
  testSetHelper(cond, JSVAL_TAG_NULL, value, dest);
}

void MacroAssemblerWasm32::testObjectSet(Condition cond,
                                         const ValueOperand& value,
                                         Register dest) {
  testSetHelper(cond, JSVAL_TAG_OBJECT, value, dest);
}

void MacroAssemblerWasm32::testUndefinedSet(Assembler::Condition cond,
                                            const ValueOperand& value,
                                            Register dest) {
  testSetHelper(cond, JSVAL_TAG_UNDEFINED, value, dest);
}

void MacroAssemblerWasm32::testSetHelper(Condition cond, JSValueTag tag,
                                         const ValueOperand& value,
                                         Register dest) {
  MOZ_RELEASE_ASSERT(cond == Equal || cond == NotEqual);

  i32_const(1);  // select true case
  i32_const(0);  // select false case

  extractTypeFromi64(value);
  i32_const(ImmTag(tag).value);
  emitCondition32(cond);

  select_instr();
  register_set32(dest);
}

void MacroAssemblerWasm32::movePtr(ImmGCPtr imm, Register dest) {
  writeDataRelocation(imm);
  movePtr(ImmPtr(imm.value), dest);
}

void MacroAssemblerWasm32::movePtr(ImmPtr ptr, Register dst) {
  static_assert(sizeof(void*) == sizeof(int32_t));
  move32(Imm32(reinterpret_cast<int32_t>(ptr.value)), dst);
}

void MacroAssemblerWasm32::movePtr(Register src, Register dst) {
  move32(src, dst);
}

void MacroAssemblerWasm32::move32(Imm32 imm, Register dest) {
  i32_const(imm.value);
  register_set32(dest);
}

void MacroAssemblerWasm32::move32(Register src, Register dest) {
  if (src == dest) {
    return;
  }

  register_get32(src);
  register_set32(dest);
}

void MacroAssemblerWasm32::moveDouble(FloatRegister src, FloatRegister dst) {
  register_getf64(src);
  register_setf64(dst);
}

void MacroAssemblerWasm32::zeroDouble(FloatRegister dst) {
  loadConstantDouble(0.0, dst);
}

void MacroAssemblerWasm32::storePtr(ImmGCPtr src, const Address& address) {
  storePtr(ImmPtr(src.value), address);
  writeDataRelocation(src);
}

void MacroAssemblerWasm32::computeEffectiveAddress(const Address& address,
                                                   Register dest) {
  wasmEvalOperand32(address);
  register_set32(dest);
}

void MacroAssemblerWasm32::computeEffectiveAddress(const BaseIndex& address,
                                                   Register dest) {
  wasmEvalOperand32(address);
  register_set32(dest);
}

void MacroAssemblerWasm32::notBoolean(const ValueOperand& val) {
  i64_const(1);
  register_get64(val.scratchReg());
  i64_xor();
  register_set64(val.scratchReg());
}

void MacroAssemblerWasm32::convertBoolToInt32(Register source, Register dest) {
  register_get32(source);
  register_set32(dest);
}

void MacroAssemblerWasm32::loadConstantDouble(double d, FloatRegister dst) {
  f64_const(d);
  register_setf64(dst);
}

void MacroAssemblerWasm32::incrementInt32Value(const Address& addr) {
  const auto payloadAddress = ToPayload(addr);

  // push address
  wasmEvalOperand32(addr);

  // push first operand
  wasmGetValue32(payloadAddress);

  // push second operand
  i32_const(1);

  i32_add();

  // store the result
  i32_store();
}

// If source is a double, load it into dest. If source is int32,
// convert it to double. Else, branch to failure.
void MacroAssemblerWasm32::ensureDouble(const ValueOperand& source,
                                        FloatRegister dest, Label* failure) {
  Label isDouble, done;
  {
    ScratchRegisterScope scratch(asMasm());
    extractTypeFromi64(source.scratchReg());
    register_set32(scratch);

    asMasm().branchTestDouble(Assembler::Equal, scratch, &isDouble);
    asMasm().branchTestInt32(Assembler::NotEqual, scratch, failure);
  }

  {
    ScratchRegisterScope scratch(asMasm());
    unboxInt32(source, scratch);
    convertInt32ToDouble(scratch, dest);
  }
  jump(&done);

  bind(&isDouble);
  unboxDouble(source, dest);

  bind(&done);
}

BaseIndex MacroAssemblerWasm32::ToPayload(const BaseIndex& address) const {
  return BaseIndex(address.base, address.index, address.scale,
                   address.offset + PAYLOAD_OFFSET);
}

BaseValueIndex MacroAssemblerWasm32::ToPayload(
    const BaseValueIndex& address) const {
  return BaseValueIndex(address.base, address.index,
                        address.offset + PAYLOAD_OFFSET);
}

void MacroAssemblerWasm32::profilerEnterFrame(Register framePtr,
                                              Register scratch) {
  asMasm().loadJSContext(scratch);
  loadPtr(Address(scratch, offsetof(JSContext, profilingActivation_)), scratch);
  storePtr(framePtr,
           Address(scratch, JitActivation::offsetOfLastProfilingFrame()));
  storePtr(ImmPtr(nullptr),
           Address(scratch, JitActivation::offsetOfLastProfilingCallSite()));
}

void MacroAssemblerWasm32::profilerExitFrame() {
  jump(asMasm().runtime()->jitRuntime()->getProfilerExitFrameTail());
}

Address MacroAssemblerWasm32::ToPayload(const Address& address) const {
  return Address(address.base, address.offset + PAYLOAD_OFFSET);
}

Address MacroAssemblerWasm32::ToType(const Address& address) {
  return Address(address.base, address.offset + TAG_OFFSET);
}

BaseIndex MacroAssemblerWasm32::ToType(const BaseIndex& address) {
  return BaseIndex(address.base, address.index, address.scale,
                   address.offset + TAG_OFFSET);
}

void MacroAssemblerWasm32::wasmSub32(Register src, Imm32 offset, Register dst) {
  register_get32(src);
  i32_const(offset.value);
  i32_sub();
  register_set32(dst);
}

void MacroAssemblerWasm32::wasmCallByIndexInLocal(uint32_t localIndex,
                                                  uint32_t typeIndex) {
  local_get(localIndex);
  i32_load();
  call_indirect(typeIndex);
}

void MacroAssemblerWasm32::wasmMovLocal32ToReg(uint32_t localIndex,
                                               Register reg) {
  local_get(localIndex);
  register_set32(reg);
}

void MacroAssemblerWasm32::wasmPushLocal(uint32_t localIndex) {
  register_get32(StackPointer);
  i32_const(STACK_SLOT_SIZE);
  i32_sub();
  register_set32(StackPointer);

  register_get32(StackPointer);
  local_get(localIndex);
  i32_store();
}

void MacroAssemblerWasm32::wasmCallByIndex(uint32_t index, uint32_t typeIndex) {
  i32_const(index);
  call_indirect(typeIndex);
}

void MacroAssemblerWasm32::fillPendingsBranches() {
  // Fill br_table's targets.
  for (auto [switchBlock, caseLabels] :
       switchInfo_.switchDispatchBlockToCaseLabels_) {
    MOZ_RELEASE_ASSERT(switchBlock->instructions.back().is<wasm32::BR_TABLE>());
    auto& dispatchInstruction =
        switchBlock->instructions.back().as<wasm32::BR_TABLE>();
    dispatchInstruction.blockIndices.clear();
    for (auto* label : caseLabels) {
      MOZ_RELEASE_ASSERT(label->bound());
      dispatchInstruction.blockIndices.push_back(
          static_cast<uint32_t>(label->offset()));
    }
  }
}

void MacroAssemblerWasm32::wasmPrepareCall() {
  wasmPushI32(FramePointer);
  wasmPushI64(wcx);
  wasmPushI64(wax);
  wasmPushI64(wdi);
}

void MacroAssemblerWasm32::lea(const BaseIndex& src, Register dst) {
  wasmEvalOperand32(src);
  register_set32(dst);
}

void MacroAssemblerWasm32::lshift32(Register src, Register dst,
                                    Register shift) {
  wasmEvalOperand32(src);
  wasmEvalOperand32(shift);
  i32_shl();
  register_set32(dst);
}

void MacroAssemblerWasm32::truncateDoubleToInt32(FloatRegister src,
                                                 Register dst, Label* fail) {
  register_getf64(src);
  f64_trunc();
  f64_const(INT32_MAX);
  emitConditionf64(Assembler::DoubleGreaterThan);

  register_getf64(src);
  f64_trunc();
  f64_const(INT32_MIN);
  emitConditionf64(Assembler::DoubleLessThan);

  i32_or();
  j(fail);

  register_getf64(src);
  i32_trunc_f64_s();
  register_set32(dst);
}

void MacroAssemblerWasm32::branchDoubleNegativeZero(FloatRegister src,
                                                    Label* fail,
                                                    bool maybeNonZero) {
  if (maybeNonZero) {
    register_getf64(src);
    f64_const(0.0);
    emitConditionf64(Assembler::DoubleNotEqual);
    j(fail);
  }

  f64_const(1.0);
  register_getf64(src);
  f64_copysign();

  f64_const(-1.0);
  emitConditionf64(Assembler::DoubleEqual);
  j(fail);
}

void MacroAssemblerWasm32::alignSP(uint32_t alignment) {
  register_get32(StackPointer);
  i32_const(~(alignment - 1));
  i32_and();
  register_set32(StackPointer);
}

void MacroAssemblerWasm32::recordLabelForSwitch(Label* caseLabel) {
  switchInfo_.recordLabelForSwitchBlock(caseLabel, currentBlock_);
}

void MacroAssemblerWasm32::setSignatureForRuntimeCall(
    wasm32::SignatureIndex signatureIndex) {
  lastRuntimeCallFunctionSignatureIndex_ = mozilla::Some(signatureIndex);
}

void MacroAssemblerWasm32::wasmPushI32(Register reg) { register_get32(reg); }

void MacroAssemblerWasm32::wasmPushI32(const Address& address) {
  wasmGetValue32(address);
}

void MacroAssemblerWasm32::wasmPushAddress(const Address& address) {
  wasmEvalOperand32(address);
}

void MacroAssemblerWasm32::wasmPushF64(const Address& address) {
  wasmEvalOperand32(address);
  f64_load();
}

void MacroAssemblerWasm32::wasmPopI32(Register reg) { register_set32(reg); }

void MacroAssemblerWasm32::wasmPopI64(Register reg) { register_set64(reg); }

void MacroAssemblerWasm32::wasmPopF64(FloatRegister reg) {
  register_setf64(reg);
}

void MacroAssemblerWasm32::wasmPushI64(Register reg) { register_get64(reg); }

void MacroAssemblerWasm32::wasmPushF32(FloatRegister reg) {
  register_getf32(reg);
}

void MacroAssemblerWasm32::wasmPushF64(FloatRegister reg) {
  register_getf64(reg);
}

static void WasmPrintMsg(const char* message) {
  std::cout << message << std::endl;
}

void MacroAssemblerWasm32::wasmPrintMsg(const char* message) {
  wasmPushI32(StackPointer);
  alignSP(ABIStackAlignment);

  i32_const(reinterpret_cast<uint32_t>(message));
  wasmCallByIndex(
      reinterpret_cast<uint32_t>(WasmPrintMsg),
      static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_I32_TO_VOID));

  wasmPopI32(StackPointer);
}

static void WasmPrint32(uint32_t value) {
  std::cout << "WasmPrint32:VALUE == " << (void*)value << std::endl;
}

void MacroAssemblerWasm32::wasmPrintReg32(Register reg) {
  MOZ_RELEASE_ASSERT(reg != StackPointer);

  wasmPushI32(StackPointer);
  alignSP(ABIStackAlignment);

  wasmPushI32(reg);
  wasmCallByIndex(
      reinterpret_cast<uint32_t>(WasmPrint32),
      static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_I32_TO_VOID));

  wasmPopI32(StackPointer);
}

static void WasmPrint64(uint64_t value) {
  std::cout << "WasmPrint64:VALUE == " << value << std::endl;
}

static void WasmPrintF64(double value) {
  std::cout << "WasmPrintF64:VALUE == " << value << std::endl;
}

void MacroAssemblerWasm32::wasmPrintReg64(Register reg) {
  wasmPushI32(StackPointer);
  alignSP(ABIStackAlignment);

  wasmPushI64(reg);
  wasmCallByIndex(
      reinterpret_cast<uint32_t>(WasmPrint64),
      static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_i64_TO_VOID));

  wasmPopI32(StackPointer);
}

void MacroAssemblerWasm32::wasmPrintRegF64(FloatRegister reg) {
  wasmPushI32(StackPointer);
  alignSP(ABIStackAlignment);

  wasmPushF64(reg);
  wasmCallByIndex(
      reinterpret_cast<uint32_t>(WasmPrintF64),
      static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_F64_TO_VOID));

  wasmPopI32(StackPointer);
}

static void WasmPrintMemory(uint32_t* base, uint32_t numValues) {
  std::cout << "WasmPrintMemory BEGIN " << (void*)base << std::endl;
  for (int i = numValues - 1; i >= 0; --i) {
    std::cout << "WasmPrint32 == " << (void*)*(base + i) << std::endl;
  }
  std::cout << "WasmPrintMemory END" << std::endl;
}

void MacroAssemblerWasm32::wasmPrintMemory(Register base, uint32_t numValues) {
  wasmPushI32(base);
  i32_const(numValues);
  wasmCallByIndex(
      reinterpret_cast<uint32_t>(WasmPrintMemory),
      static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_2_I32_TO_VOID));
}

void MacroAssemblerWasm32::extractTypeFromi64(const ValueOperand& value) {
  extractTypeFromi64(value.scratchReg());
}

void MacroAssemblerWasm32::extractTypeFromi64(Register src) {
  register_get64(src);
  i64_const(32);
  i64_shr_u();
  i32_wrap_i64();
}

void MacroAssemblerWasm32::extractPayloadFromi64(const ValueOperand& value) {
  register_get64(value.scratchReg());
  i32_wrap_i64();
}

void SwitchInfo::recordLabelForSwitchBlock(Label* caseLabel,
                                           wasm32::BasicBlock* block) {
  switchDispatchBlockToCaseLabels_[block].push_back(caseLabel);
  labelToSwitchHead_[caseLabel] = block;
}

bool SwitchInfo::contains(Label* label) const {
  return labelToSwitchHead_.find(label) != labelToSwitchHead_.end();
}

wasm32::BasicBlock* SwitchInfo::switchBlockByLabel(Label* label) {
  return labelToSwitchHead_[label];
}

void SwitchInfo::clear() {
  switchDispatchBlockToCaseLabels_.clear();
  labelToSwitchHead_.clear();
}

}  // namespace js::jit
