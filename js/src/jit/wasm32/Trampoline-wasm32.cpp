/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/MathAlgorithms.h"

#include "jit/Bailouts.h"
#include "jit/BaselineIC.h"
#include "jit/JitRuntime.h"
#include "jit/wasm32/SharedICRegisters-wasm32.h"
#include "vm/Realm.h"

#include "jit/MacroAssembler-inl.h"

using namespace js;
using namespace js::jit;

namespace {

wasm32::SignatureIndex signatureIndexByFun(const VMFunctionData& f) {
  switch (f.argc()) {
    case 7:
      return wasm32::SignatureIndex::SIGNATURE_7_I32_TO_1_I32;
    case 6:
      return wasm32::SignatureIndex::SIGNATURE_6_I32_TO_1_I32;
    case 5:
      return wasm32::SignatureIndex::SIGNATURE_5_I32_TO_1_I32;
    case 4:
      return wasm32::SignatureIndex::SIGNATURE_4_I32_TO_1_I32;
    case 3: {
      if (f.argProperties(0) == VMFunctionData::DoubleByValue &&
          f.argProperties(1) == VMFunctionData::WordByValue) {
        return wasm32::SignatureIndex::SIGNATURE_1_I32_1_F64_TO_1_I32;
      }
      return wasm32::SignatureIndex::SIGNATURE_3_I32_TO_1_I32;
    }
    case 2:
      return wasm32::SignatureIndex::SIGNATURE_2_I32_TO_1_I32;
    case 1:
      return wasm32::SignatureIndex::SIGNATURE_1_I32_TO_1_I32;
    default:
      return wasm32::SignatureIndex::SIGNATURE_VOID_TO_VOID;
  }
}

}  // namespace

void JitRuntime::generateEnterJIT(JSContext* cx, MacroAssembler& masm) {
  AutoCreatedBy acb(masm, "JitRuntime::generateEnterJIT");

  enterJITOffset_ = startTrampolineCode(masm);

  // EnterJitCode signature.
  constexpr uint32_t localCodeArgIdx = 0;
  constexpr uint32_t localArgcArgIdx = 1;
  constexpr uint32_t localArgvArgIdx = 2;
  constexpr uint32_t localOsrFrameArgIdx = 3;
  constexpr uint32_t localCalleeTokenArgIdx = 4;
  constexpr uint32_t localEnvChainArgIdx = 5;
  constexpr uint32_t localOsrNumStackValuesArgIdx = 6;
  constexpr uint32_t localResultAddressArgIdx = 7;

  {
    using wasm32::WasmLocalType;

    masm.introduceSignature(wasm32::FunctionSignature{
        {WasmLocalType::I32, WasmLocalType::I32, WasmLocalType::I32,
         WasmLocalType::I32, WasmLocalType::I32, WasmLocalType::I32,
         WasmLocalType::I32, WasmLocalType::I32},
        {}});

    // Introduce locals for registers since this stub has a different signature.
    auto framePointerLocalIndex = masm.introduceLocal(WasmLocalType::I32);
    MOZ_RELEASE_ASSERT(framePointerLocalIndex == 8);
    auto R0LocalIndex = masm.introduceLocal(WasmLocalType::I64);
    MOZ_RELEASE_ASSERT(R0LocalIndex == 9);
    auto R1LocalIndex = masm.introduceLocal(WasmLocalType::I64);
    MOZ_RELEASE_ASSERT(R1LocalIndex == 10);
    auto R2LocalIndex = masm.introduceLocal(WasmLocalType::I64);
    MOZ_RELEASE_ASSERT(R2LocalIndex == 11);
  }

  // Save old stack frame pointer and set new stack
  // frame pointer.
  masm.pushReturnAddress();
  masm.push(FramePointer);
  masm.moveStackPtrTo(FramePointer);

  // Load the number of values to be copied (argc) into wax.
  masm.wasmMovLocal32ToReg(localArgcArgIdx, wax);

  // If we are constructing, that also needs to include newTarget
  {
    Label noNewTarget;
    masm.wasmMovLocal32ToReg(localCalleeTokenArgIdx, wdx);
    masm.branchTest32(Assembler::Zero, wdx,
                      Imm32(CalleeToken_FunctionConstructing), &noNewTarget);

    masm.add32(Imm32(1), wax);

    masm.bind(&noNewTarget);
  }

  // wax <- sizeof(Value) * numValues, wax is now the offset betwen argv and the
  // last value.
  masm.lshift32(Imm32(3), wax);
  static_assert(sizeof(Value) == 1 << 3, "Constant is baked in assembly code");

  // Guarantee stack alignment of Jit frames.
  //
  // This code compensates for the offset created by the copy of the vector of
  // arguments, such that the jit frame will be aligned once the return
  // address is pushed on the stack.
  //
  // In the computation of the offset, we omit the size of the JitFrameLayout
  // which is pushed on the stack, as the JitFrameLayout size is a multiple of
  // the JitStackAlignment.
  masm.moveStackPtrTo(wcx);
  masm.sub32(wax, wcx);
  static_assert(
      sizeof(JitFrameLayout) % JitStackAlignment == 0,
      "No need to consider the JitFrameLayout for aligning the stack");

  // wcx = wcx & 15, holds alignment.
  masm.and32(Imm32(JitStackAlignment - 1), wcx);
  masm.sub32(wcx, StackPointer);

  /***************************************************************
  Loop over argv vector, push arguments onto stack in reverse order
  ***************************************************************/

  masm.wasmMovLocal32ToReg(localArgvArgIdx, wbx);

  // wax = argv + 8 * argc --wax now points one value past the last argument
  masm.add32(wbx, wax);

  // while (wax > wbx) --while still looping through arguments
  {
    Label header, footer;
    masm.bind(&header);
    masm.branch32(Assembler::BelowOrEqual, wax, wbx, &footer);

    // wax -= sizeof(Value)  --move to previous argument
    masm.sub32(Imm32(sizeof(Value)), wax);

    // Push what wax points to on stack.
    masm.pushValue(Address(wax, 0));

    masm.jump(&header);
    masm.bind(&footer);
  }

  // Load the number of actual arguments.  |result| is used to store the
  // actual number of arguments without adding an extra argument to the enter
  // JIT.
  masm.wasmMovLocal32ToReg(localResultAddressArgIdx, wax);
  masm.unboxInt32(Address(wax, 0x0), wax);

  // Push the callee token.
  masm.wasmPushLocal(localCalleeTokenArgIdx);

  // Load the InterpreterFrame address into the OsrFrameReg.
  // This address is also used for setting the constructing bit on all paths.
  masm.wasmMovLocal32ToReg(localOsrFrameArgIdx, OsrFrameReg);

  // Push the descriptor.
  masm.pushFrameDescriptorForJitCall(FrameType::CppToJSJit, wax, wax);

  // Interpreter -> Baseline OSR transition is not supported.
  masm.wasmMovLocal32ToReg(localEnvChainArgIdx, R1.scratchReg());
  (void)localOsrNumStackValuesArgIdx;

  // The callee will push the return address and frame pointer on the stack,
  // thus we check that the stack would be aligned once the call is complete.
  masm.assertStackAlignment(JitStackAlignment, 2 * sizeof(uintptr_t));

  /***************************************************************
      Call passed-in code, get return value and fill in the
      passed in return value pointer
  ***************************************************************/
  masm.wasmPrepareCall();
  masm.wasmCallByIndexInLocal(
      localCodeArgIdx,
      static_cast<uint32_t>(
          wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));
  masm.wasmPopI64(JSReturnOperand.scratchReg());
  masm.wasmMovLocal32ToReg(localResultAddressArgIdx, wax);
  masm.storeValue(JSReturnOperand, Address(wax, 0));

  /**************************************************************
      Return stack and registers to correct state
  **************************************************************/
  masm.moveToStackPtr(FramePointer);
  masm.pop(FramePointer);
  masm.ret();
}

// static
mozilla::Maybe<::JS::ProfilingFrameIterator::RegisterState>
JitRuntime::getCppEntryRegisters(JitFrameLayout* frameStackAddress) {
  return mozilla::Nothing{};
}

void JitRuntime::generateInvalidator(MacroAssembler&, Label*) { /* do nothing */
}

void JitRuntime::generateArgumentsRectifier(MacroAssembler& masm,
                                            ArgumentsRectifierKind kind) {
  AutoCreatedBy acb(masm, "JitRuntime::generateArgumentsRectifier");

  switch (kind) {
    case ArgumentsRectifierKind::Normal:
      argumentsRectifierOffset_ = startTrampolineCode(masm);
      break;
    case ArgumentsRectifierKind::TrialInlining:
      trialInliningArgumentsRectifierOffset_ = startTrampolineCode(masm);
      break;
  }

  // Caller:
  // [arg2] [arg1] [this] [ [argc] [callee] [descr] [raddr] ] <- SP

  // Frame prologue.
  //
  // NOTE: if this changes, fix the Baseline bailout code too!
  // See BaselineStackBuilder::calculatePrevFramePtr and
  // BaselineStackBuilder::buildRectifierFrame (in BaselineBailouts.cpp).
  masm.pushReturnAddress();
  masm.push(FramePointer);
  masm.moveStackPtrTo(FramePointer);

  // Load argc.
  masm.loadNumActualArgs(FramePointer, wsi);

  // Load |nformals| into %wcx.
  masm.loadPtr(
      Address(FramePointer, RectifierFrameLayout::offsetOfCalleeToken()), wax);
  masm.move32(wax, wcx);
  masm.and32(Imm32(CalleeTokenMask), wcx);
  masm.loadFunctionArgCount(wcx, wcx);

  // The frame pointer and its padding are pushed on the stack.
  // Including |this|, there are (|nformals| + 1) arguments to push to the
  // stack.  Then we push a JitFrameLayout.  We compute the padding expressed
  // in the number of extra |undefined| values to push on the stack.
  static_assert(
      sizeof(JitFrameLayout) % JitStackAlignment == 0,
      "No need to consider the JitFrameLayout for aligning the stack");
  static_assert(
      JitStackAlignment % sizeof(Value) == 0,
      "Ensure that we can pad the stack by pushing extra UndefinedValue");
  static_assert(mozilla::IsPowerOfTwo(JitStackValueAlignment),
                "must have power of two for masm.and32 to do its job");

  masm.add32(
      Imm32(JitStackValueAlignment - 1 /* for padding */ + 1 /* for |this| */),
      wcx);

  // Account for newTarget, if necessary.
  static_assert(
      CalleeToken_FunctionConstructing == 1,
      "Ensure that we can use the constructing bit to count an extra push");
  masm.move32(wax, wdx);
  masm.and32(Imm32(CalleeToken_FunctionConstructing), wdx);
  masm.add32(wdx, wcx);

  masm.and32(Imm32(~(JitStackValueAlignment - 1)), wcx);
  masm.sub32(wsi, wcx);
  masm.sub32(Imm32(1), wcx);  // For |this|.

  // Copy the number of actual arguments into edx.
  masm.mov(wsi, wdx);

  // Caller:
  // [arg2] [arg1] [this] [ [argc] [callee] [descr] [raddr] ]
  // '-- #wsi ---'
  //
  // Rectifier frame:
  // [wbp'] <- wbp [padding] <- wsp [undef] [undef] [arg2] [arg1] [this]
  //                                '--- #wcx ----' '-- #wsi ---'
  //
  // [ [argc] [callee] [descr] [raddr] ]

  {
    auto undefinedValueOperand = ValueOperand(InvalidReg, wdi);
    masm.moveValue(UndefinedValue(), undefinedValueOperand);

    // Push undefined.
    {
      Label undefLoopTop;
      masm.bind(&undefLoopTop);

      masm.pushValue(undefinedValueOperand);
      masm.branchSub32(Assembler::NonZero, Imm32(1), wcx, &undefLoopTop);
    }
  }

  // Get the topmost argument.
  masm.lea(
      BaseIndex(FramePointer, wsi, TimesEight, sizeof(RectifierFrameLayout)),
      wcx);

  // Push arguments, |nargs| + 1 times (to include |this|).
  masm.add32(Imm32(1), wsi);
  {
    Label copyLoopTop;

    masm.bind(&copyLoopTop);

    masm.pushValue(Address(wcx, 0));
    masm.sub32(Imm32(sizeof(Value)), wcx);
    masm.branchSub32(Assembler::NonZero, Imm32(1), wsi, &copyLoopTop);
  }

  {
    Label notConstructing;

    masm.mov(wax, wbx);
    masm.branchTest32(Assembler::Zero, wbx,
                      Imm32(CalleeToken_FunctionConstructing),
                      &notConstructing);

    BaseValueIndex src(FramePointer, wdx,
                       sizeof(RectifierFrameLayout) + sizeof(Value));

    masm.and32(Imm32(CalleeTokenMask), wbx);
    masm.loadFunctionArgCount(wbx, wbx);

    BaseValueIndex dst(wsp, wbx, sizeof(Value));

    ValueOperand newTarget(wcx, wdi);

    masm.loadValue(src, newTarget);
    masm.storeValue(newTarget, dst);

    masm.bind(&notConstructing);
  }

  // Construct JitFrameLayout.
  masm.push(wax);  // callee token
  masm.pushFrameDescriptorForJitCall(FrameType::Rectifier, wdx, wdx);

  // Call the target function.
  masm.and32(Imm32(CalleeTokenMask), wax);
  switch (kind) {
    case ArgumentsRectifierKind::Normal:
      masm.loadJitCodeRaw(wax, wax);
      argumentsRectifierReturnOffset_ = masm.callJitNoProfiler(wax);
      break;
    case ArgumentsRectifierKind::TrialInlining:
      Label noBaselineScript, done;
      masm.loadBaselineJitCodeRaw(wax, wbx, &noBaselineScript);
      masm.callJitNoProfiler(wbx);
      masm.jump(&done);

      // See BaselineCacheIRCompiler::emitCallInlinedFunction.
      masm.bind(&noBaselineScript);
      masm.loadJitCodeRaw(wax, wax);
      masm.callJitNoProfiler(wax);
      masm.bind(&done);
      break;
  }

  masm.mov(FramePointer, StackPointer);
  masm.pop(FramePointer);
  masm.ret();
}

void JitRuntime::generateBailoutHandler(MacroAssembler&,
                                        Label*) { /* do nothing */
}

uint32_t JitRuntime::generatePreBarrier(JSContext* cx, MacroAssembler& masm,
                                        MIRType type) {
  AutoCreatedBy acb(masm, "JitRuntime::generatePreBarrier");
  uint32_t offset = startTrampolineCode(masm);

  masm.pushReturnAddress();

  Register temp1 = wax;
  Register temp2 = wbx;
  Register temp3 = wdx;
  MOZ_ASSERT(temp1 != PreBarrierReg);
  MOZ_ASSERT(temp2 != PreBarrierReg);
  MOZ_ASSERT(temp3 != PreBarrierReg);

  masm.push(temp1);
  masm.push(temp2);
  masm.push(temp3);

  Label noBarrier;
  masm.emitPreBarrierFastPath(cx->runtime(), type, temp1, temp2, temp3,
                              &noBarrier);

  // Call into C++ to mark this GC thing.
  masm.pop(temp3);
  masm.pop(temp2);
  masm.pop(temp1);

  LiveRegisterSet save;
  save.set() = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                           FloatRegisterSet(FloatRegisters::VolatileMask));
  masm.PushRegsInMask(save);

  Register runtimeReg = wcx;
  masm.movePtr(ImmPtr(cx->runtime()), runtimeReg);

  // Since we don't provide any signature for a call at compile-time we need to
  // specify it manually.
  masm.setSignatureForRuntimeCall(
      wasm32::SignatureIndex::SIGNATURE_2_I32_TO_VOID);
  masm.setupUnalignedABICall(wax);
  masm.passABIArg(runtimeReg);
  masm.passABIArg(PreBarrierReg);
  masm.callWithABI(JitPreWriteBarrier(type));

  masm.PopRegsInMask(save);
  masm.ret();

  masm.bind(&noBarrier);
  masm.pop(temp3);
  masm.pop(temp2);
  masm.pop(temp1);

  masm.ret();

  return offset;
}

uint32_t JitRuntime::generatePostBarrier(JSContext* cx, MacroAssembler& masm) {
  AutoCreatedBy acb(masm, "JitRuntime::generatePostBarrier");
  uint32_t offset = startTrampolineCode(masm);

  masm.pushReturnAddress();
  Register objReg = R2.scratchReg();

  // Check one element cache to avoid VM call.
  Label skipBarrier;
  auto* lastCellAddr = cx->runtime()->gc.addressOfLastBufferedWholeCell();
  masm.branchPtr(Assembler::Equal, AbsoluteAddress(lastCellAddr), objReg,
                 &skipBarrier);

  AllocatableGeneralRegisterSet regs(GeneralRegisterSet::All());
  MOZ_ASSERT(!regs.has(FramePointer));
  regs.take(R0);
  regs.take(objReg);
  Register scratch = regs.takeAny();

  masm.pushValue(R0);

  using Fn = void (*)(JSRuntime* rt, js::gc::Cell* cell);
  masm.setupUnalignedABICall(scratch);
  masm.movePtr(ImmPtr(cx->runtime()), scratch);
  masm.passABIArg(scratch);
  masm.passABIArg(objReg);
  masm.callWithABI<Fn, PostWriteBarrier>();

  masm.popValue(R0);

  masm.bind(&skipBarrier);
  masm.ret();

  return offset;
}

void JitRuntime::generateBailoutTailStub(MacroAssembler&, Label*) {}

bool JitRuntime::generateVMWrapper(JSContext* cx, MacroAssembler& masm,
                                   VMFunctionId id, const VMFunctionData& f,
                                   DynFn nativeFun, uint32_t* wrapperOffset) {
  AutoCreatedBy acb(masm, "JitRuntime::generateVMWrapper");
  *wrapperOffset = startTrampolineCode(masm);

  // Avoid conflicts with argument registers while discarding the result after
  // the function call.
  AllocatableGeneralRegisterSet regs(Register::Codes::WrapperMask);

  static_assert(
      (Register::Codes::VolatileMask & ~Register::Codes::WrapperMask) == 0,
      "Wrapper register set must be a superset of Volatile register set.");

  // The context is the first argument.
  Register cxreg = regs.takeAny();

  // On link-register platforms, it is the responsibility of the VM *callee* to
  // push the return address, while the caller must ensure that the address
  // is stored in a link register on entry. This allows the VM wrapper to work
  // with both direct calls and tail calls.
  masm.pushReturnAddress();

  // Stack is:
  //    ... frame ...
  //  +8  [args]
  //  +4  descriptor
  //  +0  returnAddress
  //
  // Push the frame pointer to finish the exit frame, then link it up.
  masm.Push(FramePointer);
  masm.moveStackPtrTo(FramePointer);
  masm.loadJSContext(cxreg);
  masm.enterExitFrame(cxreg, regs.getAny(), id);

  // Reserve space for the outparameter.
  masm.reserveVMFunctionOutParamSpace(f);

  masm.setupUnalignedABICallDontSaveRestoreSP();
  masm.passABIArg(cxreg);

  // Copy arguments.
  size_t argDisp = ExitFrameLayout::Size();
  for (uint32_t explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
    switch (f.argProperties(explicitArg)) {
      case VMFunctionData::WordByValue:
        masm.passABIArg(MoveOperand(FramePointer, argDisp), ABIType::General);
        argDisp += sizeof(void*);
        break;
      case VMFunctionData::DoubleByValue:
        masm.passABIArg(MoveOperand(FramePointer, argDisp), ABIType::Float64);
        argDisp += 2 * sizeof(void*);
        break;
      case VMFunctionData::WordByRef:
        masm.passABIArg(MoveOperand(FramePointer, argDisp,
                                    MoveOperand::Kind::EffectiveAddress),
                        ABIType::General);
        argDisp += sizeof(void*);
        break;
      case VMFunctionData::DoubleByRef:
        masm.passABIArg(MoveOperand(FramePointer, argDisp,
                                    MoveOperand::Kind::EffectiveAddress),
                        ABIType::General);
        argDisp += 2 * sizeof(void*);
        break;
    }
  }

  // Copy the implicit outparam, if any.
  const int32_t outParamOffset =
      -int32_t(ExitFooterFrame::Size()) - f.sizeOfOutParamStackSlot();
  if (f.outParam != Type_Void) {
    masm.passABIArg(MoveOperand(FramePointer, outParamOffset,
                                MoveOperand::Kind::EffectiveAddress),
                    ABIType::General);
  }

  masm.setSignatureForRuntimeCall(signatureIndexByFun(f));
  masm.callWithABI(nativeFun, ABIType::General,
                   CheckUnsafeCallWithABI::DontCheckHasExitFrame);
  // Test for failure.
  Label success;
  switch (f.failType()) {
    case Type_Cell:
      masm.branchTestPtr(Assembler::NonZero, ReturnReg, ReturnReg, &success);
      masm.wasmPrintMsg("Exceptions from callVM are NYI.");
      masm.unreachable();  // Exceptions are NYI.
      break;
    case Type_Bool:
      masm.branchTest32(Assembler::NonZero, ReturnReg, ReturnReg, &success);
      masm.wasmPrintMsg("Exceptions from callVM are NYI.");
      masm.unreachable();  // Exceptions are NYI.
      break;
    case Type_Void:
      break;
    default:
      MOZ_CRASH("unknown failure kind");
  }
  masm.bind(&success);

  // Load the outparam.
  masm.loadVMFunctionOutParam(f, Address(FramePointer, outParamOffset));

  switch (f.outParam) {
    case Type_Handle: {
      if (f.outParamRootType == VMFunctionData::RootValue) {
        masm.wasmPushI64(JSReturnOperand.scratchReg());
      } else {
        masm.wasmPushI64(ReturnReg);
      }
      break;
    }

    case Type_Value: {
      masm.wasmPushI64(JSReturnOperand.scratchReg());
      break;
    }

    case Type_Int32:
    case Type_Pointer:
    case Type_Bool: {
      masm.wasmPushI64(ReturnReg);
      break;
    }

    case Type_Double: {
      masm.wasmPrintMsg("Type_Double return case for vm wrapper NYI");
      masm.unreachable();
      break;
    }

    default: {
      masm.wasmPushI64(ReturnReg);
      MOZ_ASSERT(f.outParam == Type_Void);
      break;
    }
  }

  // Pop frame and restore frame pointer.
  masm.moveToStackPtr(FramePointer);
  masm.pop(FramePointer);

  masm.retn(Imm32(sizeof(ExitFrameLayout) - sizeof(void*) +
                  f.explicitStackSlots() * sizeof(void*) +
                  f.extraValuesToPop * sizeof(Value)));
  return true;
}
