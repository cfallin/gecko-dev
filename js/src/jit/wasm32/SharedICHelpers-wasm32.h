/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_SharedICHelpers_wasm32_h
#define jit_wasm32_SharedICHelpers_wasm32_h

#include "jit/BaselineIC.h"
#include "jit/JitFrames.h"
#include "jit/MacroAssembler.h"
#include "jit/SharedICRegisters.h"

namespace js::jit {

static const size_t ICStackValueOffset = 0;

inline void EmitRestoreTailCallReg(MacroAssembler&) {
  // Do nothing.
}

inline void EmitRepushTailCallReg(MacroAssembler&) {
  // Do nothing.
}

inline void EmitCallIC(MacroAssembler& masm, CodeOffset* callOffset) {
  masm.wasmPrepareCall();

  masm.call(Address(ICStubReg, ICStub::offsetOfStubCode()));
  *callOffset = CodeOffset(masm.currentOffset());

  masm.wasmPopI64(JSReturnOperand.scratchReg());
}

inline void EmitDirectCallIC(MacroAssembler& masm, uint32_t functionIndex,
                             CodeOffset* callOffset) {
  masm.wasmPrepareCall();

  masm.wasm_call(functionIndex);
  *callOffset = CodeOffset(masm.currentOffset());

  masm.wasmPopI64(JSReturnOperand.scratchReg());
}

inline void EmitReturnFromIC(MacroAssembler& masm) {
  masm.wasmPushI64(JSReturnOperand.scratchReg());
  masm.return_instr();
}

inline void EmitBaselineLeaveStubFrame(MacroAssembler& masm) {
  Address stubAddr(FramePointer, BaselineStubFrameLayout::ICStubOffsetFromFP);
  masm.loadPtr(stubAddr, ICStubReg);

  masm.mov(FramePointer, StackPointer);
  masm.Pop(FramePointer);

  // Load the return address.
  masm.Pop(ICTailCallReg);

  // Discard the frame descriptor.
  masm.add32(Imm32(sizeof(intptr_t)), StackPointer);
  masm.implicitPop(sizeof(intptr_t));
}

inline void EmitStubGuardFailure(MacroAssembler& masm) {
  // Load next stub into ICStubReg
  masm.loadPtr(Address(ICStubReg, ICCacheIRStub::offsetOfNext()), ICStubReg);

  masm.wasmPrepareCall();

  // Return address is already loaded, just jump to the next stubcode.
  masm.jump(Address(ICStubReg, ICStub::offsetOfStubCode()));

  masm.return_instr();
}

template <typename AddrType>
inline void EmitPreBarrier(MacroAssembler& masm, const AddrType& addr,
                           MIRType type) {
  masm.guardedCallPreBarrier(addr, type);
}

}  // namespace js::jit

#endif /* jit_wasm32_SharedICHelpers_wasm32_h */
