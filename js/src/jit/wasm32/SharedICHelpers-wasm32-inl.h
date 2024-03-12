/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_SharedICHelpers_wasm32_inl_h
#define jit_wasm32_SharedICHelpers_wasm32_inl_h

#include "jit/BaselineFrame.h"
#include "jit/SharedICHelpers.h"

#include "jit/MacroAssembler-inl.h"

namespace js::jit {

inline void EmitBaselineTailCallVM(TrampolinePtr target, MacroAssembler& masm,
                                   uint32_t argSize) {
#ifdef DEBUG
  MOZ_CRASH("Not supported");
#endif

  // Push frame descriptor and perform the tail call.
  masm.pushFrameDescriptor(FrameType::BaselineJS);
  // The return address will be pushed by the VM wrapper, for compatibility
  // with direct calls. Refer to the top of generateVMWrapper().

  masm.jump(target);

  // According to the wasm abi we should pass return value from runtime.
  masm.wasmPushI64(ReturnReg);

  // To unwind one function frame and return to the caller
  // because wasm mvp doesn't support tail calls.
  masm.return_instr();
}

inline void EmitBaselineCallVM(TrampolinePtr target, MacroAssembler& masm) {
  masm.pushFrameDescriptor(FrameType::BaselineStub);
  masm.jump(target);

  // IC calls returns JSValues.
  masm.wasmPushI64(ReturnReg);
  masm.wasmPopI64(JSReturnOperand.scratchReg());
}

inline void EmitBaselineEnterStubFrame(MacroAssembler& masm, Register) {
#ifdef DEBUG
  MOZ_CRASH("Not supported");
#endif

  // Push frame descriptor and fake return address.
  masm.pushFrameDescriptor(FrameType::BaselineJS);
  masm.Push(Imm32(0xbeef));

  // Save old frame pointer, stack pointer and stub reg.
  masm.Push(FramePointer);
  masm.mov(StackPointer, FramePointer);

  masm.Push(ICStubReg);
}

}  // namespace js::jit

#endif /* jit_wasm32_SharedICHelpers_wasm32_inl_h */
