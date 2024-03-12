/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_SharedICRegisters_wasm32_h
#define jit_wasm32_SharedICRegisters_wasm32_h

#include "jit/Registers.h"
#include "jit/RegisterSets.h"
#include "jit/wasm32/MacroAssembler-wasm32.h"

namespace js {
namespace jit {

// ValueOperands R0, R1, and R2
static constexpr ValueOperand R0(InvalidReg, wcx);
static constexpr ValueOperand R1(InvalidReg, wax);
static constexpr ValueOperand R2(InvalidReg, wsi);

// ICTailCallReg and ICStubReg reuse
// registers from R2.
static constexpr Register ICTailCallReg = wsi;
static constexpr Register ICStubReg = wdi;

static constexpr FloatRegister FloatReg0 = d0;
static constexpr FloatRegister FloatReg1 = d1;
static constexpr FloatRegister FloatReg2 = d2;
static constexpr FloatRegister FloatReg3 = d3;

}  // namespace jit
}  // namespace js

#endif /* jit_wasm32_SharedICRegisters_wasm32_h */
