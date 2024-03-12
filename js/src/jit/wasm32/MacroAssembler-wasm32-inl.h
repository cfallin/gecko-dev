/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_MacroAssembler_wasm32_inl_h
#define jit_wasm32_MacroAssembler_wasm32_inl_h

#include "jit/wasm32/MacroAssembler-wasm32.h"

namespace js::jit {

//{{{ check_macroassembler_style

void MacroAssembler::move64(Imm64 imm, Register64 dst) {
  i64_const(imm.value);
  register_set64(dst.low);
}

void MacroAssembler::move64(Register64 src, Register64 dst) {
  wasmGetValue64(src.low);
  register_set64(dst.low);
}

void MacroAssembler::move64To32(Register64 src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::moveDoubleToGPR64(FloatRegister src, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::moveGPR64ToDouble(Register64 src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::move32To64ZeroExtend(Register src, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::move8To64SignExtend(Register src, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::move16To64SignExtend(Register src, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::move32To64SignExtend(Register src, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::move32SignExtendToPtr(Register src, Register dest) {
  register_get32(src);
  register_set32(dest);
}

void MacroAssembler::move32ZeroExtendToPtr(Register src, Register dest) {
  register_get32(src);
  register_set32(dest);
}

void MacroAssembler::load32SignExtendToPtr(const Address& src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::notPtr(Register reg) { MOZ_CRASH(); }

void MacroAssembler::andPtr(Register src, Register dest) { and32(src, dest); }

void MacroAssembler::andPtr(Imm32 imm, Register dest) { and32(imm, dest); }

void MacroAssembler::and64(Imm64 imm, Register64 dst) {
  i64_const(imm.value);
  register_get64(dst.low);
  i64_and();
  register_set64(dst.low);
}

void MacroAssembler::and64(Register64 src, Register64 dst) {
  register_get64(src.low);
  register_get64(dst.low);
  i64_and();
  register_set64(dst.low);
}

void MacroAssembler::or64(Imm64 imm, Register64 dest) { MOZ_CRASH(); }

void MacroAssembler::xor64(Imm64 imm, Register64 dest) {
  wasmGetValue64(dest.low);
  i64_const(imm.value);
  i64_xor();
  register_set64(dest.low);
}

void MacroAssembler::xor64(Register64 src, Register64 dest) {
  wasmGetValue64(dest.low);
  wasmGetValue64(src.low);
  i64_xor();
  register_set64(dest.low);
}

void MacroAssembler::orPtr(Register src, Register dest) { or32(src, dest); }

void MacroAssembler::orPtr(Imm32 imm, Register dest) { or32(imm, dest); }

void MacroAssembler::or64(Register64 src, Register64 dest) { MOZ_CRASH(); }

void MacroAssembler::xorPtr(Register src, Register dest) { xor32(src, dest); }

void MacroAssembler::xorPtr(Imm32 imm, Register dest) { xor32(imm, dest); }

void MacroAssembler::byteSwap64(Register64 reg) { MOZ_CRASH(); }

void MacroAssembler::addPtr(Register src, Register dest) { add32(src, dest); }

void MacroAssembler::addPtr(Imm32 imm, Register dest) { add32(imm, dest); }

void MacroAssembler::addPtr(ImmWord imm, Register dest) { MOZ_CRASH(); }

void MacroAssembler::add64(Register64 src, Register64 dst) {
  register_get64(src.low);
  register_get64(dst.low);
  i64_add();
  register_set64(dst.low);
}

void MacroAssembler::add64(Imm32 imm, Register64 dst) {
  wasmGetValue64(imm);
  register_get64(dst.low);
  i64_add();
  register_set64(dst.low);
}

void MacroAssembler::add64(Imm64 imm, Register64 dst) {
  i64_const(imm.value);
  register_get64(dst.low);
  i64_add();
  register_set64(dst.low);
}

CodeOffset MacroAssembler::sub32FromStackPtrWithPatch(Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::patchSub32FromStackPtr(CodeOffset offset, Imm32 imm) {
  MOZ_CRASH();
}

void MacroAssembler::subPtr(Register src, Register dest) { sub32(src, dest); }

void MacroAssembler::subPtr(Imm32 imm, Register dest) { sub32(imm, dest); }

void MacroAssembler::sub64(Register64 src, Register64 dest) { MOZ_CRASH(); }

void MacroAssembler::sub64(Imm64 imm, Register64 dest) { MOZ_CRASH(); }

void MacroAssembler::mulHighUnsigned32(Imm32 imm, Register src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::mulPtr(Register rhs, Register srcDest) { MOZ_CRASH(); }

void MacroAssembler::mul64(Imm64 imm, const Register64& dest) { MOZ_CRASH(); }

void MacroAssembler::mul64(const Register64& src, const Register64& dest,
                           const Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::mulBy3(Register src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::inc64(AbsoluteAddress dest) {
  wasmEvalOperand64(dest);

  wasmGetValue64(dest);
  i64_const(1);
  i64_add();

  i64_store();
}

void MacroAssembler::neg64(Register64 reg) { MOZ_CRASH(); }

void MacroAssembler::negPtr(Register reg) { MOZ_CRASH(); }

void MacroAssembler::lshiftPtr(Imm32 imm, Register dest) {
  lshift32(imm, dest);
}

void MacroAssembler::rshiftPtr(Imm32 imm, Register dest) {
  rshift32(imm, dest);
}

void MacroAssembler::rshiftPtrArithmetic(Imm32 imm, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::lshift64(Imm32 imm, Register64 srcDest) {
  register_get64(srcDest.low);
  wasmGetValue64(imm);
  i64_shl();
  register_set64(srcDest.low);
}

void MacroAssembler::lshift64(Register shift, Register64 srcDest) {
  register_get64(srcDest.low);
  wasmGetValue64(shift);
  i64_shl();
  register_set64(srcDest.low);
}

void MacroAssembler::rshift64(Imm32 imm, Register64 dst) {
  register_get64(dst.low);
  wasmGetValue64(imm);
  i64_shr_s();
  register_set64(dst.low);
}

void MacroAssembler::rshift64(Register shift, Register64 srcDest) {
  register_get64(srcDest.low);
  register_get64(shift);
  i64_shr_s();
  register_set64(srcDest.low);
}

void MacroAssembler::rshift64Arithmetic(Imm32 imm, Register64 dest) {
  MOZ_CRASH();
}

void MacroAssembler::lshiftPtr(Register shift, Register srcDest) {
  MOZ_CRASH();
}

void MacroAssembler::rshiftPtr(Register shift, Register srcDest) {
  MOZ_CRASH();
}

void MacroAssembler::rshift64Arithmetic(Register shift, Register64 srcDest) {
  MOZ_CRASH();
}

void MacroAssembler::clz64(Register64 src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::ctz64(Register64 src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::popcnt64(Register64 src, Register64 dest, Register temp) {
  MOZ_CRASH();
}

template <typename T1, typename T2>
void MacroAssembler::cmpPtrSet(Condition cond, T1 lhs, T2 rhs, Register dest) {
  i32_const(1);  // select true case
  i32_const(0);  // select false case

  wasmGetValue32(lhs);
  wasmGetValue32(rhs);
  emitCondition32(cond);

  select_instr();
  register_set32(dest);
}

void MacroAssembler::branchToComputedAddress(const BaseIndex& address) {
  MOZ_CRASH();
}

void MacroAssembler::move8ZeroExtend(Register src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::move8SignExtend(Register src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::move16SignExtend(Register src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::loadAbiReturnAddress(Register dest) { MOZ_CRASH(); }

void MacroAssembler::not32(Register reg) {
  register_get32(reg);
  i32_const(-1);
  i32_xor();
  register_set32(reg);
}

void MacroAssembler::and32(Register src, Register dest) {
  wasmEvalOperand32(src);
  wasmEvalOperand32(dest);
  i32_and();
  register_set32(dest);
}

void MacroAssembler::and32(Imm32 imm, Register dest) {
  register_get32(dest);
  i32_const(imm.value);
  i32_and();
  register_set32(dest);
}

void MacroAssembler::and32(Imm32 imm, const Address& dest) {
  wasmEvalOperand32(dest);

  wasmGetValue32(dest);
  i32_const(imm.value);
  i32_and();

  i32_store();
}

void MacroAssembler::and32(const Address& src, Register dest) {
  wasmGetValue32(src);
  wasmEvalOperand32(dest);
  i32_and();
  register_set32(dest);
}

void MacroAssembler::or32(Register src, Register dest) {
  register_get32(dest);
  register_get32(src);
  i32_or();

  register_set32(dest);
}

void MacroAssembler::or32(Imm32 imm, Register dest) {
  register_get32(dest);
  i32_const(imm.value);
  i32_or();

  register_set32(dest);
}

void MacroAssembler::or32(Imm32 imm, const Address& dest) {
  // push address
  wasmEvalOperand32(dest);

  // push first operand
  wasmGetValue32(dest);

  // push second operand
  i32_const(imm.value);

  i32_or();

  // store the result
  i32_store();
}

void MacroAssembler::xor32(Register src, Register dest) {
  wasmEvalOperand32(src);
  wasmEvalOperand32(dest);
  i32_xor();
  register_set32(dest);
}

void MacroAssembler::xor32(Imm32 imm, Register dest) {
  wasmEvalOperand32(imm);
  wasmEvalOperand32(dest);
  i32_xor();
  register_set32(dest);
}

void MacroAssembler::xor32(Imm32 imm, const Address& dest) {
  wasmEvalOperand32(dest);

  wasmGetValue32(dest);
  i32_const(imm.value);
  i32_xor();

  i32_store();
}

void MacroAssembler::xor32(const Address& src, Register dest) {
  wasmGetValue32(src);
  wasmEvalOperand32(dest);
  i32_xor();
  register_set32(dest);
}

void MacroAssembler::byteSwap16SignExtend(Register reg) { MOZ_CRASH(); }

void MacroAssembler::byteSwap16ZeroExtend(Register reg) { MOZ_CRASH(); }

void MacroAssembler::byteSwap32(Register reg) { MOZ_CRASH(); }

void MacroAssembler::add32(Register src, Register dest) {
  register_get32(src);
  register_get32(dest);

  i32_add();
  register_set32(dest);
}

void MacroAssembler::add32(Imm32 imm, Register dest) {
  register_get32(dest);
  i32_const(imm.value);
  i32_add();
  register_set32(dest);
}

void MacroAssembler::add32(Imm32 imm, Register src, Register dest) {
  register_get32(src);
  i32_const(imm.value);
  i32_add();
  register_set32(dest);
}

void MacroAssembler::add32(Imm32 imm, const Address& dest) {
  wasmEvalOperand32(dest);

  wasmGetValue32(dest);
  i32_const(imm.value);
  i32_add();

  i32_store();
}

void MacroAssembler::addFloat32(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::addDouble(FloatRegister src, FloatRegister dest) {
  register_getf64(src);
  register_getf64(dest);
  f64_add();
  register_setf64(dest);
}

void MacroAssembler::sub32(const Address& src, Register dest) {
  wasmGetValue32(src);
  wasmGetValue32(dest);
  i32_sub();
  register_set32(dest);
}

void MacroAssembler::sub32(Register src, Register dest) {
  register_get32(dest);
  register_get32(src);
  i32_sub();
  register_set32(dest);
}

void MacroAssembler::sub32(Imm32 imm, Register dest) {
  register_get32(dest);
  i32_const(imm.value);
  i32_sub();
  register_set32(dest);
}

void MacroAssembler::subFloat32(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::subDouble(FloatRegister src, FloatRegister dest) {
  register_getf64(dest);
  register_getf64(src);
  f64_sub();
  register_setf64(dest);
}

void MacroAssembler::mul32(Register rhs, Register srcDest) {
  register_get32(srcDest);
  register_get32(rhs);
  i32_mul();
  register_set32(srcDest);
}

void MacroAssembler::mul32(Imm32 imm, Register srcDest) {
  register_get32(srcDest);
  i32_const(imm.value);
  i32_mul();
  register_set32(srcDest);
}

void MacroAssembler::mulFloat32(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::mulDouble(FloatRegister src, FloatRegister dest) {
  register_getf64(src);
  register_getf64(dest);
  f64_mul();
  register_setf64(dest);
}

void MacroAssembler::divFloat32(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::divDouble(FloatRegister src, FloatRegister dest) {
  register_getf64(dest);
  register_getf64(src);
  f64_div();
  register_setf64(dest);
}

void MacroAssembler::neg32(Register reg) {
  i32_const(0);
  register_get32(reg);
  i32_sub();
  register_set32(reg);
}

void MacroAssembler::negateFloat(FloatRegister reg) {
  register_getf32(reg);
  f32_neg();
  register_setf32(reg);
}

void MacroAssembler::negateDouble(FloatRegister reg) {
  register_getf64(reg);
  f64_neg();
  register_setf64(reg);
}

void MacroAssembler::abs32(Register src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::absFloat32(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::absDouble(FloatRegister src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::sqrtDouble(FloatRegister src, FloatRegister dest) {
  register_getf64(src);
  f64_sqrt();
  register_setf64(dest);
}

void MacroAssembler::lshift32(Imm32 shift, Register srcDest) {
  MOZ_RELEASE_ASSERT(0 <= shift.value && shift.value < 32);

  register_get32(srcDest);
  i32_const(shift.value);
  i32_shl();

  register_set32(srcDest);
}

void MacroAssembler::rshift32(Imm32 shift, Register srcDest) {
  MOZ_RELEASE_ASSERT(0 <= shift.value && shift.value < 32);

  register_get32(srcDest);
  i32_const(shift.value);
  i32_shr_u();
  register_set32(srcDest);
}

void MacroAssembler::rshift32(Register shift, Register srcDest) {
  register_get32(srcDest);
  register_get32(shift);
  i32_shr_u();
  register_set32(srcDest);
}

void MacroAssembler::rshift32Arithmetic(Imm32 shift, Register srcDest) {
  MOZ_RELEASE_ASSERT(0 <= shift.value && shift.value < 32);
  register_get32(srcDest);
  i32_const(shift.value);
  i32_shr_s();
  register_set32(srcDest);
}

void MacroAssembler::rshift32Arithmetic(Register shift, Register srcDest) {
  register_get32(srcDest);
  register_get32(shift);
  i32_shr_s();
  register_set32(srcDest);
}

void MacroAssembler::lshift32(Register shift, Register srcDest) {
  register_get32(srcDest);
  register_get32(shift);
  i32_shl();
  register_set32(srcDest);
}

void MacroAssembler::memoryBarrier(MemoryBarrierBits barrier) { MOZ_CRASH(); }

void MacroAssembler::clampIntToUint8(Register reg) { MOZ_CRASH(); }

template <class L>
void MacroAssembler::branchTest32(Condition cond, Register lhs, Register rhs,
                                  L label) {
  if (cond == Zero || cond == NonZero) {
    if (lhs == rhs) {
      register_get32(lhs);
    } else {
      register_get32(lhs);
      register_get32(rhs);
      i32_and();
    }
    emitCondition32(cond);
    j(label);
    return;
  }

  if (lhs == rhs && (cond == Signed || cond == NotSigned)) {
    register_get32(lhs);
    i32_const(0);

    if (cond == Signed) {
      i32_lt_s();
    } else {
      i32_ge_s();
    }

    j(label);
    return;
  }

  // NYI.
  MOZ_CRASH();
}

template <class L>
void MacroAssembler::branchTest32(Condition cond, Register lhs, Imm32 rhs,
                                  L label) {
  MOZ_RELEASE_ASSERT(cond == Zero || cond == NonZero);

  register_get32(lhs);
  i32_const(rhs.value);
  i32_and();
  if (cond == NonZero) {
    i32_const(0);
    i32_neq();
  } else {
    i32_eqz();
  }
  j(label);
}

void MacroAssembler::branchTest32(Condition cond, const Address& lhs, Imm32 rhs,
                                  Label* label) {
  wasmGetValue32(lhs);
  i32_const(rhs.value);
  i32_and();

  if (cond == Zero) {
    i32_eqz();
  } else if (cond == NonZero) {
    i32_const(0);
    i32_neq();
  } else {
    MOZ_CRASH();
  }

  j(label);
}

void MacroAssembler::branchTest32(Condition cond, const AbsoluteAddress& lhs,
                                  Imm32 rhs, Label* label) {
  MOZ_RELEASE_ASSERT(cond == Zero);

  i32_const(reinterpret_cast<int32_t>(lhs.addr));
  i32_load();

  i32_const(rhs.value);

  i32_eqz();
  j(label);
}

template <class L>
void MacroAssembler::branchTestPtr(Condition cond, Register lhs, Register rhs,
                                   L label) {
  branchTest32(cond, lhs, rhs, label);
}
void MacroAssembler::branchTestPtr(Condition cond, Register lhs, Imm32 rhs,
                                   Label* label) {
  branchTest32(cond, lhs, rhs, label);
}

void MacroAssembler::branchTestPtr(Condition cond, const Address& lhs,
                                   Imm32 rhs, Label* label) {
  branchTest32(cond, lhs, rhs, label);
}

template <class L>
void MacroAssembler::branchTest64(Condition cond, Register64 lhs,
                                  Register64 rhs, Register temp, L label) {
  MOZ_CRASH();
}

void MacroAssembler::branchTestNumber(Condition cond, Register tag,
                                      Label* label) {
  branchTestNumberImpl(cond, tag, label);
}

void MacroAssembler::branchTestNumber(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTestNumberImpl(cond, value, label);
}

template <typename T>
void MacroAssembler::branchTestNumberImpl(Condition cond, const T& t,
                                          Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  extractTagHelper32(t);
  i32_const(ImmTag(JS::detail::ValueUpperInclNumberTag).value);
  if (cond == Assembler::Equal) {
    i32_le_u();
  } else {
    i32_gt_u();
  }

  j(label);
}

void MacroAssembler::branchTestPrimitive(Condition cond, Register tag,
                                         Label* label) {
  branchTestPrimitiveImpl(cond, tag, label);
}

void MacroAssembler::branchTestPrimitive(Condition cond,
                                         const ValueOperand& value,
                                         Label* label) {
  branchTestPrimitiveImpl(cond, value, label);
}

template <typename T>
void MacroAssembler::branchTestPrimitiveImpl(Condition cond, const T& t,
                                             Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  extractTagHelper32(t);
  i32_const(ImmTag(JS::detail::ValueUpperExclPrimitiveTag).value);
  if (cond == Assembler::Equal) {
    i32_lt_u();
  } else {
    i32_ge_u();
  }

  j(label);
}

void MacroAssembler::branchTestUndefined(Condition cond, Register tag,
                                         Label* label) {
  branchTagNonDouble(JSVAL_TAG_UNDEFINED, tag, cond, label);
}

void MacroAssembler::branchTestUndefined(Condition cond, const Address& address,
                                         Label* label) {
  branchTagNonDouble(JSVAL_TAG_UNDEFINED, address, cond, label);
}

void MacroAssembler::branchTestUndefined(Condition cond,
                                         const BaseIndex& address,
                                         Label* label) {
  branchTagNonDouble(JSVAL_TAG_UNDEFINED, address, cond, label);
}

void MacroAssembler::branchTestUndefined(Condition cond,
                                         const ValueOperand& value,
                                         Label* label) {
  branchTagNonDouble(JSVAL_TAG_UNDEFINED, value, cond, label);
}

void MacroAssembler::branchTestInt32(Condition cond, Register tag,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_INT32, tag, cond, label);
}

void MacroAssembler::branchTestInt32(Condition cond, const Address& address,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_INT32, address, cond, label);
}

void MacroAssembler::branchTestInt32(Condition cond, const BaseIndex& address,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_INT32, address, cond, label);
}

void MacroAssembler::branchTestInt32(Condition cond, const ValueOperand& value,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_INT32, value, cond, label);
}

void MacroAssembler::branchTestInt32Truthy(bool truthy,
                                           const ValueOperand& value,
                                           Label* label) {
  extractPayloadFromi64(value);

  if (truthy) {
    i32_const(0);
    i32_neq();
  } else {
    i32_eqz();
  }

  j(label);
}

void MacroAssembler::branchTestDouble(Condition cond, const Address& address,
                                      Label* label) {
  branchTestDoubleImpl(cond, address, label);
}

void MacroAssembler::branchTestDouble(Condition cond, const BaseIndex& address,
                                      Label* label) {
  branchTestDoubleImpl(cond, address, label);
}

void MacroAssembler::branchTestDouble(Condition cond, Register tag,
                                      Label* label) {
  branchTestDoubleImpl(cond, tag, label);
}

void MacroAssembler::branchTestDouble(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTestDoubleImpl(cond, value, label);
}

template <typename T>
void MacroAssembler::branchTestDoubleImpl(Condition cond, const T& t,
                                          Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  extractTagHelper32(t);
  i32_const(ImmTag(JSVAL_TAG_CLEAR).value);

  if (cond == Assembler::Equal) {
    i32_lt_u();
  } else {
    i32_ge_u();
  }

  j(label);
}

void MacroAssembler::branchTestBoolean(Condition cond, Register tag,
                                       Label* label) {
  branchTagNonDouble(JSVAL_TAG_BOOLEAN, tag, cond, label);
}

void MacroAssembler::branchTestBoolean(Condition cond, const Address& address,
                                       Label* label) {
  branchTagNonDouble(JSVAL_TAG_BOOLEAN, address, cond, label);
}

void MacroAssembler::branchTestBoolean(Condition cond, const BaseIndex& address,
                                       Label* label) {
  branchTagNonDouble(JSVAL_TAG_BOOLEAN, address, cond, label);
}

void MacroAssembler::branchTestBoolean(Condition cond,
                                       const ValueOperand& value,
                                       Label* label) {
  branchTagNonDouble(JSVAL_TAG_BOOLEAN, value, cond, label);
}

void MacroAssembler::branchTestBooleanTruthy(bool truthy,
                                             const ValueOperand& value,
                                             Label* label) {
  extractPayloadHelper32(value);

  if (truthy) {
    i32_const(0);
    i32_neq();
  } else {
    i32_eqz();
  }

  j(label);
}

void MacroAssembler::branchTestString(Condition cond, Register tag,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_STRING, tag, cond, label);
}

void MacroAssembler::branchTestString(Condition cond, const Address& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_STRING, address, cond, label);
}

void MacroAssembler::branchTestString(Condition cond, const BaseIndex& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_STRING, address, cond, label);
}

void MacroAssembler::branchTestString(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_STRING, value, cond, label);
}

void MacroAssembler::branchTestStringTruthy(bool truthy,
                                            const ValueOperand& value,
                                            Label* label) {
  extractPayloadHelper32(value);
  i32_const(JSString::offsetOfLength());
  i32_load();

  if (truthy) {
    i32_const(0);
    i32_neq();
  } else {
    i32_eq();
  }

  j(label);
}

void MacroAssembler::branchTestSymbol(Condition cond, Register tag,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_SYMBOL, tag, cond, label);
}

void MacroAssembler::branchTestSymbol(Condition cond, const Address& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_SYMBOL, address, cond, label);
}

void MacroAssembler::branchTestSymbol(Condition cond, const BaseIndex& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_SYMBOL, address, cond, label);
}

void MacroAssembler::branchTestSymbol(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_SYMBOL, value, cond, label);
}

void MacroAssembler::branchTestBigInt(Condition cond, Register tag,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_BIGINT, tag, cond, label);
}

void MacroAssembler::branchTestBigInt(Condition cond, const Address& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_BIGINT, address, cond, label);
}

void MacroAssembler::branchTestBigInt(Condition cond, const BaseIndex& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_BIGINT, address, cond, label);
}

void MacroAssembler::branchTestBigInt(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_BIGINT, value, cond, label);
}

void MacroAssembler::branchTestBigIntTruthy(bool truthy,
                                            const ValueOperand& value,
                                            Label* label) {
  extractPayloadHelper32(value);
  i32_const(JS::BigInt::offsetOfDigitLength());
  i32_load();

  if (truthy) {
    i32_const(0);
    i32_neq();
  } else {
    i32_eq();
  }

  j(label);
}

void MacroAssembler::branchTestNull(Condition cond, Register tag,
                                    Label* label) {
  branchTagNonDouble(JSVAL_TAG_NULL, tag, cond, label);
}

void MacroAssembler::branchTestNull(Condition cond, const Address& address,
                                    Label* label) {
  branchTagNonDouble(JSVAL_TAG_NULL, address, cond, label);
}

void MacroAssembler::branchTestNull(Condition cond, const BaseIndex& address,
                                    Label* label) {
  branchTagNonDouble(JSVAL_TAG_NULL, address, cond, label);
}

void MacroAssembler::branchTestNull(Condition cond, const ValueOperand& value,
                                    Label* label) {
  branchTagNonDouble(JSVAL_TAG_NULL, value, cond, label);
}

void MacroAssembler::branchTestObject(Condition cond, Register tag,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_OBJECT, tag, cond, label);
}

void MacroAssembler::branchTestObject(Condition cond, const Address& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_OBJECT, address, cond, label);
}

void MacroAssembler::branchTestObject(Condition cond, const BaseIndex& address,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_OBJECT, address, cond, label);
}

void MacroAssembler::branchTestObject(Condition cond, const ValueOperand& value,
                                      Label* label) {
  branchTagNonDouble(JSVAL_TAG_OBJECT, value, cond, label);
}

void MacroAssembler::branchTestGCThing(Condition cond, const Address& address,
                                       Label* label) {
  branchTestGCThingImpl(cond, address, label);
}

void MacroAssembler::branchTestGCThing(Condition cond, const BaseIndex& address,
                                       Label* label) {
  branchTestGCThingImpl(cond, address, label);
}

void MacroAssembler::branchTestGCThing(Condition cond,
                                       const ValueOperand& value,
                                       Label* label) {
  branchTestGCThingImpl(cond, value, label);
}

template <typename T>
void MacroAssembler::branchTestGCThingImpl(Condition cond, const T& operand,
                                           Label* label) {
  MOZ_RELEASE_ASSERT(cond == Equal || cond == NotEqual);

  extractTagHelper32(operand);
  i32_const(ImmTag(JS::detail::ValueLowerInclGCThingTag).value);

  if (cond == Assembler::Equal) {
    i32_ge_u();
  } else {
    i32_lt_u();
  }

  j(label);
}

void MacroAssembler::branchTestMagic(Condition cond, Register tag,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_MAGIC, tag, cond, label);
}

void MacroAssembler::branchTestMagic(Condition cond, const Address& address,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_MAGIC, address, cond, label);
}

void MacroAssembler::branchTestMagic(Condition cond, const BaseIndex& address,
                                     Label* label) {
  branchTagNonDouble(JSVAL_TAG_MAGIC, address, cond, label);
}

void MacroAssembler::branchTestMagic(Condition cond, const Address& valaddr,
                                     JSWhyMagic why, Label* label) {
  MOZ_CRASH();
}

template <class L>
void MacroAssembler::branchTestMagic(Condition cond, const ValueOperand& value,
                                     L label) {
  branchTagNonDouble(JSVAL_TAG_MAGIC, value, cond, label);
}

void MacroAssembler::branchTestValue(Condition cond, const BaseIndex& lhs,
                                     const ValueOperand& rhs, Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

  wasmGetValue64(lhs);
  wasmGetValue64(rhs.scratchReg());
  emitCondition64(cond);
  j(label);
}

void MacroAssembler::branchTestDoubleTruthy(bool truthy, FloatRegister reg,
                                            Label* label) {
  if (reg.isDouble()) {
    register_getf64(reg);
    f64_const(0.0);

    if (truthy) {
      f64_neq();
    } else {
      f64_eq();
    }
  } else if (reg.isSingle()) {
    register_getf32(reg);
    f32_const(0.0);

    if (truthy) {
      f32_neq();
    } else {
      f32_eq();
    }
  } else {
    MOZ_CRASH();
  }

  j(label);
}

template <typename T>
void MacroAssemblerWasm32::fallibleUnboxPtrImpl(const T& src, Register dest,
                                                JSValueType type, Label* fail) {
  switch (type) {
    case JSVAL_TYPE_OBJECT:
      asMasm().branchTestObject(Assembler::NotEqual, src, fail);
      break;
    case JSVAL_TYPE_STRING:
      asMasm().branchTestString(Assembler::NotEqual, src, fail);
      break;
    case JSVAL_TYPE_SYMBOL:
      asMasm().branchTestSymbol(Assembler::NotEqual, src, fail);
      break;
    case JSVAL_TYPE_BIGINT:
      asMasm().branchTestBigInt(Assembler::NotEqual, src, fail);
      break;
    default:
      MOZ_CRASH("Unexpected type");
  }
  unboxNonDouble(src, dest, type);
}

void MacroAssembler::fallibleUnboxPtr(const ValueOperand& src, Register dest,
                                      JSValueType type, Label* fail) {
  fallibleUnboxPtrImpl(src, dest, type, fail);
}

void MacroAssembler::fallibleUnboxPtr(const Address& src, Register dest,
                                      JSValueType type, Label* fail) {
  fallibleUnboxPtrImpl(src, dest, type, fail);
}

void MacroAssembler::fallibleUnboxPtr(const BaseIndex& src, Register dest,
                                      JSValueType type, Label* fail) {
  fallibleUnboxPtrImpl(src, dest, type, fail);
}

void MacroAssembler::cmpPtrMovePtr(Condition cond, Register lhs, Register rhs,
                                   Register src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::cmpPtrMovePtr(Condition cond, Register lhs,
                                   const Address& rhs, Register src,
                                   Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::branch32(Condition cond, const BaseIndex& lhs, Imm32 rhs,
                              Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branch32(Condition cond, const AbsoluteAddress& lhs,
                              Imm32 rhs, Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branch32(Condition cond, const Address& lhs, Register rhs,
                              Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branch32(Condition cond, const Address& lhs, Imm32 rhs,
                              Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branch32(Condition cond, const AbsoluteAddress& lhs,
                              Register rhs, Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

template <class L>
void MacroAssembler::branch32(Condition cond, Register lhs, Register rhs,
                              L label) {
  branch32Impl(cond, lhs, rhs, label);
}

template <class L>
void MacroAssembler::branch32(Condition cond, Register lhs, Imm32 rhs,
                              L label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branch32(Condition cond, wasm::SymbolicAddress lhs,
                              Imm32 rhs, Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branch8(Condition cond, const Address& lhs, Imm32 rhs,
                             Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branch8(Condition cond, const BaseIndex& lhs, Register rhs,
                             Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branch16(Condition cond, const Address& lhs, Imm32 rhs,
                              Label* label) {
  MOZ_CRASH();
}

template <class L>
void MacroAssembler::branchPtr(Condition cond, Register lhs, Register rhs,
                               L label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond, Register lhs, Imm32 rhs,
                               Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond, Register lhs, ImmPtr rhs,
                               Label* label) {
  branch32Impl(cond, lhs, Imm32(reinterpret_cast<int32_t>(rhs.value)), label);
}

void MacroAssembler::branchPtr(Condition cond, Register lhs, ImmGCPtr rhs,
                               Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branchPtr(Condition cond, Register lhs, ImmWord rhs,
                               Label* label) {
  branch32Impl(cond, lhs, Imm32(static_cast<int32_t>(rhs.value)), label);
}

template <class L>
void MacroAssembler::branchPtr(Condition cond, const Address& lhs, Register rhs,
                               L label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond, const Address& lhs, ImmPtr rhs,
                               Label* label) {
  branch32Impl(cond, lhs, Imm32(reinterpret_cast<int32_t>(rhs.value)), label);
}

void MacroAssembler::branchPtr(Condition cond, const Address& lhs, ImmGCPtr rhs,
                               Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branchPtr(Condition cond, const Address& lhs, ImmWord rhs,
                               Label* label) {
  branch32Impl(cond, lhs, Imm32(static_cast<int32_t>(rhs.value)), label);
}

void MacroAssembler::branchPtr(Condition cond, const BaseIndex& lhs,
                               ImmWord rhs, Label* label) {
  branch32Impl(cond, lhs, Imm32(static_cast<int32_t>(rhs.value)), label);
}

void MacroAssembler::branchPtr(Condition cond, const BaseIndex& lhs,
                               Register rhs, Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond, const AbsoluteAddress& lhs,
                               Register rhs, Label* label) {
  branch32Impl(cond, lhs, rhs, label);
}

void MacroAssembler::branchPtr(Condition cond, const AbsoluteAddress& lhs,
                               ImmWord rhs, Label* label) {
  branch32Impl(cond, lhs, Imm32(static_cast<int32_t>(rhs.value)), label);
}

void MacroAssembler::branchPtr(Condition cond, wasm::SymbolicAddress lhs,
                               Register rhs, Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branchFloat(DoubleCondition cond, FloatRegister lhs,
                                 FloatRegister rhs, Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branchDouble(DoubleCondition cond, FloatRegister lhs,
                                  FloatRegister rhs, Label* label) {
  register_getf64(lhs);
  register_getf64(rhs);
  emitConditionf64(cond);
  j(label);
}

template <typename T>
void MacroAssembler::branchAdd32(Condition cond, T src, Register dest,
                                 Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Overflow);

  branch32Overflow(BinOperation::PLUS, src, dest, label);
}

template <typename T>
void MacroAssembler::branchSub32(Condition cond, T src, Register dest,
                                 Label* label) {
  if (cond == Assembler::Overflow) {
    return branch32Overflow(BinOperation::SUB, src, dest, label);
  }

  sub32(src, dest);

  register_get32(dest);
  emitCondition32(cond);
  j(label);
}

template <typename T>
void MacroAssembler::branchMul32(Condition cond, T src, Register dest,
                                 Label* label) {
  MOZ_RELEASE_ASSERT(cond == Assembler::Overflow);

  wasmGetValue32(src);
  i64_extend_i32_s();
  wasmGetValue32(dest);
  i64_extend_i32_s();
  i64_mul();

  i64_const(INT32_MAX);
  i64_gt_s();

  wasmGetValue32(src);
  i64_extend_i32_s();
  wasmGetValue32(src);
  i64_extend_i32_s();
  i64_mul();

  i64_const(INT32_MIN);
  i64_lt_s();

  i32_or();

  j(label);

  wasmGetValue32(src);
  wasmGetValue32(dest);
  i32_mul();
  register_set32(dest);
}

template <typename T>
void MacroAssembler::branchRshift32(Condition cond, T src, Register dest,
                                    Label* label) {
  MOZ_ASSERT(cond == Zero || cond == NonZero);
  rshift32(src, dest);

  register_get32(dest);
  emitCondition32(cond);
  j(label);
}

void MacroAssembler::branchNeg32(Condition cond, Register reg, Label* label) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::branchAddPtr(Condition cond, T src, Register dest,
                                  Label* label) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::branchSubPtr(Condition cond, T src, Register dest,
                                  Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branchMulPtr(Condition cond, Register src, Register dest,
                                  Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::decBranchPtr(Condition cond, Register lhs, Imm32 rhs,
                                  Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::spectreZeroRegister(Condition cond, Register scratch,
                                         Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::spectreMovePtr(Condition cond, Register src,
                                    Register dest) {
  MOZ_CRASH();
}

FaultingCodeOffset MacroAssembler::storeUncanonicalizedDouble(
    FloatRegister src, const Address& dest) {
  MOZ_CRASH();
}

FaultingCodeOffset MacroAssembler::storeUncanonicalizedDouble(
    FloatRegister src, const BaseIndex& dest) {
  MOZ_CRASH();
}

FaultingCodeOffset MacroAssembler::storeUncanonicalizedFloat32(
    FloatRegister src, const Address& dest) {
  MOZ_CRASH();
}

FaultingCodeOffset MacroAssembler::storeUncanonicalizedFloat32(
    FloatRegister src, const BaseIndex& dest) {
  MOZ_CRASH();
}

void MacroAssembler::addPtr(Imm32 imm, const Address& dest) {
  add32(imm, dest);
}

void MacroAssembler::addPtr(const Address& src, Register dest) { MOZ_CRASH(); }

void MacroAssembler::subPtr(Register src, const Address& dest) { MOZ_CRASH(); }

void MacroAssembler::subPtr(const Address& addr, Register dest) { MOZ_CRASH(); }

void MacroAssembler::branchTruncateFloat32MaybeModUint32(FloatRegister src,
                                                         Register dest,
                                                         Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::branchTruncateDoubleMaybeModUint32(FloatRegister src,
                                                        Register dest,
                                                        Label* fail) {
  branchTruncateDoubleToInt32(src, dest, fail);
}

void MacroAssembler::test32MovePtr(Condition cond, const Address& addr,
                                   Imm32 mask, Register src, Register dest) {
  MOZ_ASSERT(cond == Assembler::Zero || cond == Assembler::NonZero);

  wasmGetValue32(src);   // select true case
  wasmGetValue32(dest);  // select false case

  wasmGetValue32(addr);
  wasmGetValue32(mask);
  i32_and();
  emitCondition32(cond);

  select_instr();
  register_set32(dest);
}

void MacroAssembler::cmp32MovePtr(Condition cond, Register lhs, Imm32 rhs,
                                  Register src, Register dest) {
  wasmGetValue32(src);   // select true case
  wasmGetValue32(dest);  // select false case

  wasmGetValue32(lhs);
  wasmGetValue32(rhs);
  emitCondition32(cond);

  select_instr();
  register_set32(dest);
}

void MacroAssembler::test32LoadPtr(Condition cond, const Address& addr,
                                   Imm32 mask, const Address& src,
                                   Register dest) {
  MOZ_ASSERT(cond == Assembler::Zero || cond == Assembler::NonZero);

  wasmGetValue32(src);   // select true case
  wasmGetValue32(dest);  // select false case

  wasmGetValue32(addr);
  wasmGetValue32(mask);
  i32_and();
  emitCondition32(cond);

  select_instr();
  register_set32(dest);
}

void MacroAssembler::spectreBoundsCheck32(Register index, Register length,
                                          Register maybeScratch,
                                          Label* failure) {
  MOZ_ASSERT(!JitOptions.spectreIndexMasking);
  branch32(Assembler::BelowOrEqual, length, index, failure);
}

void MacroAssembler::spectreBoundsCheck32(Register index, const Address& length,
                                          Register maybeScratch,
                                          Label* failure) {
  MOZ_RELEASE_ASSERT(!JitOptions.spectreIndexMasking);
  branch32(Assembler::BelowOrEqual, length, index, failure);
}

void MacroAssembler::spectreBoundsCheckPtr(Register index, Register length,
                                           Register maybeScratch,
                                           Label* failure) {
  MOZ_CRASH();
}

void MacroAssembler::spectreBoundsCheckPtr(Register index,
                                           const Address& length,
                                           Register maybeScratch,
                                           Label* failure) {
  MOZ_CRASH();
}

void MacroAssembler::cmp32Load32(Condition cond, Register lhs, Imm32 rhs,
                                 const Address& src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::cmp32LoadPtr(Condition cond, const Address& lhs, Imm32 rhs,
                                  const Address& src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::branchTruncateFloat32ToInt32(FloatRegister src,
                                                  Register dest, Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::branchTruncateDoubleToInt32(FloatRegister src,
                                                 Register dst, Label* fail) {
  truncateDoubleToInt32(src, dst, fail);
}

void MacroAssembler::mulDoublePtr(ImmPtr imm, Register temp,
                                  FloatRegister dst) {
  wasmEvalOperand32(imm);
  f64_load();
  register_getf64(dst);
  f64_mul();
  register_setf64(dst);
}

template <typename T1, typename T2>
void MacroAssembler::cmp32Set(Condition cond, T1 lhs, T2 rhs, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::cmp64Set(Condition cond, Address lhs, Imm64 rhs,
                              Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::cmp32Move32(Condition cond, Register lhs, Imm32 rhs,
                                 Register src, Register dest) {
  cmp32Move32Impl(cond, lhs, rhs, src, dest);
}

void MacroAssembler::cmp32Move32(Condition cond, Register lhs, Register rhs,
                                 Register src, Register dest) {
  cmp32Move32Impl(cond, lhs, rhs, src, dest);
}

void MacroAssembler::cmp32Move32(Condition cond, Register lhs,
                                 const Address& rhs, Register src,
                                 Register dest) {
  cmp32Move32Impl(cond, lhs, rhs, src, dest);
}

void MacroAssembler::branchAdd64(Condition cond, Imm64 imm, Register64 dest,
                                 Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::quotient32(Register rhs, Register srcDest,
                                bool isUnsigned) {
  MOZ_CRASH();
}

void MacroAssembler::remainder32(Register rhs, Register srcDest,
                                 bool isUnsigned) {
  MOZ_CRASH();
}

void MacroAssembler::branch64(Condition cond, Register64 lhs, Imm64 val,
                              Label* success, Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::branch64(Condition cond, Register64 lhs, Register64 rhs,
                              Label* success, Label* fail) {
  MOZ_CRASH();
}

void MacroAssembler::branch64(Condition cond, const Address& lhs, Imm64 val,
                              Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branch64(Condition cond, const Address& lhs,
                              Register64 rhs, Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::branch64(Condition cond, const Address& lhs,
                              const Address& rhs, Register scratch,
                              Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::minFloat32(FloatRegister other, FloatRegister srcDest,
                                bool handleNaN) {
  register_getf32(srcDest);
  register_getf32(other);
  f32_min();
  register_setf32(srcDest);
}

void MacroAssembler::minDouble(FloatRegister other, FloatRegister srcDest,
                               bool handleNaN) {
  register_getf64(srcDest);
  register_getf64(other);
  f64_min();
  register_setf64(srcDest);
}

void MacroAssembler::maxFloat32(FloatRegister other, FloatRegister srcDest,
                                bool handleNaN) {
  register_getf32(srcDest);
  register_getf32(other);
  f32_max();
  register_setf32(srcDest);
}

void MacroAssembler::maxDouble(FloatRegister other, FloatRegister srcDest,
                               bool handleNaN) {
  register_getf64(srcDest);
  register_getf64(other);
  f64_max();
  register_setf64(srcDest);
}

void MacroAssembler::rotateLeft(Imm32 count, Register input, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::rotateLeft(Register count, Register input, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::rotateLeft64(Imm32 count, Register64 input,
                                  Register64 dest, Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::rotateLeft64(Register count, Register64 input,
                                  Register64 dest, Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::rotateRight(Imm32 count, Register input, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::rotateRight(Register count, Register input,
                                 Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::rotateRight64(Imm32 count, Register64 input,
                                   Register64 dest, Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::rotateRight64(Register count, Register64 input,
                                   Register64 dest, Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::flexibleLshift32(Register shift, Register srcDest) {
  lshift32(shift, srcDest);
}

void MacroAssembler::flexibleRshift32(Register shift, Register srcDest) {
  rshift32(shift, srcDest);
}

void MacroAssembler::flexibleRshift32Arithmetic(Register shift,
                                                Register srcDest) {
  rshift32Arithmetic(shift, srcDest);
}

void MacroAssembler::branchPrivatePtr(Condition cond, const Address& lhs,
                                      Register rhs, Label* label) {
  MOZ_CRASH();
}

void MacroAssembler::clz32(Register src, Register dest, bool knownNotZero) {
  MOZ_CRASH();
}

void MacroAssembler::ctz32(Register src, Register dest, bool knownNotZero) {
  MOZ_CRASH();
}

void MacroAssembler::popcnt32(Register src, Register dest, Register temp) {
  MOZ_CRASH();
}

void MacroAssembler::moveFloat32ToGPR(FloatRegister src, Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::moveGPRToFloat32(Register src, FloatRegister dest) {
  MOZ_CRASH();
}

void MacroAssembler::sqrtFloat32(FloatRegister src, FloatRegister dest) {
  register_getf32(src);
  f32_sqrt();
  register_setf32(dest);
}

void MacroAssembler::cmp16Set(Condition cond, Address lhs, Imm32 rhs,
                              Register dest) {
  MOZ_CRASH();
}

void MacroAssembler::cmp8Set(Condition cond, Address lhs, Imm32 rhs,
                             Register dest) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::testNumberSet(Condition cond, const T& src,
                                   Register dest) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::testBooleanSet(Condition cond, const T& src,
                                    Register dest) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::testStringSet(Condition cond, const T& src,
                                   Register dest) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::testSymbolSet(Condition cond, const T& src,
                                   Register dest) {
  MOZ_CRASH();
}

template <typename T>
void MacroAssembler::testBigIntSet(Condition cond, const T& src,
                                   Register dest) {
  MOZ_CRASH();
}

//}}} check_macroassembler_style

}  // namespace js::jit

#endif /* jit_wasm32_MacroAssembler_wasm32_inl_h */
