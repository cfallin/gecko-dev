/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_Assembler_wasm32_h
#define jit_wasm32_Assembler_wasm32_h

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "jit/CompactBuffer.h"
#include "jit/Registers.h"
#include "jit/RegisterSets.h"
#include "jit/shared/Assembler-shared.h"
#include "jit/wasm32/Architecture-wasm32.h"
#include "jit/wasm32/ir/IR.h"
#include "js/Value.h"

namespace js::jit {

struct ImmTag : public Imm32 {
  explicit ImmTag(JSValueTag mask) : Imm32(int32_t(mask)) {}
};

struct ImmType : public ImmTag {
  explicit ImmType(JSValueType type) : ImmTag(JSVAL_TYPE_TO_TAG(type)) {}
};

class MacroAssembler;

static constexpr Register wax{Registers::wax_64};
static constexpr Register wbx{Registers::wbx_64};
static constexpr Register wbp{Registers::wbp};
static constexpr Register wcx{Registers::wcx_64};
static constexpr Register wdx{Registers::wdx_64};
static constexpr Register wdi{Registers::wdi_64};
static constexpr Register wsi{Registers::wsi_64};
static constexpr Register wsp{Registers::wsp};
static constexpr Register w0{Registers::w0};

static constexpr Register InvalidReg{Registers::invalid_reg};
static constexpr Register InvalidReg2{Registers::invalid_reg2};

// Wasm32 is a 32 bit architecture but it has 64 bit types.
// So, we represent JSReturnOpernad(JSReturnReg_Type, JSReturnReg_Data)
// as a single 64 bit register wcx.
static constexpr Register JSReturnReg_Type = InvalidReg2;
static constexpr Register JSReturnReg_Data = wcx;
static constexpr Register StackPointer = wsp;
static constexpr Register FramePointer = wbp;
static constexpr Register ReturnReg = wax;

static constexpr FloatRegister f0 = FloatRegister(FloatRegisters::f0);
static constexpr FloatRegister f1 = FloatRegister(FloatRegisters::f1);
static constexpr FloatRegister f2 = FloatRegister(FloatRegisters::f2);
static constexpr FloatRegister f3 = FloatRegister(FloatRegisters::f3);
static constexpr FloatRegister f4 = FloatRegister(FloatRegisters::f4);
static constexpr FloatRegister f5 = FloatRegister(FloatRegisters::f5);
static constexpr FloatRegister f6 = FloatRegister(FloatRegisters::f6);
static constexpr FloatRegister f7 = FloatRegister(FloatRegisters::f7);

static constexpr FloatRegister d0 = FloatRegister(FloatRegisters::d0);
static constexpr FloatRegister d1 = FloatRegister(FloatRegisters::d1);
static constexpr FloatRegister d2 = FloatRegister(FloatRegisters::d2);
static constexpr FloatRegister d3 = FloatRegister(FloatRegisters::d3);
static constexpr FloatRegister d4 = FloatRegister(FloatRegisters::d4);
static constexpr FloatRegister d5 = FloatRegister(FloatRegisters::d5);
static constexpr FloatRegister d6 = FloatRegister(FloatRegisters::d6);
static constexpr FloatRegister d7 = FloatRegister(FloatRegisters::d7);

static constexpr FloatRegister ReturnFloat32Reg = {FloatRegisters::d0};
static constexpr FloatRegister ReturnDoubleReg = {FloatRegisters::d0};
static constexpr FloatRegister ReturnSimd128Reg = {FloatRegisters::invalid_reg};
static constexpr FloatRegister ScratchSimd128Reg = {
    FloatRegisters::invalid_reg};
static constexpr FloatRegister InvalidFloatReg = {FloatRegisters::invalid_reg};

static constexpr Register ScratchReg = w0;

// Helper class for ScratchRegister usage. Asserts that only one piece
// of code thinks it has exclusive ownership of the scratch register.
struct ScratchRegisterScope : public AutoRegisterScope {
  explicit ScratchRegisterScope(MacroAssembler& masm)
      : AutoRegisterScope(masm, ScratchReg) {}
};

static constexpr FloatRegister ScratchFloat32Reg_ = {FloatRegisters::f7};

struct ScratchFloat32Scope : public AutoFloatRegisterScope {
  explicit ScratchFloat32Scope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchFloat32Reg_) {}
};

static constexpr FloatRegister ScratchDoubleReg_ = {FloatRegisters::d7};

struct ScratchDoubleScope : public AutoFloatRegisterScope {
  explicit ScratchDoubleScope(MacroAssembler& masm)
      : AutoFloatRegisterScope(masm, ScratchDoubleReg_) {}
};

static constexpr Register OsrFrameReg{Registers::wdi_64};
static constexpr Register PreBarrierReg{Registers::wdi_64};

static constexpr Register InterpreterPCReg{Registers::invalid_reg};
static constexpr Register CallTempReg0{Registers::invalid_reg};
static constexpr Register CallTempReg1{Registers::invalid_reg};
static constexpr Register CallTempReg2{Registers::invalid_reg};
static constexpr Register CallTempReg3{Registers::invalid_reg};
static constexpr Register CallTempReg4{Registers::invalid_reg};
static constexpr Register CallTempReg5{Registers::invalid_reg};
static constexpr Register CallTempNonArgRegs[] = {InvalidReg, InvalidReg};
static const uint32_t NumCallTempNonArgRegs = std::size(CallTempNonArgRegs);

static constexpr Register IntArgReg0{Registers::invalid_reg};
static constexpr Register IntArgReg1{Registers::invalid_reg};
static constexpr Register IntArgReg2{Registers::invalid_reg};
static constexpr Register IntArgReg3{Registers::invalid_reg};
static constexpr Register HeapReg{Registers::invalid_reg};

static constexpr Register RegExpMatcherRegExpReg{Registers::invalid_reg};
static constexpr Register RegExpMatcherStringReg{Registers::invalid_reg};
static constexpr Register RegExpMatcherLastIndexReg{Registers::invalid_reg};

static constexpr Register RegExpExecTestRegExpReg{Registers::invalid_reg};
static constexpr Register RegExpExecTestStringReg{Registers::invalid_reg};

static constexpr Register RegExpSearcherRegExpReg{Registers::invalid_reg};
static constexpr Register RegExpSearcherStringReg{Registers::invalid_reg};
static constexpr Register RegExpSearcherLastIndexReg{Registers::invalid_reg};

#if defined(JS_NUNBOX32)
static constexpr ValueOperand JSReturnOperand(InvalidReg, wcx);
static constexpr Register64 ReturnReg64(InvalidReg,
                                        Register{Registers::invalid_reg2});
#elif defined(JS_PUNBOX64)
static constexpr ValueOperand JSReturnOperand(InvalidReg);
static constexpr Register64 ReturnReg64(InvalidReg);
#else
#  error "Bad architecture"
#endif

static constexpr Register ABINonArgReg0{Registers::invalid_reg};
static constexpr Register ABINonArgReg1{Registers::invalid_reg};
static constexpr Register ABINonArgReg2{Registers::invalid_reg};
static constexpr Register ABINonArgReg3{Registers::invalid_reg};
static constexpr Register ABINonArgReturnReg0{Registers::invalid_reg};
static constexpr Register ABINonArgReturnReg1{Registers::invalid_reg};
static constexpr Register ABINonVolatileReg{Registers::invalid_reg};
static constexpr Register ABINonArgReturnVolatileReg{Registers::invalid_reg};

static constexpr FloatRegister ABINonArgDoubleReg = {FloatRegisters::d0};

static constexpr Register WasmTableCallScratchReg0{Registers::invalid_reg};
static constexpr Register WasmTableCallScratchReg1{Registers::invalid_reg};
static constexpr Register WasmTableCallSigReg{Registers::invalid_reg};
static constexpr Register WasmTableCallIndexReg{Registers::invalid_reg};
static constexpr Register InstanceReg{Registers::invalid_reg};
static constexpr Register WasmJitEntryReturnScratch{Registers::invalid_reg};
static constexpr Register WasmCallRefCallScratchReg0{Registers::invalid_reg};
static constexpr Register WasmCallRefCallScratchReg1{Registers::invalid_reg};
static constexpr Register WasmCallRefReg{Registers::invalid_reg};
static constexpr Register WasmTailCallInstanceScratchReg{
    Registers::invalid_reg};
static constexpr Register WasmTailCallRAScratchReg{Registers::invalid_reg};
static constexpr Register WasmTailCallFPScratchReg{Registers::invalid_reg};

static constexpr uint32_t ABIStackAlignment = 16;
static constexpr uint32_t CodeAlignment = 16;
static constexpr uint32_t JitStackAlignment = 16;
static constexpr uint32_t JitStackValueAlignment =
    JitStackAlignment / sizeof(Value);
static_assert(JitStackAlignment % sizeof(Value) == 0 &&
                  JitStackValueAlignment >= 1,
              "Stack alignment should be a non-zero multiple of sizeof(Value)");

static const Scale ScalePointer = TimesFour;

constexpr uint32_t Int8SizeLog2 = 0;
constexpr uint32_t Int16SizeLog2 = 1;
constexpr uint32_t Int32SizeLog2 = 2;
constexpr uint32_t Int64SizeLog2 = 3;

class AssemblerWasm32 : public AssemblerShared {
 public:
  enum Condition {
    Equal,
    NotEqual,
    Above,
    AboveOrEqual,
    Below,
    BelowOrEqual,
    GreaterThan,
    GreaterThanOrEqual,
    LessThan,
    LessThanOrEqual,
    Overflow,
    CarrySet,
    CarryClear,
    Signed,
    NotSigned,
    Zero,
    NonZero,
    Always,
  };

  enum DoubleCondition {
    DoubleOrdered,
    DoubleEqual,
    DoubleNotEqual,
    DoubleGreaterThan,
    DoubleGreaterThanOrEqual,
    DoubleLessThan,
    DoubleLessThanOrEqual,
    DoubleUnordered,
    DoubleEqualOrUnordered,
    DoubleNotEqualOrUnordered,
    DoubleGreaterThanOrUnordered,
    DoubleGreaterThanOrEqualOrUnordered,
    DoubleLessThanOrUnordered,
    DoubleLessThanOrEqualOrUnordered
  };

  static Condition InvertCondition(Condition);

  static DoubleCondition InvertCondition(DoubleCondition) { MOZ_CRASH(); }

  uint32_t size() const { return codeBuffer_.size(); }

  void executableCopy(void* buffer);

  void finish();

  void i32_le_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_LE_U{}});
  }

  void i32_le_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_LE_S{}});
  }

  void i32_lt_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_LT_U{}});
  }

  void i32_lt_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_LT_S{}});
  }

  void i32_ge_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_GE_U{}});
  }

  void i32_ge_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_GE_S{}});
  }

  void i32_gt_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_GT_U{}});
  }

  void i32_gt_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_GT_S{}});
  }

  void select_instr() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::SELECT_INSTR{}});
  }

  void unreachable_instr() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::UNREACHABLE{}});
  }

  void drop_instr() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::DROP{}});
  }

  void wasm_call(uint32_t functionIndex) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::CALL_INSTR{functionIndex}});
  }

  void call_indirect(uint32_t typeIndex, uint32_t tableIndex = 0u) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::CALL_INDIRECT{typeIndex, tableIndex}});
  }

  void return_call_indirect(uint32_t typeIndex, uint32_t tableIndex = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::RETURN_CALL_INDIRECT{typeIndex, tableIndex}});
  }

  void return_instr() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::RETURN_INSTR{}});
  }

  void br_instr(uint32_t blockIndex) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::BR_INSTR{blockIndex, mozilla::Nothing{}}});
  }

  void br_if(uint32_t targetBlockIndex, uint32_t fallthroughBlockIndex) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::BR_INSTR{
        targetBlockIndex, mozilla::Some(fallthroughBlockIndex)}});
  }

  void br_table(std::vector<uint32_t> indices) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::BR_TABLE{std::move(indices)}});
  }

  void wasm_nop() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::NOP_INSTR{}});
  }

  void end_instr() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::END_INSTR{}});
  }

  void f32_add() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_ADD{}});
  }

  void f32_sub() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_SUB{}});
  }

  void f32_mul() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_MUL{}});
  }

  void f32_div() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_DIV{}});
  }

  void f32_const(float value) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F32_CONST{value}});
  }

  void f32_lt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_LT{}});
  }

  void f32_le() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_LE{}});
  }

  void f32_gt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_GT{}});
  }

  void f32_ge() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_GE{}});
  }

  void f32_eq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_EQ{}});
  }

  void f32_neq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_NEQ{}});
  }

  void f32_min() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_MIN{}});
  }

  void f32_max() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_MAX{}});
  }

  void f64_add() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_ADD{}});
  }

  void f64_sub() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_SUB{}});
  }

  void f64_mul() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_MUL{}});
  }

  void f64_div() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_DIV{}});
  }

  void i32_reinterpret_f32() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I32_REINTERPRET_F32{}});
  }

  void i64_reinterpret_f64() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_REINTERPRET_F64{}});
  }

  void f64_reinterpret_i64() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_REINTERPRET_I64{}});
  }

  void f64_convert_i32_s() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_CONVERT_I32_S{}});
  }

  void f64_convert_i64_s() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_CONVERT_I64_S{}});
  }

  void f64_copysign() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_COPYSIGN{}});
  }

  void f32_trunc() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_TRUNC{}});
  }

  void f64_trunc() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_TRUNC{}});
  }

  void f32_floor() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_FLOOR{}});
  }

  void f64_floor() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_FLOOR{}});
  }

  void f32_neg() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_NEG{}});
  }

  void f64_neg() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_NEG{}});
  }

  void f32_sqrt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F32_SQRT{}});
  }

  void f64_sqrt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_SQRT{}});
  }

  void f64_lt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_LT{}});
  }

  void f64_le() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_LE{}});
  }

  void f64_gt() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_GT{}});
  }

  void f64_ge() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_GE{}});
  }

  void f64_eq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_EQ{}});
  }

  void f64_neq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_NEQ{}});
  }

  void f64_min() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_MIN{}});
  }

  void f64_max() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::F64_MAX{}});
  }

  void f32_load(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::F32_LOAD{wasm32::MemoryArgument{Int32SizeLog2, offset}}});
  }

  void f32_store(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::F32_STORE{wasm32::MemoryArgument{Int32SizeLog2, offset}}});
  }

  void f64_load(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::F64_LOAD{wasm32::MemoryArgument{Int64SizeLog2, offset}}});
  }

  void f64_store(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::F64_STORE{wasm32::MemoryArgument{Int64SizeLog2, offset}}});
  }

  void f64_const(double value) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_CONST{value}});
  }

  void f32_nearest() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F32_NEAREST{}});
  }

  void f64_nearest() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::F64_NEAREST{}});
  }

  void global_get(uint32_t globalIndex) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::GLOBAL_GET{globalIndex}});
  }

  void global_set(uint32_t globalIndex) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::GLOBAL_SET{globalIndex}});
  }

  void i32_store(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_STORE{wasm32::MemoryArgument{Int32SizeLog2, offset}}});
  }

  void i32_store8(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_STORE_8{wasm32::MemoryArgument{Int8SizeLog2, offset}}});
  }

  void i32_store16(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_STORE_16{wasm32::MemoryArgument{Int16SizeLog2, offset}}});
  }

  void i32_load(uint32_t offset = 0u) {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_LOAD{wasm32::MemoryArgument{Int32SizeLog2, offset}}});
  }

  void i32_load8_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_LOAD_8U{wasm32::MemoryArgument{Int8SizeLog2, 0}}});
  }

  void i32_load8_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_LOAD_8S{wasm32::MemoryArgument{Int8SizeLog2, 0}}});
  }

  void i32_load16_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_LOAD_16U{wasm32::MemoryArgument{Int16SizeLog2, 0}}});
  }

  void i32_load16_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{
        wasm32::I32_LOAD_16S{wasm32::MemoryArgument{Int16SizeLog2, 0}}});
  }

  void i32_const(int32_t value) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I32_CONST{value}});
  }

  void i32_add() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_ADD{}});
  }
  void i32_sub() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_SUB{}});
  }
  void i32_mul() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_MUL{}});
  }
  void i32_div_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_DIV_U{}});
  }
  void i32_div_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_DIV_S{}});
  }
  void i32_rem_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_REM_U{}});
  }
  void i32_rem_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_REM_S{}});
  }
  void i32_and() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_AND{}});
  }
  void i32_or() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_OR{}});
  }
  void i32_xor() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_XOR{}});
  }
  void i32_shl() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_SHL{}});
  }
  void i32_shr_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_SHR_U{}});
  }
  void i32_shr_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_SHR_S{}});
  }
  void i32_eq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_EQ{}});
  }
  void i32_neq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_NEQ{}});
  }
  void i32_eqz() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I32_EQZ{}});
  }

  void i64_const(int64_t value) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_CONST{value}});
  }

  void i64_gt_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_GT_S{}});
  }
  void i64_gt_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_GT_U{}});
  }
  void i64_ge_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_GE_S{}});
  }
  void i64_ge_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_GE_S{}});
  }

  void i64_lt_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_LT_S{}});
  }
  void i64_lt_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_LT_U{}});
  }
  void i64_le_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_LE_S{}});
  }
  void i64_le_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_LE_U{}});
  }

  void i64_load(uint32_t offset = 0u) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_LOAD{Int64SizeLog2, offset}});
  }

  void i64_store(uint32_t offset = 0u) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_STORE{Int64SizeLog2, offset}});
  }

  void i64_eq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_EQ{}});
  }
  void i64_eqz() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_EQZ{}});
  }
  void i64_neq() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_NEQ{}});
  }

  void i32_wrap_i64() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I32_WRAP_I64{}});
  }
  void i32_trunc_f64_s() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I32_TRUNC_F64_S{}});
  }

  void i64_extend_i32_s() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_EXTEND_I32_S{}});
  }
  void i64_extend_i32_u() {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::I64_EXTEND_I32_U{}});
  }
  void i64_add() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_ADD{}});
  }
  void i64_sub() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_SUB{}});
  }
  void i64_mul() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_MUL{}});
  }
  void i64_or() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_OR{}});
  }
  void i64_and() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_AND{}});
  }
  void i64_shr_u() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_SHR_U{}});
  }
  void i64_shr_s() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_SHR_S{}});
  }
  void i64_shl() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_SHL{}});
  }
  void i64_xor() {
    currentBlock_->addInstruction(wasm32::WasmInstruction{wasm32::I64_XOR{}});
  }

  void local_get(uint32_t localIndex) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::LOCAL_GET{localIndex}});
  }

  void local_set(uint32_t localIndex) {
    currentBlock_->addInstruction(
        wasm32::WasmInstruction{wasm32::LOCAL_SET{localIndex}});
  }

  void register_get(Register reg);
  void register_set(Register reg);
  void register_get(FloatRegister reg);
  void register_set(FloatRegister reg);

  void register_get32(Register reg);
  void register_set32(Register reg);

  void register_get64(Register reg);
  void register_set64(Register reg);

  void register_getf32(FloatRegister reg);
  void register_setf32(FloatRegister reg);

  void register_getf64(FloatRegister reg);
  void register_setf64(FloatRegister reg);

  uint32_t registerToGlobalIndex(Register r) {
    return static_cast<uint32_t>(r.reg_);
  }

  uint32_t registerToGlobalIndex(FloatRegister r) {
    return static_cast<uint32_t>(r.code());
  }

  void local_get(Register r) {
    local_get(numberOfParameters() + Registers::ToLocalIndex(r.code()));
  }
  void local_set(Register r) {
    local_set(numberOfParameters() + Registers::ToLocalIndex(r.code()));
  }
  void global_get(Register r) { global_get(registerToGlobalIndex(r)); }
  void global_set(Register r) { global_set(registerToGlobalIndex(r)); }

  void local_get(FloatRegister r) {
    local_get(numberOfParameters() + Registers::NumLocals +
              FloatRegisters::ToLocalIndex(r.code()));
  }
  void local_set(FloatRegister r) {
    local_set(numberOfParameters() + Registers::NumLocals +
              FloatRegisters::ToLocalIndex(r.code()));
  }
  void global_get(FloatRegister r) { global_get(registerToGlobalIndex(r)); }
  void global_set(FloatRegister r) { global_set(registerToGlobalIndex(r)); }

  void emitCondition32(Condition cond);
  void emitCondition64(Condition cond);
  void emitConditionf64(DoubleCondition cond);

  bool is64BitReg(Register r) const { return Registers::Is64Bit(r.code()); }

  void introduceSignature(wasm32::FunctionSignature signature);
  uint32_t introduceLocal(wasm32::WasmLocalType type);

  void introduceLocalsForRegisters();

  uint32_t numberOfParameters() const {
    return funcSignature_.parameterTypes.size();
  }

  uint32_t curBlockId() const { return currentBlock_->id; }

  void resetCurrentCFG();

  void ensureNewBlockWithLink();
  void ensureNewBlock();

  wasm32::BasicBlock* createBlock();

  wasm32::BasicBlock* root_;
  wasm32::BasicBlock* currentBlock_;
  wasm32::FunctionSignature funcSignature_;
  std::vector<wasm32::WasmLocalType> locals_;
  uint32_t nextBlockId_;
  std::map<uint32_t, std::unique_ptr<wasm32::BasicBlock>> blocks_;

  std::vector<uint8_t> codeBuffer_;
};

class Assembler : public AssemblerWasm32 {
 public:
  template <typename T, typename S>
  static void PatchDataWithValueCheck(CodeLocationLabel, T, S) {
    MOZ_CRASH();
  }
  static void PatchWrite_Imm32(CodeLocationLabel, Imm32) { MOZ_CRASH(); }

  static void PatchWrite_NearCall(CodeLocationLabel, CodeLocationLabel) {
    MOZ_CRASH();
  }
  static uint32_t PatchWrite_NearCallSize() { MOZ_CRASH(); }

  static void ToggleToJmp(CodeLocationLabel) { MOZ_CRASH(); }
  static void ToggleToCmp(CodeLocationLabel) { MOZ_CRASH(); }
  static void ToggleCall(CodeLocationLabel, bool) { MOZ_CRASH(); }

  static void Bind(uint8_t*, const CodeLabel&) { MOZ_CRASH(); }

  static uintptr_t GetPointer(uint8_t*) { MOZ_CRASH(); }

  static bool HasRoundInstruction(RoundingMode) { return false; }

  void verifyHeapAccessDisassembly(uint32_t begin, uint32_t end,
                                   const Disassembler::HeapAccess& heapAccess) {
    MOZ_CRASH();
  }

  void writeDataRelocation(ImmGCPtr ptr);

  void setUnlimitedBuffer() { MOZ_CRASH(); }

 protected:
  CompactBufferWriter dataRelocations_;
};

class Operand {
 public:
  explicit Operand(const Address&) { MOZ_CRASH(); }
  explicit Operand(const Register) { MOZ_CRASH(); }
  explicit Operand(const FloatRegister) { MOZ_CRASH(); }
  explicit Operand(Register, Imm32) { MOZ_CRASH(); }
  explicit Operand(Register, int32_t) { MOZ_CRASH(); }
};

class ABIArgGenerator {
  ABIArg current_;
  uint32_t nextArgIndex_;

 public:
  ABIArgGenerator();
  ABIArg next(MIRType type);
  ABIArg& current();
  uint32_t stackBytesConsumedSoFar() const;
  void increaseStackOffset(uint32_t);
};

}  // namespace js::jit

#endif /* jit_wasm32_Assembler_wasm32_h */
