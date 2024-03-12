/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_ir_ir_h
#define jit_wasm32_ir_ir_h

#include "mozilla/Assertions.h"
#include "mozilla/Maybe.h"
#include "mozilla/Variant.h"

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace js::jit::wasm32 {

struct MemoryArgument {
  uint32_t align;
  uint32_t offset;
};

struct I32_CONST {
  int32_t value;
};

struct I32_STORE {
  MemoryArgument memarg;
};

struct I32_STORE_8 {
  MemoryArgument memarg;
};

struct I32_STORE_16 {
  MemoryArgument memarg;
};

struct I32_LOAD {
  MemoryArgument memarg;
};

struct I32_LOAD_8U {
  MemoryArgument memarg;
};

struct I32_LOAD_8S {
  MemoryArgument memarg;
};

struct I32_LOAD_16U {
  MemoryArgument memarg;
};

struct I32_LOAD_16S {
  MemoryArgument memarg;
};

struct LOCAL_GET {
  uint32_t localIndex;
};

struct LOCAL_SET {
  uint32_t localIndex;
};

struct GLOBAL_GET {
  uint32_t globalIndex;
};

struct GLOBAL_SET {
  uint32_t globalIndex;
};

struct I32_ADD {};
struct I32_SUB {};
struct I32_MUL {};
struct I32_DIV_U {};
struct I32_DIV_S {};
struct I32_REM_U {};
struct I32_REM_S {};
struct I32_AND {};
struct I32_OR {};
struct I32_XOR {};
struct I32_SHL {};
struct I32_SHR_U {};
struct I32_SHR_S {};
struct I32_EQ {};
struct I32_NEQ {};
struct I32_EQZ {};
struct I32_LT_U {};
struct I32_LT_S {};
struct I32_LE_U {};
struct I32_LE_S {};
struct I32_GE_U {};
struct I32_GE_S {};
struct I32_GT_U {};
struct I32_GT_S {};

struct I64_CONST {
  int64_t value;
};

struct I64_LOAD {
  MemoryArgument memarg;
};

struct I64_STORE {
  MemoryArgument memarg;
};

struct I64_EQ {};
struct I64_EQZ {};
struct I64_NEQ {};
struct I32_WRAP_I64 {};
struct I32_TRUNC_F64_S {};
struct I64_EXTEND_I32_S {};
struct I64_EXTEND_I32_U {};
struct I64_ADD {};
struct I64_SUB {};
struct I64_MUL {};
struct I64_OR {};
struct I64_AND {};
struct I64_XOR {};
struct I64_SHR_U {};
struct I64_SHR_S {};
struct I64_SHL {};
struct I64_GT_S {};
struct I64_GT_U {};
struct I64_GE_S {};
struct I64_GE_U {};
struct I64_LT_S {};
struct I64_LT_U {};
struct I64_LE_S {};
struct I64_LE_U {};
struct SELECT_INSTR {};
struct RETURN_INSTR {};
struct UNREACHABLE {};
struct DROP {};

struct CALL_INSTR {
  uint32_t functionIndex;
};

struct CALL_INDIRECT {
  uint32_t typeIndex;
  uint32_t tableIndex;
};

struct RETURN_CALL_INDIRECT {
  uint32_t typeIndex;
  uint32_t tableIndex;
};

struct BR_INSTR {
  uint32_t blockIndex;
  mozilla::Maybe<uint32_t> fallthroughBlockIndex;

  bool isConditional() const { return !!fallthroughBlockIndex; }
};

struct BR_TABLE {
  std::vector<uint32_t> blockIndices;
};

struct NOP_INSTR {};
struct END_INSTR {};

struct F32_CONST {
  float value;
};

struct F32_NEAREST {};
struct F64_NEAREST {};

struct F32_LT {};
struct F32_LE {};
struct F32_GT {};
struct F32_GE {};
struct F32_EQ {};
struct F32_NEQ {};
struct F32_MIN {};
struct F32_MAX {};

struct F32_ADD {};
struct F32_SUB {};
struct F32_MUL {};
struct F32_DIV {};
struct F64_ADD {};
struct F64_SUB {};
struct F64_MUL {};
struct F64_DIV {};

struct F64_LT {};
struct F64_LE {};
struct F64_GT {};
struct F64_GE {};
struct F64_EQ {};
struct F64_NEQ {};

struct F64_MIN {};
struct F64_MAX {};

struct F64_CONST {
  double value;
};

struct I32_REINTERPRET_F32 {};
struct I64_REINTERPRET_F64 {};
struct F64_REINTERPRET_I64 {};
struct F64_CONVERT_I32_S {};
struct F64_CONVERT_I64_S {};
struct F64_COPYSIGN {};
struct F32_TRUNC {};
struct F64_TRUNC {};
struct F32_FLOOR {};
struct F64_FLOOR {};
struct F32_NEG {};
struct F64_NEG {};
struct F32_SQRT {};
struct F64_SQRT {};

struct F32_STORE {
  MemoryArgument memarg;
};

struct F32_LOAD {
  MemoryArgument memarg;
};

struct F64_STORE {
  MemoryArgument memarg;
};

struct F64_LOAD {
  MemoryArgument memarg;
};

constexpr uint32_t INVALID_BLOCK_IDX = UINT32_MAX;

using WasmInstruction = mozilla::Variant<
    I32_CONST, I32_STORE, I32_STORE_8, I32_STORE_16, I32_LOAD, I32_LOAD_8U,
    I32_LOAD_8S, I32_LOAD_16U, I32_LOAD_16S, I32_ADD, I32_SUB, I32_MUL,
    I32_DIV_U, I32_DIV_S, I32_REM_U, I32_REM_S, I32_AND, I32_OR, I32_XOR,
    I32_SHL, I32_SHR_U, I32_SHR_S, I32_EQ, I32_NEQ, I32_EQZ, I32_LT_U, I32_LT_S,
    I32_LE_U, I32_LE_S, I32_GE_U, I32_GE_S, I32_GT_U, I32_GT_S, I64_CONST,
    I64_LOAD, I64_STORE, I64_EQ, I64_EQZ, I64_NEQ, I32_WRAP_I64,
    I32_TRUNC_F64_S, I64_EXTEND_I32_S, I64_EXTEND_I32_U, I64_ADD, I64_SUB,
    I64_MUL, I64_OR, I64_AND, I64_XOR, I64_SHR_U, I64_SHR_S, I64_SHL, I64_GT_S,
    I64_GT_U, I64_GE_S, I64_GE_U, I64_LT_S, I64_LT_U, I64_LE_S, I64_LE_U,
    LOCAL_GET, LOCAL_SET, GLOBAL_GET, GLOBAL_SET, SELECT_INSTR, UNREACHABLE,
    DROP, CALL_INSTR, CALL_INDIRECT, RETURN_CALL_INDIRECT, RETURN_INSTR,
    BR_INSTR, BR_TABLE, NOP_INSTR, END_INSTR, F32_CONST, F32_NEAREST,
    F64_NEAREST, F32_LT, F32_LE, F32_GT, F32_GE, F32_EQ, F32_NEQ, F32_MIN,
    F32_MAX, F32_ADD, F32_SUB, F32_MUL, F32_DIV, F64_ADD, F64_SUB, F64_MUL,
    F64_DIV, I32_REINTERPRET_F32, I64_REINTERPRET_F64, F64_REINTERPRET_I64,
    F64_CONVERT_I32_S, F64_CONVERT_I64_S, F64_COPYSIGN, F32_TRUNC, F64_TRUNC,
    F32_FLOOR, F64_FLOOR, F32_NEG, F64_NEG, F32_SQRT, F64_SQRT, F64_LT, F64_LE,
    F64_GT, F64_GE, F64_EQ, F64_NEQ, F64_MIN, F64_MAX, F64_CONST, F32_LOAD,
    F32_STORE, F64_LOAD, F64_STORE>;

std::string InstructionToString(const WasmInstruction& instruction);

struct BasicBlock {
  std::list<WasmInstruction> instructions;
  std::vector<BasicBlock*> successors;
  uint32_t id;

  void addInstruction(WasmInstruction instruction) {
    instructions.push_back(instruction);
  }

  void addSuccessor(BasicBlock* block) { successors.push_back(block); }

  bool empty() const { return instructions.empty(); }
};

enum class WasmLocalType {
  I32,
  I64,
  F32,
  F64,
};

struct FunctionSignature {
  std::vector<WasmLocalType> parameterTypes;
  std::vector<WasmLocalType> returnTypes;
};

struct FunctionDescription {
  BasicBlock* root;
  std::map<uint32_t, std::unique_ptr<BasicBlock>> blocks;
  FunctionSignature signature;
  std::vector<WasmLocalType> locals;
};

enum class SignatureIndex : uint32_t {
  SIGNATURE_VOID_TO_VOID = 0,
  SIGNATURE_8_I32_TO_VOID = 1,
  SIGNATURE_6_I32_TO_1_I32 = 2,
  SIGNATURE_5_I32_TO_1_I32 = 3,
  SIGNATURE_1_I32_TO_VOID = 4,
  SIGNATURE_2_I32_TO_VOID = 5,
  SIGNATURE_4_I32_TO_1_I32 = 6,
  SIGNATURE_7_I32_TO_1_I32 = 7,
  SIGNATURE_2_I32_TO_1_I32 = 8,
  SIGNATURE_3_I32_TO_1_I32 = 9,
  SIGNATURE_1_i32_3_I64_TO_I64 = 10,
  SIGNATURE_1_i64_TO_VOID = 11,
  SIGNATURE_3_I32_TO_VOID = 12,
  SIGNATURE_1_F64_TO_VOID = 13,
  SIGNATURE_1_I32_TO_1_I32 = 14,
  SIGNATURE_1_I32_1_F64_TO_1_I32 = 15,
  SIGNATURE_2_F64_TO_1_F64 = 16,
  SIGNATURE_3_F64_TO_1_F64 = 17,
  SIGNATURE_4_F64_TO_1_F64 = 18,
  SIGNATURE_1_i32_4_I64_TO_I64 = 19,
  SIGNATURE_1_F64_TO_1_I32 = 20,
  SIGNATURE_COUNT,
};

}  // namespace js::jit::wasm32

#endif  // jit_wasm32_ir_ir_h
