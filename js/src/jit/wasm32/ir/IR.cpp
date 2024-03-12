/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/ir/IR.h"

namespace js::jit::wasm32 {

std::string InstructionToString(const WasmInstruction& instruction) {
  std::string output;
  instruction.match(
      [&output](const I32_CONST& instr) { output = "I32_CONST"; },
      [&output](const I32_STORE& instr) { output = "I32_STORE"; },
      [&output](const I32_STORE_8& instr) { output = "I32_STORE_8"; },
      [&output](const I32_STORE_16& instr) { output = "I32_STORE_16"; },
      [&output](const I32_LOAD& instr) { output = "I32_LOAD"; },
      [&output](const I32_LOAD_8U& instr) { output = "I32_LOAD_8U"; },
      [&output](const I32_LOAD_8S& instr) { output = "I32_LOAD_8S"; },
      [&output](const I32_LOAD_16U& instr) { output = "I32_LOAD_16U"; },
      [&output](const I32_LOAD_16S& instr) { output = "I32_LOAD_16S"; },
      [&output](const I32_ADD& instr) { output = "I32_ADD"; },
      [&output](const I32_SUB& instr) { output = "I32_SUB"; },
      [&output](const I32_MUL& instr) { output = "I32_MUL"; },
      [&output](const I32_DIV_U& instr) { output = "I32_DIV_U"; },
      [&output](const I32_DIV_S& instr) { output = "I32_DIV_S"; },
      [&output](const I32_REM_U& instr) { output = "I32_REM_U"; },
      [&output](const I32_REM_S& instr) { output = "I32_REM_S"; },
      [&output](const I32_AND& instr) { output = "I32_AND"; },
      [&output](const I32_OR& instr) { output = "I32_OR"; },
      [&output](const I32_XOR& instr) { output = "I32_XOR"; },
      [&output](const I32_SHL& instr) { output = "I32_SHL"; },
      [&output](const I32_SHR_U& instr) { output = "I32_SHR_U"; },
      [&output](const I32_SHR_S& instr) { output = "I32_SHR_S"; },
      [&output](const I32_EQ& instr) { output = "I32_EQ"; },
      [&output](const I32_NEQ& instr) { output = "I32_NEQ"; },
      [&output](const I32_EQZ& instr) { output = "I32_EQZ"; },
      [&output](const I32_LT_U& instr) { output = "I32_LT_U"; },
      [&output](const I32_LT_S& instr) { output = "I32_LT_S"; },
      [&output](const I32_LE_U& instr) { output = "I32_LE_U"; },
      [&output](const I32_LE_S& instr) { output = "I32_LE_S"; },
      [&output](const I32_GE_U& instr) { output = "I32_GE_U"; },
      [&output](const I32_GE_S& instr) { output = "I32_GE_S"; },
      [&output](const I32_GT_U& instr) { output = "I32_GT_U"; },
      [&output](const I32_GT_S& instr) { output = "I32_GT_S"; },
      [&output](const I64_CONST& instr) { output = "I64_CONST"; },
      [&output](const I64_LOAD& instr) { output = "I64_LOAD"; },
      [&output](const I64_STORE& instr) { output = "I64_STORE"; },
      [&output](const I64_EQ& instr) { output = "I64_EQ"; },
      [&output](const I64_EQZ& instr) { output = "I64_EQZ"; },
      [&output](const I64_NEQ& instr) { output = "I64_NEQ"; },
      [&output](const I32_WRAP_I64& instr) { output = "I32_WRAP_I64"; },
      [&output](const I32_TRUNC_F64_S& instr) { output = "I32_TRUNC_F64_S"; },
      [&output](const I64_EXTEND_I32_S& instr) { output = "I64_EXTEND_I32_S"; },
      [&output](const I64_EXTEND_I32_U& instr) { output = "I64_EXTEND_I32_U"; },
      [&output](const I64_ADD& instr) { output = "I64_ADD"; },
      [&output](const I64_SUB& instr) { output = "I64_SUB"; },
      [&output](const I64_MUL& instr) { output = "I64_MUL"; },
      [&output](const I64_OR& instr) { output = "I64_OR"; },
      [&output](const I64_AND& instr) { output = "I64_AND"; },
      [&output](const I64_XOR& instr) { output = "I64_XOR"; },
      [&output](const I64_SHR_U& instr) { output = "I64_SHR_U"; },
      [&output](const I64_SHR_S& instr) { output = "I64_SHR_S"; },
      [&output](const I64_SHL& instr) { output = "I64_SHL"; },
      [&output](const I64_GT_S& instr) { output = "I64_GT_S"; },
      [&output](const I64_GT_U& instr) { output = "I64_GT_U"; },
      [&output](const I64_GE_S& instr) { output = "I64_GE_S"; },
      [&output](const I64_GE_U& instr) { output = "I64_GE_U"; },
      [&output](const I64_LT_S& instr) { output = "I64_LT_S"; },
      [&output](const I64_LT_U& instr) { output = "I64_LT_U"; },
      [&output](const I64_LE_S& instr) { output = "I64_LE_S"; },
      [&output](const I64_LE_U& instr) { output = "I64_LE_U"; },
      [&output](const LOCAL_GET& instr) { output = "LOCAL_GET"; },
      [&output](const LOCAL_SET& instr) { output = "LOCAL_SET"; },
      [&output](const GLOBAL_GET& instr) { output = "GLOBAL_GET"; },
      [&output](const GLOBAL_SET& instr) { output = "GLOBAL_SET"; },
      [&output](const SELECT_INSTR& instr) { output = "SELECT_INSTR"; },
      [&output](const UNREACHABLE& instr) { output = "UNREACHABLE"; },
      [&output](const DROP&) { output = "DROP"; },
      [&output](const CALL_INSTR& instr) { output = "CALL"; },
      [&output](const CALL_INDIRECT& instr) { output = "CALL_INDIRECT"; },
      [&output](const RETURN_CALL_INDIRECT& instr) {
        output = "RETURN_CALL_INDIRECT";
      },
      [&output](const RETURN_INSTR& instr) { output = "RETURN_INSTR"; },
      [&output](const BR_INSTR& instr) {
        output = instr.isConditional() ? "BR_IF" : "BR_INSTR";
      },
      [&output](const BR_TABLE& instr) { output = "BR_TABLE"; },
      [&output](const NOP_INSTR& instr) { output = "NOP"; },
      [&output](const END_INSTR& instr) { output = "END_INSTR"; },
      [&output](const F32_CONST& instr) { output = "F32_CONST"; },
      [&output](const F32_NEAREST& instr) { output = "F32_NEAREST"; },
      [&output](const F64_NEAREST& instr) { output = "F64_NEAREST"; },
      [&output](const F32_LT& instr) { output = "F32_LT"; },
      [&output](const F32_LE& instr) { output = "F32_LE"; },
      [&output](const F32_GT& instr) { output = "F32_GT"; },
      [&output](const F32_GE& instr) { output = "F32_GE"; },
      [&output](const F32_EQ& instr) { output = "F32_EQ"; },
      [&output](const F32_NEQ& instr) { output = "F32_NEQ"; },
      [&output](const F32_MIN& instr) { output = "F32_MIN"; },
      [&output](const F32_MAX& instr) { output = "F32_MAX"; },
      [&output](const F32_ADD& instr) { output = "F32_ADD"; },
      [&output](const F32_SUB& instr) { output = "F32_SUB"; },
      [&output](const F32_MUL& instr) { output = "F32_MUL"; },
      [&output](const F32_DIV& instr) { output = "F32_DIV"; },
      [&output](const F64_ADD& instr) { output = "F64_ADD"; },
      [&output](const F64_SUB& instr) { output = "F64_SUB"; },
      [&output](const F64_MUL& instr) { output = "F64_MUL"; },
      [&output](const F64_DIV& instr) { output = "F64_DIV"; },
      [&output](const I32_REINTERPRET_F32& instr) {
        output = "I32_REINTERPRET_F32";
      },
      [&output](const I64_REINTERPRET_F64& instr) {
        output = "I64_REINTERPRET_F64";
      },
      [&output](const F64_REINTERPRET_I64& instr) {
        output = "F64_REINTERPRET_I64";
      },
      [&output](const F64_CONVERT_I32_S& instr) {
        output = "F64_CONVERT_I32_S";
      },
      [&output](const F64_CONVERT_I64_S& instr) {
        output = "F64_CONVERT_I64_S";
      },
      [&output](const F64_COPYSIGN& instr) { output = "F64_COPYSIGN"; },
      [&output](const F32_TRUNC& instr) { output = "F32_TRUNC"; },
      [&output](const F64_TRUNC& instr) { output = "F64_TRUNC"; },
      [&output](const F32_FLOOR& instr) { output = "F32_FLOOR"; },
      [&output](const F64_FLOOR& instr) { output = "F64_FLOOR"; },
      [&output](const F32_NEG& instr) { output = "F32_NEG"; },
      [&output](const F64_NEG& instr) { output = "F64_NEG"; },
      [&output](const F32_SQRT& instr) { output = "F32_SQRT"; },
      [&output](const F64_SQRT& instr) { output = "F64_SQRT"; },
      [&output](const F64_LT& instr) { output = "F64_LT"; },
      [&output](const F64_LE& instr) { output = "F64_LE"; },
      [&output](const F64_GT& instr) { output = "F64_GT"; },
      [&output](const F64_GE& instr) { output = "F64_GE"; },
      [&output](const F64_EQ& instr) { output = "F64_EQ"; },
      [&output](const F64_NEQ& instr) { output = "F64_NEQ"; },
      [&output](const F64_MIN& instr) { output = "F64_MIN"; },
      [&output](const F64_MAX& instr) { output = "F64_MAX"; },
      [&output](const F64_CONST& instr) { output = "F64_CONST"; },
      [&output](const F32_LOAD& instr) { output = "F32_LOAD"; },
      [&output](const F32_STORE& instr) { output = "F32_STORE"; },
      [&output](const F64_LOAD& instr) { output = "F64_LOAD"; },
      [&output](const F64_STORE& instr) { output = "F64_STORE"; });
  return output;
}

}  // namespace js::jit::wasm32
