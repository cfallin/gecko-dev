/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/Assembler-wasm32.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <set>
#include <string>

#ifdef DEBUG
#  include <iostream>
#endif

#include "jit/wasm32/codegen/Codegen.h"

namespace js::jit {

namespace {

#ifdef DEBUG
std::string blockToName(wasm32::BasicBlock* block) {
  return std::string("b") + std::to_string(block->id);
}

void dumpGraphHelper(wasm32::BasicBlock* root,
                     std::set<wasm32::BasicBlock*>* visited) {
  if (!root) {
    return;
  }

  visited->insert(root);
  for (const auto& successor : root->successors) {
    std::cout << blockToName(root) << " -> " << blockToName(successor) << ";"
              << std::endl;
  }

  for (const auto& successor : root->successors) {
    if (visited->find(successor) != visited->end()) {
      continue;
    }
    dumpGraphHelper(successor, visited);
  }
}

void dumpGraph(wasm32::BasicBlock* root) {
  std::set<wasm32::BasicBlock*> visited;

  std::cout << "digraph graphname {" << std::endl;
  dumpGraphHelper(root, &visited);
  std::cout << "}" << std::endl;
}
#endif

}  // namespace

Assembler::Condition AssemblerWasm32::InvertCondition(Condition cond) {
  switch (cond) {
    case Equal:
      return NotEqual;
    case NotEqual:
      return Equal;
    case Zero:
      return NonZero;
    case NonZero:
      return Zero;
    default:
      MOZ_CRASH("unexpected condition");
  }
}

void AssemblerWasm32::register_get(Register reg) {
  if (Registers::IsLocal(reg.code())) {
    local_get(reg);
  } else {
    global_get(reg);
  }
}

void AssemblerWasm32::register_set(Register reg) {
  if (Registers::IsLocal(reg.code())) {
    local_set(reg);
  } else {
    global_set(reg);
  }
}

void AssemblerWasm32::register_get(FloatRegister reg) {
  if (FloatRegisters::IsLocal(reg.code())) {
    local_get(reg);
  } else {
    global_get(reg);
  }
}

void AssemblerWasm32::register_set(FloatRegister reg) {
  if (FloatRegisters::IsLocal(reg.code())) {
    local_set(reg);
  } else {
    global_set(reg);
  }
}

void AssemblerWasm32::register_get32(Register reg) {
  register_get(reg);
  if (is64BitReg(reg)) {
    i32_wrap_i64();
  }
}

void AssemblerWasm32::register_set32(Register reg) {
  if (is64BitReg(reg)) {
    i64_extend_i32_u();
  }
  register_set(reg);
}

void AssemblerWasm32::register_get64(Register reg) {
  MOZ_RELEASE_ASSERT(is64BitReg(reg));
  register_get(reg);
}

void AssemblerWasm32::register_set64(Register reg) {
  MOZ_RELEASE_ASSERT(is64BitReg(reg));
  register_set(reg);
}

void AssemblerWasm32::register_getf32(FloatRegister reg) {
  MOZ_RELEASE_ASSERT(reg.isSingle());
  register_get(reg);
}

void AssemblerWasm32::register_setf32(FloatRegister reg) {
  MOZ_RELEASE_ASSERT(reg.isSingle());
  register_set(reg);
}

void AssemblerWasm32::register_getf64(FloatRegister reg) {
  MOZ_RELEASE_ASSERT(reg.isDouble());
  register_get(reg);
}

void AssemblerWasm32::register_setf64(FloatRegister reg) {
  MOZ_RELEASE_ASSERT(reg.isDouble());
  register_set(reg);
}

void AssemblerWasm32::emitConditionf64(Assembler::DoubleCondition cond) {
  switch (cond) {
    case Assembler::DoubleEqual:
      return f64_eq();
    case Assembler::DoubleNotEqual:
    case Assembler::DoubleNotEqualOrUnordered:
      return f64_neq();
    case Assembler::DoubleGreaterThan:
      return f64_gt();
    case Assembler::DoubleGreaterThanOrEqual:
      return f64_ge();
    case Assembler::DoubleLessThan:
      return f64_lt();
    case Assembler::DoubleLessThanOrEqual:
      return f64_le();
    default:
      MOZ_CRASH();
      break;
  }
}

void AssemblerWasm32::emitCondition32(Assembler::Condition cond) {
  switch (cond) {
    case Assembler::Equal:
      return i32_eq();
    case Assembler::NotEqual:
      return i32_neq();
    case Assembler::Above:
      return i32_gt_u();
    case Assembler::AboveOrEqual:
      return i32_ge_u();
    case Assembler::Below:
      return i32_lt_u();
    case Assembler::BelowOrEqual:
      return i32_le_s();
    case Assembler::GreaterThan:
      return i32_gt_s();
    case Assembler::GreaterThanOrEqual:
      return i32_ge_s();
    case Assembler::LessThan:
      return i32_lt_s();
    case Assembler::LessThanOrEqual:
      return i32_le_s();
    case Assembler::Zero:
      return i32_eqz();
    case Assembler::NonZero: {
      i32_const(0);
      i32_neq();
      return;
    }
    default:
      MOZ_CRASH();
      break;
  }
}

void AssemblerWasm32::emitCondition64(Assembler::Condition cond) {
  switch (cond) {
    case Assembler::Equal:
      return i64_eq();
    case Assembler::NotEqual:
      return i64_neq();
    case Assembler::Above:
      return i64_gt_u();
    case Assembler::Below:
      return i64_lt_u();
    case Assembler::BelowOrEqual:
      return i64_le_s();
    case Assembler::GreaterThan:
      return i64_gt_s();
    case Assembler::LessThan:
      return i64_lt_s();
    case Assembler::LessThanOrEqual:
      return i64_le_s();
    case Assembler::GreaterThanOrEqual:
      return i64_ge_s();
    case Assembler::Zero:
      return i64_eqz();
    case Assembler::NonZero: {
      i64_const(0);
      i64_neq();
      return;
    }
    default:
      MOZ_CRASH();
      break;
  }
}

void AssemblerWasm32::executableCopy(void* buffer) {
  std::copy(codeBuffer_.cbegin(), codeBuffer_.cend(),
            static_cast<uint8_t*>(buffer));
  codeBuffer_.clear();
}

void AssemblerWasm32::finish() {
  introduceLocalsForRegisters();

  auto methodCode =
      wasm32::Wasm32CodeGenerator().generateCode(wasm32::FunctionDescription{
          root_, std::move(blocks_), std::move(funcSignature_),
          std::move(locals_)});
  codeBuffer_.insert(codeBuffer_.end(),
                     std::make_move_iterator(methodCode.begin()),
                     std::make_move_iterator(methodCode.end()));

  resetCurrentCFG();
}

void AssemblerWasm32::introduceSignature(wasm32::FunctionSignature signature) {
  funcSignature_ = std::move(signature);
}

uint32_t AssemblerWasm32::introduceLocal(wasm32::WasmLocalType type) {
  const auto index = locals_.size() + numberOfParameters();
  locals_.push_back(type);
  return index;
}

void AssemblerWasm32::introduceLocalsForRegisters() {
  // For the registers: wbx, wdx, wdi, w0 - w7.
  for (std::size_t i = 0; i < (3 + 8); ++i) {
    introduceLocal(wasm32::WasmLocalType::I64);
  }

  // For the registers: f0 - f7.
  for (std::size_t i = 0; i < 8; ++i) {
    introduceLocal(wasm32::WasmLocalType::F32);
  }

  // For the registers: d0 - d7.
  for (std::size_t i = 0; i < 8; ++i) {
    introduceLocal(wasm32::WasmLocalType::F64);
  }
}

void AssemblerWasm32::resetCurrentCFG() {
  blocks_.clear();
  nextBlockId_ = 0;
  root_ = createBlock();
  currentBlock_ = root_;
  locals_.clear();
  funcSignature_.returnTypes.clear();
  funcSignature_.parameterTypes.clear();
}

void AssemblerWasm32::ensureNewBlockWithLink() {
  if (currentBlock_->empty()) {
    return;
  }

  auto* newBlock = createBlock();
  currentBlock_->addSuccessor(newBlock);
  currentBlock_ = newBlock;
}

void AssemblerWasm32::ensureNewBlock() {
  if (currentBlock_->empty()) {
    return;
  }

  currentBlock_ = createBlock();
}

wasm32::BasicBlock* AssemblerWasm32::createBlock() {
  auto newBlock = std::make_unique<wasm32::BasicBlock>();
  if (!newBlock) {
    MOZ_CRASH();
  }
  newBlock->id = nextBlockId_++;
  auto* newBlockPtr = newBlock.get();
  blocks_.emplace(newBlock->id, std::move(newBlock));
  return newBlockPtr;
}

void Assembler::writeDataRelocation(ImmGCPtr ptr) {
  // Raw GC pointer relocations and Value relocations both end up in
  // Assembler::TraceDataRelocations.
  if (ptr.value) {
    if (gc::IsInsideNursery(ptr.value)) {
      embedsNurseryPointers_ = true;
    }
    dataRelocations_.writeUnsigned(0);
  }
}

ABIArg ABIArgGenerator::next(MIRType type) {
  current_ = ABIArg(nextArgIndex_);
  ++nextArgIndex_;
  return current_;
}

ABIArg& ABIArgGenerator::current() { return current_; }

uint32_t ABIArgGenerator::stackBytesConsumedSoFar() const {
  // For wasm32 calls we don't use shadow stack.
  return 0;
}

void ABIArgGenerator::increaseStackOffset(uint32_t) {
  // For wasm32 it doesn't make any sense.
  MOZ_CRASH();
}

ABIArgGenerator::ABIArgGenerator() : current_(), nextArgIndex_(0u) {}

}  // namespace js::jit
