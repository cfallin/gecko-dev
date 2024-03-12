/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_codegen_codegen_h
#define jit_wasm32_codegen_codegen_h

#include <map>
#include <memory>
#include <vector>

#include "jit/wasm32/codegen/WasmEmitter.h"
#include "jit/wasm32/ir/IR.h"

namespace js::jit::wasm32 {

namespace {
struct LayoutTreeNode;
}  // namespace

class Wasm32CodeGenerator {
 public:
  std::vector<uint8_t> generateCode(FunctionDescription function);

 private:
  void emitLocalsInfo(std::vector<WasmLocalType> localTypes);

  void emitLayoutTree(LayoutTreeNode* root, bool emitFirstAsLoop);

  void encodeInstruction(const WasmInstruction& instruction);

  WasmEmitter emitter;
};

}  // namespace js::jit::wasm32

#endif  // jit_wasm32_codegen_codegen_h
