/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_codegen_Wasm_module_builder_h
#define jit_wasm32_codegen_Wasm_module_builder_h

#include "mozilla/Maybe.h"
#include "mozilla/Variant.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "jit/wasm32/codegen/WasmEmitter.h"

namespace js::jit::wasm32 {

struct FuncType {
  const std::vector<ValueType> params;
  const std::vector<ValueType> results;
};

struct Limits {
  uint32_t min;
  mozilla::Maybe<uint32_t> max;
};

struct GlobalType {
  ValueType type;
  Mutability mutability;
};

struct FuncImportDescription {
  uint32_t typeIndex;
};

struct TableImportDescription {
  RefType type;
  Limits limits;
};

struct MemoryImportDescription {
  Limits limits;
};

struct GlobalImportDescription {
  GlobalType type;
};

using ImportDescription =
    mozilla::Variant<FuncImportDescription, TableImportDescription,
                     MemoryImportDescription, GlobalImportDescription>;

struct Import {
  const std::string module;
  const std::string name;
  ImportDescription description;
};

struct Export {
  const std::string name;
  uint32_t functionIndex;
};

struct WasmFunction {
  uint32_t typeIndex;
  std::vector<uint8_t> code;
};

class WasmModuleBuilder : public WasmEmitter {
 public:
  void AddType(FuncType type) { types_.push_back(std::move(type)); }
  uint32_t NumTypes() const { return types_.size(); }

  void AddImport(Import import) { imports_.push_back(std::move(import)); }
  void AddExport(Export exportFunc) {
    exports_.push_back(std::move(exportFunc));
  }

  void AddFunction(WasmFunction function) {
    functions_.push_back(std::move(function));
  }
  void AddGlobal(GlobalType global) { globals_.push_back(std::move(global)); }
  void AddStartFunction(uint32_t functionIndex) {
    startFunctionIndex_.emplace(functionIndex);
  }
  void DeclareFunction(uint32_t functionIndex) {
    declaredFunctions_.push_back(functionIndex);
  }

  std::vector<uint8_t> finalize();

 private:
  void emitTypeSection();
  void emitTypesVector(const std::vector<ValueType>& types);
  void emitLimits(const Limits& limits);
  void emitImportSection();
  void emitName(const std::string& str);
  void emitFunctionSection();
  void emitGlobalSection();
  void emitExportSection();
  void emitStartSection();
  void emitElementSection();
  void emitCodeSection();

  std::vector<FuncType> types_;
  std::vector<Import> imports_;
  std::vector<Export> exports_;
  std::vector<WasmFunction> functions_;
  std::vector<GlobalType> globals_;
  mozilla::Maybe<uint32_t> startFunctionIndex_;
  std::vector<uint32_t> declaredFunctions_;
};

}  // namespace js::jit::wasm32

#endif  // jit_wasm32_codegen_Wasm_module_builder_h
