/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/codegen/WasmModuleBuilder.h"

namespace js::jit::wasm32 {

std::vector<uint8_t> WasmModuleBuilder::finalize() {
  emitMagic();
  emitVersion();

  emitTypeSection();
  emitImportSection();
  emitFunctionSection();
  emitGlobalSection();
  emitExportSection();
  emitStartSection();
  emitElementSection();
  emitCodeSection();

  return WasmEmitter::finalize();
}

void WasmModuleBuilder::emitTypeSection() {
  emit(SectionId::Type);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(types_.size());
  for (const auto& type : types_) {
    emit(0x60);  // Type constructor for function types.
    emitTypesVector(type.params);
    emitTypesVector(type.results);
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitTypesVector(const std::vector<ValueType>& types) {
  emitVarU32(types.size());
  for (const auto type : types) {
    emit(type);
  }
}

void WasmModuleBuilder::emitLimits(const Limits& limits) {
  if (!limits.max) {
    emit(0x00);
    emitVarU32(limits.min);
    return;
  }

  emit(0x01);
  emitVarU32(limits.min);
  emitVarU32(*limits.max);
}

void WasmModuleBuilder::emitImportSection() {
  emit(SectionId::Import);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(imports_.size());
  for (const auto& import : imports_) {
    emitName(import.module);
    emitName(import.name);

    import.description.match(
        [this](const FuncImportDescription& funcDescr) {
          emit(ImportKind::Func);
          emitVarU32(funcDescr.typeIndex);
        },
        [this](const TableImportDescription& tableDescr) {
          emit(ImportKind::Table);
          emit(tableDescr.type);
          emitLimits(tableDescr.limits);
        },
        [this](const MemoryImportDescription& memDescr) {
          emit(ImportKind::Memory);
          emitLimits(memDescr.limits);
        },
        [this](const GlobalImportDescription& globalDescr) {
          emit(ImportKind::Global);
          emit(globalDescr.type.type);
          emit(globalDescr.type.mutability);
        });
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitExportSection() {
  emit(SectionId::Export);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(exports_.size());
  for (const auto& eport : exports_) {
    emitName(eport.name);
    emitVarI32(0x00);
    emitVarI32(eport.functionIndex);
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitName(const std::string& str) {
  emitVarU32(str.size());
  for (auto ch : str) {
    emit(ch);
  }
}

void WasmModuleBuilder::emitFunctionSection() {
  emit(SectionId::Function);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(functions_.size());
  for (const auto& func : functions_) {
    emitVarU32(func.typeIndex);
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitGlobalSection() {
  if (globals_.empty()) {
    return;
  }

  emit(SectionId::Global);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(globals_.size());
  for (const auto& global : globals_) {
    emit(global.type);
    emit(global.mutability);

    MOZ_RELEASE_ASSERT(global.type == ValueType::I32);
    emit({0x41, 0x00, 0x0b});  // i32.const 0
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitStartSection() {
  if (!startFunctionIndex_) {
    return;
  }

  emit(SectionId::Start);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(*startFunctionIndex_);

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitElementSection() {
  emit(SectionId::Elem);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emit({0x01, 0x03, 0x00});  // declarative
  emitVarU32(declaredFunctions_.size());
  for (const auto funcIdx : declaredFunctions_) {
    emitVarU32(funcIdx);
  }

  patchVarU32(patchLoc, currentOffset() - start);
}

void WasmModuleBuilder::emitCodeSection() {
  emit(SectionId::Code);
  std::size_t patchLoc = emitPatchableVarU32();
  std::size_t start = currentOffset();

  emitVarU32(functions_.size());
  for (const auto& func : functions_) {
    emitVarU32(func.code.size());
    emit(std::move(func.code));
  }

  patchVarU32(patchLoc, currentOffset() - start);
  functions_.clear();
}

}  // namespace js::jit::wasm32
