/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "shell/wasmshellcommon.h"

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "jsapi.h"
#include "jsfriendapi.h"
#include "gc/Allocator.h"
#include "gc/GC.h"
#include "gc/Zone.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineJIT.h"
#include "jit/ExecutableAllocator.h"
#include "jit/JitCode.h"
#include "jit/JitOptions.h"
#include "jit/JitZone.h"
#include "jit/Linker.h"
#include "jit/wasm32/codegen/WasmEmitter.h"
#include "jit/wasm32/codegen/WasmModuleBuilder.h"
#include "jit/wasm32/ir/IR.h"
#include "js/CompilationAndEvaluation.h"
#include "js/Initialization.h"
#include "js/PropertyAndElement.h"  // JS_GetProperty, JS_DefineProperty
#include "js/RootingAPI.h"          // JS::Rooted, JS::Handle
#include "js/SourceText.h"
#include "js/Utility.h"
#include "js/Value.h"  // JS::Value, JS::StringValue
#include "js/ValueArray.h"
#include "vm/JSFunction.h"
#include "vm/JSObject.h"  // JSObject
#include "vm/JSScript.h"
#include "vm/Stack.h"
#include "jit/VMFunctionList-inl.h"
#include "vm/JSScript-inl.h"

JSContext* currentCtx = nullptr;
JSObject* currentGlobal = nullptr;

struct WasmJitModule {
  struct Function {
    std::vector<uint8_t*> patchAddresses;
    js::jit::wasm32::SignatureIndex sigIndex;
    std::vector<uint8_t> code;
  };

  void addFunction(Function code);

  uint32_t recordICStub(js::jit::JitCode* code);

  void finalize();

  std::size_t numJitFunctions() const { return jitFunctions_.size(); }

  Function generateInitFunction();

  std::vector<uint8_t> data;
  std::vector<Function> jitFunctions_;
};

namespace {

template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};

template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

uint8_t* findEndOfCode(uint8_t* begin) {
  while (*reinterpret_cast<uint32_t*>(begin) !=
         js::jit::wasm32::WasmJitCodeDelimiterPattern) {
    ++begin;
  }
  return begin;
}

void addFunction(WasmJitModule* module, uint8_t* beginCode,
                 js::jit::wasm32::SignatureIndex signatureIndex) {
  uint8_t* endCode = findEndOfCode(beginCode);

  // First 4 bytes (sizeof(uint32_t)) in the code specially allocated for
  // storing index.
  module->addFunction(WasmJitModule::Function{
      {beginCode},
      signatureIndex,
      std::vector<uint8_t>(beginCode + sizeof(uint32_t), endCode)});
}

template <class ID>
void installVMWrapper(ID id, js::jit::JitRuntime* jitRuntime,
                      WasmJitModule* module) {
  static_assert(std::is_same_v<ID, js::jit::VMFunctionId>);
  uint8_t* beginStub = jitRuntime->getVMWrapper(id).value;
  addFunction(module, beginStub,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
}

void installVMWrappers(js::jit::JitRuntime* jitRuntime, WasmJitModule* module) {
  static constexpr auto vmFunctionsCount =
      static_cast<size_t>(js::jit::VMFunctionId::Count);
  for (size_t i = 0; i < vmFunctionsCount; ++i) {
    const auto id = js::jit::VMFunctionId(i);
    installVMWrapper(id, jitRuntime, module);
  }
}

void installJitEntryStub(js::jit::JitRuntime* jitRuntime,
                         WasmJitModule* module) {
  uint8_t* beginStub = reinterpret_cast<uint8_t*>(jitRuntime->enterJit());
  addFunction(module, beginStub,
              js::jit::wasm32::SignatureIndex::SIGNATURE_8_I32_TO_VOID);
}

void installArgumentRectifierStub(js::jit::JitRuntime* jitRuntime,
                                  WasmJitModule* module) {
  uint8_t* beginStub =
      reinterpret_cast<uint8_t*>(jitRuntime->getArgumentsRectifier().value);
  addFunction(module, beginStub,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
}

void installPreBarrierStubs(js::jit::JitRuntime* jitRuntime,
                            WasmJitModule* module) {
  for (const auto type : {js::jit::MIRType::Value, js::jit::MIRType::String,
                          js::jit::MIRType::Object, js::jit::MIRType::Shape}) {
    uint8_t* beginStub =
        reinterpret_cast<uint8_t*>(jitRuntime->preBarrier(type).value);
    addFunction(module, beginStub,
                js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
  }
}

void installPostBarrierStub(js::jit::JitRuntime* jitRuntime,
                            WasmJitModule* module) {
  uint8_t* beginStub =
      reinterpret_cast<uint8_t*>(jitRuntime->postBarrier().value);
  addFunction(module, beginStub,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_4_I64_TO_I64);
}

void installDoubleToInt32ValueStub(js::jit::JitRuntime* jitRuntime,
                                   WasmJitModule* module) {
  uint8_t* beginStub =
      reinterpret_cast<uint8_t*>(jitRuntime->getDoubleToInt32ValueStub().value);
  addFunction(module, beginStub,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
}

void installICFallbackStubs(js::jit::JitRuntime* jitRuntime,
                            WasmJitModule* module) {
  for (uint32_t kindIdx = 0;
       kindIdx < static_cast<uint32_t>(js::jit::BaselineICFallbackKind::Count);
       ++kindIdx) {
    const auto kind = static_cast<js::jit::BaselineICFallbackKind>(kindIdx);
    uint8_t* beginStub = jitRuntime->baselineICFallbackCode().addr(kind).value;
    addFunction(module, beginStub,
                js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
  }
}

void installJitRuntime(js::jit::JitRuntime* jitRuntime, WasmJitModule* module) {
  installJitEntryStub(jitRuntime, module);
  installArgumentRectifierStub(jitRuntime, module);
  installPreBarrierStubs(jitRuntime, module);
  installPostBarrierStub(jitRuntime, module);
  installVMWrappers(jitRuntime, module);
  installDoubleToInt32ValueStub(jitRuntime, module);
  installICFallbackStubs(jitRuntime, module);
}

uint32_t installICStub(js::jit::ICCacheIRStub* newStub, WasmJitModule* module) {
  uint8_t* codeBegin = newStub->jitCode()->raw();
  const auto functionIndex = module->recordICStub(newStub->jitCode());
  addFunction(module, codeBegin,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
  return functionIndex;
}

void installJitFunction(js::jit::JitRuntime* jitRuntime,
                        JSScript* compilationTarget, WasmJitModule* module) {
  MOZ_RELEASE_ASSERT(compilationTarget->hasBaselineScript());

  uint8_t* codeBegin = compilationTarget->jitCodeRaw();
  addFunction(module, codeBegin,
              js::jit::wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64);
}

void installPreviouslyInlinedICStubs(
    JSScript* compilationTarget,
    std::unordered_map<uint32_t, js::jit::ICStubInfo>* functionIndexToICStub,
    WasmJitModule* module) {
  using js::jit::callSiteToItsStub;
  using js::jit::ICCacheIRStub;

  for (auto& [icCallSite, stub] : callSiteToItsStub) {
    if (icCallSite.script == compilationTarget) {
      const auto functionIndex =
          installICStub(reinterpret_cast<ICCacheIRStub*>(stub.newStub), module);
      (*functionIndexToICStub)[functionIndex] = stub;
    }
  }
}

void recordInlinedFunctions(
    std::unordered_map<uint32_t, js::jit::ICStubInfo>* functionIndexToICStub) {
  using js::jit::callSiteToItsStub;
  using js::jit::functionIndexToCallSites;

  for (auto& [inlinedFunctionIndex, callSites] : functionIndexToCallSites) {
    MOZ_RELEASE_ASSERT((*functionIndexToICStub).find(inlinedFunctionIndex) !=
                       functionIndexToICStub->end());
    MOZ_RELEASE_ASSERT(functionIndexToCallSites.find(inlinedFunctionIndex) !=
                       functionIndexToCallSites.end());
    for (auto& callSite : callSites) {
      callSiteToItsStub[callSite] =
          (*functionIndexToICStub)[inlinedFunctionIndex];
    }
  }

  functionIndexToCallSites.clear();
}

}  // namespace

void* AllocateBytes(size_t len) { return std::malloc(len); }

void FreeBytes(void* ptr) { std::free(ptr); }

void SetArgv(char** argv, size_t index, char* value) { argv[index] = value; }

uint8_t* moduleData(WasmJitModule* mod) { return mod->data.data(); }

size_t moduleSize(WasmJitModule* mod) { return mod->data.size(); }

void freeModule(WasmJitModule* mod) { delete mod; }

WasmJitModule* jitRuntimeModule() {
  MOZ_RELEASE_ASSERT(currentCtx->runtime()->hasJitRuntime());

  auto* jitModule = new WasmJitModule();
  auto* jitRuntime = currentCtx->runtime()->jitRuntime();
  installJitRuntime(jitRuntime, jitModule);

  MOZ_RELEASE_ASSERT(jitModule->numJitFunctions() > 0);

  jitModule->finalize();
  return jitModule;
}

WasmJitModule* jitModule() {
  using js::jit::AttachNew;
  using js::jit::DiscardStubs;
  using js::jit::ICCacheIRStub;
  using js::jit::ICCallSite;
  using js::jit::ICEntry;
  using js::jit::ICFallbackStub;
  using js::jit::ICStubInfo;
  using js::jit::icStubToItsCode;
  using js::jit::jitCandidates;
  using js::jit::pendingICStubs;

  MOZ_RELEASE_ASSERT(currentCtx->runtime()->hasJitRuntime());

  auto* jitModule = new WasmJitModule();
  {
    JSAutoRealm ar(currentCtx, currentGlobal);

    auto* jitRuntime = currentCtx->runtime()->jitRuntime();

    std::unordered_map<uint32_t, ICStubInfo> functionIndexToICStub;
    for (auto& [icEntryUntyped, action] : pendingICStubs) {
      auto* icEntry = reinterpret_cast<ICEntry*>(icEntryUntyped);
      std::visit(
          overloaded{
              [jitModule, icEntry,
               &functionIndexToICStub](const AttachNew& attachNew) {
                auto* newStub = reinterpret_cast<ICCacheIRStub*>(
                    attachNew.stubInfo.newStub);
                const auto functionIndex = installICStub(newStub, jitModule);

                // Add ICStub into IC chain.
                auto* fallbackStub = reinterpret_cast<ICFallbackStub*>(
                    attachNew.stubInfo.fallbackStub);
                fallbackStub->addNewStub(icEntry, newStub);
                functionIndexToICStub[functionIndex] = attachNew.stubInfo;
              },
              [icEntry](const DiscardStubs& discard) {
                auto* fallbackStub =
                    reinterpret_cast<ICFallbackStub*>(discard.fallbackStub);
                icStubToItsCode.erase(
                    ICCallSite{fallbackStub->pcOffset(), discard.script});
                fallbackStub->discardStubs(currentCtx->zone(), icEntry);
              }},
          action);
    }
    pendingICStubs.clear();

    for (JSScript* compilationTarget : jitCandidates) {
      installPreviouslyInlinedICStubs(compilationTarget, &functionIndexToICStub,
                                      jitModule);

      const auto status =
          js::jit::BaselineCompile(currentCtx, compilationTarget,
                                   /* forceDebugInstrumentation */ false);
      if (status != js::jit::Method_Compiled) {
        continue;
      }

      installJitFunction(jitRuntime, compilationTarget, jitModule);
    }

    recordInlinedFunctions(&functionIndexToICStub);
  }
  jitCandidates.clear();

  if (jitModule->numJitFunctions() == 0) {
    return nullptr;
  }

  jitModule->finalize();
  return jitModule;
}

void WasmJitModule::addFunction(Function code) {
  jitFunctions_.push_back(std::move(code));
}

uint32_t WasmJitModule::recordICStub(js::jit::JitCode* code) {
  const auto functionIndex = numJitFunctions();
  js::jit::icCodeToFunctionIndex[code] = functionIndex;
  return functionIndex;
}

void WasmJitModule::finalize() {
  using namespace js::jit;
  using js::jit::wasm32::FuncType;
  using js::jit::wasm32::ValueType;

  // Since all jitted function have been added we can generate start function.
  addFunction(generateInitFunction());

  wasm32::WasmModuleBuilder builder;

  {
    // Type section
    // Be careful with the ordering, signatures from this section may be used in
    // Masm.

    auto initFunctionType = FuncType{{}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_VOID_TO_VOID));
    builder.AddType(std::move(initFunctionType));

    auto enterJitCodeType = FuncType{
        {ValueType::I32, ValueType::I32, ValueType::I32, ValueType::I32,
         ValueType::I32, ValueType::I32, ValueType::I32, ValueType::I32},
        {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_8_I32_TO_VOID));
    builder.AddType(std::move(enterJitCodeType));

    auto runtimeFuncSignature1 =
        FuncType{{ValueType::I32, ValueType::I32, ValueType::I32,
                  ValueType::I32, ValueType::I32, ValueType::I32},
                 {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_6_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature1));

    auto runtimeFuncSignature2 =
        FuncType{{ValueType::I32, ValueType::I32, ValueType::I32,
                  ValueType::I32, ValueType::I32},
                 {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_5_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature2));

    auto debugPrintType1 = FuncType{{ValueType::I32}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_I32_TO_VOID));
    builder.AddType(std::move(debugPrintType1));

    auto debugPrintType2 = FuncType{{ValueType::I32, ValueType::I32}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_2_I32_TO_VOID));
    builder.AddType(std::move(debugPrintType2));

    auto runtimeFuncSignature3 = FuncType{
        {ValueType::I32, ValueType::I32, ValueType::I32, ValueType::I32},
        {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_4_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature3));

    auto runtimeFuncSignature4 = FuncType{
        {ValueType::I32, ValueType::I32, ValueType::I32, ValueType::I32,
         ValueType::I32, ValueType::I32, ValueType::I32},
        {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_7_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature4));

    auto runtimeFuncSignature5 =
        FuncType{{ValueType::I32, ValueType::I32}, {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_2_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature5));

    auto runtimeFuncSignature6 = FuncType{
        {ValueType::I32, ValueType::I32, ValueType::I32}, {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_3_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature6));

    auto runtimeFuncSignature7 = FuncType{
        {ValueType::I32, ValueType::I64, ValueType::I64, ValueType::I64},
        {ValueType::I64}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(
            wasm32::SignatureIndex::SIGNATURE_1_i32_3_I64_TO_I64));
    builder.AddType(std::move(runtimeFuncSignature7));

    auto runtimeFuncSignature8 = FuncType{{ValueType::I64}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_i64_TO_VOID));
    builder.AddType(std::move(runtimeFuncSignature8));

    auto runtimeFuncSignature9 =
        FuncType{{ValueType::I32, ValueType::I32, ValueType::I32}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_3_I32_TO_VOID));
    builder.AddType(std::move(runtimeFuncSignature9));

    auto runtimeFuncSignature10 = FuncType{{ValueType::F64}, {}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(wasm32::SignatureIndex::SIGNATURE_1_F64_TO_VOID));
    builder.AddType(std::move(runtimeFuncSignature10));

    auto runtimeFuncSignature11 = FuncType{{ValueType::I32}, {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_1_I32_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature11));

    auto runtimeFuncSignature12 =
        FuncType{{ValueType::I32, ValueType::F64}, {ValueType::I32}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(
            wasm32::SignatureIndex::SIGNATURE_1_I32_1_F64_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature12));

    auto runtimeFuncSignature13 =
        FuncType{{ValueType::F64, ValueType::F64}, {ValueType::F64}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_2_F64_TO_1_F64));
    builder.AddType(std::move(runtimeFuncSignature13));

    auto runtimeFuncSignature14 = FuncType{
        {ValueType::F64, ValueType::F64, ValueType::F64}, {ValueType::F64}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_3_F64_TO_1_F64));
    builder.AddType(std::move(runtimeFuncSignature14));

    auto runtimeFuncSignature15 = FuncType{
        {ValueType::F64, ValueType::F64, ValueType::F64, ValueType::F64},
        {ValueType::F64}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_4_F64_TO_1_F64));
    builder.AddType(std::move(runtimeFuncSignature15));

    auto runtimeFuncSignature16 =
        FuncType{{ValueType::I32, ValueType::I64, ValueType::I64,
                  ValueType::I64, ValueType::I64},
                 {ValueType::I64}};
    MOZ_RELEASE_ASSERT(
        builder.NumTypes() ==
        static_cast<uint32_t>(
            wasm32::SignatureIndex::SIGNATURE_1_i32_4_I64_TO_I64));
    builder.AddType(std::move(runtimeFuncSignature16));

    auto runtimeFuncSignature17 = FuncType{{ValueType::F64}, {ValueType::I32}};
    MOZ_RELEASE_ASSERT(builder.NumTypes() ==
                       static_cast<uint32_t>(
                           wasm32::SignatureIndex::SIGNATURE_1_F64_TO_1_I32));
    builder.AddType(std::move(runtimeFuncSignature17));
  }

  {
    // Import section
    //  (import "env" "memory" (memory $memory 1))
    //  (import "env" "__indirect_function_table" (table $table 0 funcref))
    //  (import "env" "__stack_pointer" (global $sp (mut i32)))

    auto memoryImportDescr =
        wasm32::MemoryImportDescription{wasm32::Limits{1u, mozilla::Nothing()}};
    builder.AddImport(wasm32::Import{
        "env", "memory",
        wasm32::ImportDescription{std::move(memoryImportDescr)}});

    auto tableImportDescr = wasm32::TableImportDescription{
        wasm32::RefType::Funcref, wasm32::Limits{0u, mozilla::Nothing()}};
    builder.AddImport(
        wasm32::Import{"env", "__indirect_function_table",
                       wasm32::ImportDescription{std::move(tableImportDescr)}});

    auto importDescr =
        wasm32::ImportDescription{wasm32::GlobalImportDescription{
            wasm32::GlobalType{ValueType::I32, wasm32::Mutability::Var}}};
    builder.AddImport(wasm32::Import{"env", "__stack_pointer", importDescr});
  }

  {
    // Functions section
    for (uint32_t i = 0; i < jitFunctions_.size(); ++i) {
      auto& jitFunc = jitFunctions_[i];
      builder.AddFunction(wasm32::WasmFunction{
          static_cast<uint32_t>(jitFunc.sigIndex), std::move(jitFunc.code)});
    }
  }

  {
    // Export section
    // We export all our function except init function to prevent wasm-opt from
    // removing them.
    std::string exportName = "e";
    for (uint32_t i = 0; i < jitFunctions_.size() - 1; ++i) {
      builder.AddExport(wasm32::Export{exportName + std::to_string(i), i});
    }
  }

  {
    // Start section
    builder.AddStartFunction(jitFunctions_.size() - 1);
  }

  {
    // Element section
    for (uint32_t i = 0; i < jitFunctions_.size() - 1; ++i) {
      builder.DeclareFunction(i);
    }
  }

  data = builder.finalize();
  js::jit::icCodeToFunctionIndex.clear();
}

WasmJitModule::Function WasmJitModule::generateInitFunction() {
  js::jit::wasm32::WasmEmitter emitter;

  // declare local[0] type=i32
  emitter.emit({0x01, 0x01, 0x7f});

  // local[0] = table.size()
  emitter.emitTableSize();
  emitter.emitLocalSet(0);

  // table.grow(jit_functions_.size())
  emitter.emitRefNullFunc();
  emitter.emitI32Const(jitFunctions_.size());
  emitter.emitTableGrow();

  {
    // Check table grow result.
    emitter.emitI32Const(-1);
    emitter.emitI32Eq();

    emitter.emitIf();
    emitter.emitEmptyBlockType();
    emitter.emitUnreachable();
    emitter.emitEnd();
  }

  for (uint32_t i = 0u; i < jitFunctions_.size(); ++i) {
    emitter.emitLocalGet(0);
    emitter.emitRefFunc(i);
    emitter.emitTableSet();

    for (const auto* addr : jitFunctions_[i].patchAddresses) {
      emitter.emitI32Const(reinterpret_cast<uint32_t>(addr));
      emitter.emitLocalGet(0);  // function index
      emitter.emitI32Store(0, js::jit::Int32SizeLog2);
    }

    // Increment local[0]
    emitter.emitLocalGet(0);
    emitter.emitI32Const(1);
    emitter.emitI32Add();
    emitter.emitLocalSet(0);
  }

  emitter.emitEnd();
  return Function{{},
                  js::jit::wasm32::SignatureIndex::SIGNATURE_VOID_TO_VOID,
                  emitter.finalize()};
}
