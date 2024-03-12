/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef shell_wasmshellcommon_js_h
#define shell_wasmshellcommon_js_h

#include "jit/JitRuntime.h"

extern JSContext* currentCtx;
extern JSObject* currentGlobal;
struct WasmJitModule;

#define EXPORT(name) __attribute__((export_name(name)))

EXPORT("AllocateBytes")
void* AllocateBytes(size_t len);

EXPORT("FreeBytes")
void FreeBytes(void* ptr);

EXPORT("AllocateArgv")
void* AllocateArgv(size_t len);

EXPORT("SetArgv")
void SetArgv(char** argv, size_t index, char* value);

EXPORT("moduleData")
uint8_t* moduleData(WasmJitModule* mod);
EXPORT("moduleSize")
size_t moduleSize(WasmJitModule* mod);
EXPORT("freeModule")
void freeModule(WasmJitModule* mod);

EXPORT("jitRuntimeModule")
WasmJitModule* jitRuntimeModule();

EXPORT("jitModule")
WasmJitModule* jitModule();

#endif /* shell_wasmshellcommon_js_h */
