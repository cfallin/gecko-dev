/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*
 * JavaScript "portable baseline interpreter": an interpreter that is
 * capable of running ICs, but without any native code.
 *
 * See the [SMDOC] in vm/PortableBaselineInterpret.h for a high-level
 * overview.
 */

#include "vm/PortableBaselineInterpret.h"

#include "mozilla/Maybe.h"
#include <algorithm>

#include "jsapi.h"

#include "builtin/DataViewObject.h"
#include "builtin/MapObject.h"
#include "builtin/String.h"
#include "debugger/DebugAPI.h"
#include "jit/BaselineFrame.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineJIT.h"
#include "jit/CacheIR.h"
#include "jit/CacheIRCompiler.h"
#include "jit/CacheIRReader.h"
#include "jit/JitFrames.h"
#include "jit/JitScript.h"
#include "jit/JSJitFrameIter.h"
#include "jit/VMFunctions.h"
#include "proxy/DeadObjectProxy.h"
#include "proxy/DOMProxy.h"
#include "vm/AsyncFunction.h"
#include "vm/AsyncIteration.h"
#include "vm/EnvironmentObject.h"
#include "vm/EqualityOperations.h"
#include "vm/GeneratorObject.h"
#include "vm/Interpreter.h"
#include "vm/Iteration.h"
#include "vm/JitActivation.h"
#include "vm/JSScript.h"
#include "vm/Opcodes.h"
#include "vm/PlainObject.h"
#include "vm/Shape.h"
#include "vm/Weval.h"

#include "debugger/DebugAPI-inl.h"
#include "jit/BaselineFrame-inl.h"
#include "jit/JitScript-inl.h"
#include "vm/EnvironmentObject-inl.h"
#include "vm/Interpreter-inl.h"
#include "vm/JSScript-inl.h"
#include "vm/PlainObject-inl.h"

#ifdef ENABLE_JS_PBL_WEVAL
WEVAL_DEFINE_GLOBALS()
#endif

namespace js {
namespace pbl {

using namespace js::jit;

/*
 * Debugging: enable `TRACE_INTERP` for an extremely detailed dump of
 * what PBL is doing at every opcode step.
 */

// #define TRACE_INTERP

#ifdef TRACE_INTERP
#  define TRACE_PRINTF(...) \
    do {                    \
      printf(__VA_ARGS__);  \
      fflush(stdout);       \
    } while (0)
#else
#  define TRACE_PRINTF(...) \
    do {                    \
    } while (0)
#endif

// Whether we are using the "hybrid" strategy for ICs (see the [SMDOC]
// in PortableBaselineInterpret.h for more). This is currently a
// constant, but may become configurable in the future.
static const bool kHybridICs = false;

/*
 * -----------------------------------------------
 * Stack handling
 * -----------------------------------------------
 */

// Large enough for an exit frame.
static const size_t kStackMargin = 1024;

/*
 * A 64-bit value on the auxiliary stack. May either be a raw uint64_t
 * or a `Value` (JS NaN-boxed value).
 */
struct StackVal {
  uint64_t value;

  explicit StackVal(uint64_t v) : value(v) {}
  explicit StackVal(const Value& v) : value(v.asRawBits()) {}

  uint64_t asUInt64() const { return value; }
  Value asValue() const { return Value::fromRawBits(value); }
};

/*
 * A native-pointer-sized value on the auxiliary stack. This is
 * separate from the above because we support running on 32-bit
 * systems as well! May either be a `void*` (or cast to a
 * `CalleeToken`, which is a typedef for a `void*`), or a `uint32_t`,
 * which always fits in a native pointer width on our supported
 * platforms. (See static_assert below.)
 */
struct StackValNative {
  static_assert(sizeof(uintptr_t) >= sizeof(uint32_t),
                "Must be at least a 32-bit system to use PBL.");

  uintptr_t value;

  explicit StackValNative(void* v) : value(reinterpret_cast<uintptr_t>(v)) {}
  explicit StackValNative(uint32_t v) : value(v) {}

  void* asVoidPtr() const { return reinterpret_cast<void*>(value); }
  CalleeToken asCalleeToken() const {
    return reinterpret_cast<CalleeToken>(value);
  }
};

// Assert that the stack alignment is no more than the size of a
// StackValNative -- we rely on this when setting up call frames.
static_assert(JitStackAlignment <= sizeof(StackValNative));

#define PUSH(val) *--sp = (val)
#define POP() (*sp++)
#define POPN(n) sp += (n)

#define PUSHNATIVE(val)                                               \
  do {                                                                \
    StackValNative* nativeSP = reinterpret_cast<StackValNative*>(sp); \
    *--nativeSP = (val);                                              \
    sp = reinterpret_cast<StackVal*>(nativeSP);                       \
  } while (0)
#define POPNNATIVE(n) \
  sp = reinterpret_cast<StackVal*>(reinterpret_cast<StackValNative*>(sp) + (n))

/*
 * Helper class to manage the auxiliary stack and push/pop frames.
 */
struct Stack {
  StackVal* fp;
  StackVal* base;
  StackVal* top;
  StackVal* unwindingSP;

  explicit Stack(PortableBaselineStack& pbs)
      : fp(reinterpret_cast<StackVal*>(pbs.top)),
        base(reinterpret_cast<StackVal*>(pbs.base)),
        top(reinterpret_cast<StackVal*>(pbs.top)),
        unwindingSP(nullptr) {}

  MOZ_ALWAYS_INLINE bool check(StackVal* sp, size_t size, bool margin = true) {
    return reinterpret_cast<uintptr_t>(base) + size +
               (margin ? kStackMargin : 0) <=
           reinterpret_cast<uintptr_t>(sp);
  }

  [[nodiscard]] MOZ_ALWAYS_INLINE StackVal* allocate(StackVal* sp,
                                                     size_t size) {
    if (!check(sp, size, false)) {
      return nullptr;
    }
    sp = reinterpret_cast<StackVal*>(reinterpret_cast<uintptr_t>(sp) - size);
    return sp;
  }

  uint32_t frameSize(StackVal* sp, BaselineFrame* curFrame) const {
    return sizeof(StackVal) * (reinterpret_cast<StackVal*>(fp) - sp);
  }

  [[nodiscard]] MOZ_ALWAYS_INLINE BaselineFrame* pushFrame(StackVal* sp,
                                                           JSContext* cx,
                                                           JSObject* envChain) {
    TRACE_PRINTF("pushFrame: sp = %p fp = %p\n", sp, fp);
    if (sp == base) {
      return nullptr;
    }
    PUSHNATIVE(StackValNative(fp));
    fp = sp;
    TRACE_PRINTF("pushFrame: new fp = %p\n", fp);

    BaselineFrame* frame =
        reinterpret_cast<BaselineFrame*>(allocate(sp, BaselineFrame::Size()));
    if (!frame) {
      return nullptr;
    }

    frame->setFlags(BaselineFrame::Flags::RUNNING_IN_INTERPRETER);
    frame->setEnvironmentChain(envChain);
    JSScript* script = frame->script();
    frame->setICScript(script->jitScript()->icScript());
    frame->setInterpreterFields(script->code());
#ifdef DEBUG
    frame->setDebugFrameSize(0);
#endif
    return frame;
  }

  StackVal* popFrame() {
    StackVal* newTOS =
        reinterpret_cast<StackVal*>(reinterpret_cast<StackValNative*>(fp) + 1);
    fp = reinterpret_cast<StackVal*>(
        reinterpret_cast<StackValNative*>(fp)->asVoidPtr());
    MOZ_ASSERT(fp);
    TRACE_PRINTF("popFrame: fp = %p\n", fp);
    return newTOS;
  }

  void setFrameSize(StackVal* sp, BaselineFrame* prevFrame) {
#ifdef DEBUG
    MOZ_ASSERT(fp != nullptr);
    uintptr_t frameSize =
        reinterpret_cast<uintptr_t>(fp) - reinterpret_cast<uintptr_t>(sp);
    MOZ_ASSERT(reinterpret_cast<uintptr_t>(fp) >=
               reinterpret_cast<uintptr_t>(sp));
    TRACE_PRINTF("pushExitFrame: fp = %p cur() = %p -> frameSize = %d\n", fp,
                 sp, int(frameSize));
    MOZ_ASSERT(frameSize >= BaselineFrame::Size());
    prevFrame->setDebugFrameSize(frameSize);
#endif
  }

  [[nodiscard]] MOZ_ALWAYS_INLINE StackVal* pushExitFrame(
      StackVal* sp, BaselineFrame* prevFrame) {
    uint8_t* prevFP =
        reinterpret_cast<uint8_t*>(prevFrame) + BaselineFrame::Size();
    MOZ_ASSERT(reinterpret_cast<StackVal*>(prevFP) == fp);
    setFrameSize(sp, prevFrame);

    if (!check(sp, sizeof(StackVal) * 4, false)) {
      return nullptr;
    }

    PUSHNATIVE(StackValNative(
        MakeFrameDescriptorForJitCall(FrameType::BaselineJS, 0)));
    PUSHNATIVE(StackValNative(nullptr));  // fake return address.
    PUSHNATIVE(StackValNative(prevFP));
    StackVal* exitFP = sp;
    fp = exitFP;
    TRACE_PRINTF(" -> fp = %p\n", fp);
    PUSHNATIVE(StackValNative(uint32_t(ExitFrameType::Bare)));
    return exitFP;
  }

  void popExitFrame(StackVal* fp) {
    StackVal* prevFP = reinterpret_cast<StackVal*>(
        reinterpret_cast<StackValNative*>(fp)->asVoidPtr());
    MOZ_ASSERT(prevFP);
    this->fp = prevFP;
    TRACE_PRINTF("popExitFrame: fp -> %p\n", fp);
  }

  BaselineFrame* frameFromFP() {
    return reinterpret_cast<BaselineFrame*>(reinterpret_cast<uintptr_t>(fp) -
                                            BaselineFrame::Size());
  }

  static HandleValue handle(StackVal* sp) {
    return HandleValue::fromMarkedLocation(reinterpret_cast<Value*>(sp));
  }
  static MutableHandleValue handleMut(StackVal* sp) {
    return MutableHandleValue::fromMarkedLocation(reinterpret_cast<Value*>(sp));
  }
};

/*
 * -----------------------------------------------
 * Interpreter state
 * -----------------------------------------------
 */

struct ICRegs {
  static const int kMaxICVals = 16;
  uint64_t icVals[kMaxICVals];
  int extraArgs;

  ICRegs() {}
};

struct State {
  RootedValue value0;
  RootedValue value1;
  RootedValue value2;
  RootedValue value3;
  RootedValue res;
  RootedObject obj0;
  RootedObject obj1;
  RootedObject obj2;
  RootedString str0;
  RootedString str1;
  RootedScript script0;
  Rooted<PropertyName*> name0;
  Rooted<jsid> id0;
  Rooted<JSAtom*> atom0;
  RootedFunction fun0;
  Rooted<Scope*> scope0;

  explicit State(JSContext* cx)
      : value0(cx),
        value1(cx),
        value2(cx),
        value3(cx),
        res(cx),
        obj0(cx),
        obj1(cx),
        obj2(cx),
        str0(cx),
        str1(cx),
        script0(cx),
        name0(cx),
        id0(cx),
        atom0(cx),
        fun0(cx),
        scope0(cx) {}
};

/*
 * -----------------------------------------------
 * RAII helpers for pushing exit frames.
 *
 * (See [SMDOC] in PortableBaselineInterpret.h for more.)
 * -----------------------------------------------
 */

class VMFrameManager {
  JSContext* cx;
  BaselineFrame* frame;
  friend class VMFrame;

 public:
  VMFrameManager(JSContext*& cx_, BaselineFrame* frame_)
      : cx(cx_), frame(frame_) {
    // Once the manager exists, we need to create an exit frame to
    // have access to the cx (unless the caller promises it is not
    // calling into the rest of the runtime).
    cx_ = nullptr;
  }

  void switchToFrame(BaselineFrame* frame) { this->frame = frame; }

  // Provides the JSContext, but *only* if no calls into the rest of
  // the runtime (that may invoke a GC or stack walk) occur. Avoids
  // the overhead of pushing an exit frame.
  JSContext* cxForLocalUseOnly() const { return cx; }
};

class VMFrame {
  JSContext* cx;
  Stack& stack;
  StackVal* exitFP;
  void* prevSavedStack;

 public:
  VMFrame(VMFrameManager& mgr, Stack& stack_, StackVal* sp, jsbytecode* pc)
      : cx(mgr.cx), stack(stack_) {
    mgr.frame->interpreterPC() = pc;
    exitFP = stack.pushExitFrame(sp, mgr.frame);
    if (!exitFP) {
      return;
    }
    cx->activation()->asJit()->setJSExitFP(reinterpret_cast<uint8_t*>(exitFP));
    prevSavedStack = cx->portableBaselineStack().top;
    cx->portableBaselineStack().top = reinterpret_cast<void*>(spBelowFrame());
  }

  StackVal* spBelowFrame() {
    return reinterpret_cast<StackVal*>(reinterpret_cast<uintptr_t>(exitFP) -
                                       sizeof(StackValNative));
  }

  ~VMFrame() {
    stack.popExitFrame(exitFP);
    cx->portableBaselineStack().top = prevSavedStack;
  }

  JSContext* getCx() const { return cx; }
  operator JSContext*() const { return cx; }

  bool success() const { return exitFP != nullptr; }
};

#define PUSH_EXIT_FRAME_OR_RET(value)                           \
  VMFrame cx(ctx.frameMgr, ctx.stack, sp, pc);                  \
  if (!cx.success()) {                                          \
    return value;                                               \
  }                                                             \
  StackVal* sp = cx.spBelowFrame(); /* shadow the definition */ \
  (void)sp;                         /* avoid unused-variable warnings */

#define PUSH_IC_FRAME() PUSH_EXIT_FRAME_OR_RET(PBIResult::Error)
#define PUSH_FALLBACK_IC_FRAME() PUSH_EXIT_FRAME_OR_RET(PBIResult::Error)
#define PUSH_EXIT_FRAME() PUSH_EXIT_FRAME_OR_RET(PBIResult::Error)

/*
 * -----------------------------------------------
 * IC Interpreter
 * -----------------------------------------------
 */

// Bundled state for passing to ICs, in order to reduce the number of
// arguments and hence make the call more ABI-efficient. (On some
// platforms, e.g. Wasm on Wasmtime on x86-64, we have as few as four
// register arguments available before args go through the stack.)
struct ICCtx {
  State& state;
  Stack& stack;
  VMFrameManager frameMgr;
  ICRegs icregs;

  jsbytecode* pc;
  StackVal* sp;
  BaselineFrame* frame;

  ICCtx(JSContext* cx, BaselineFrame* frame_, State& state_, Stack& stack_)
      : state(state_),
        stack(stack_),
        frameMgr(cx, frame_),
        icregs(),
        pc(nullptr),
        sp(nullptr),
        frame(frame_) {}
};

// Universal signature for an IC stub function.
typedef PBIResult (*ICStubFunc)(ICCtx& ctx, ICStub* stub,
                                const CacheIRStubInfo* stubInfo,
                                const uint8_t* code, uint64_t arg0,
                                uint64_t arg1, uint64_t arg2, uint64_t* ret);

#define CALL_IC(ctx, stub, result, arg0, arg1, arg2, ret)               \
  do {                                                                  \
    ICStubFunc func = reinterpret_cast<ICStubFunc>(stub->rawJitCode()); \
    result = func(ctx, stub, nullptr, nullptr, arg0, arg1, arg2, ret);  \
  } while (0)

typedef PBIResult (*PBIFunc)(JSContext* cx_, State& state, Stack& stack,
                             StackVal* sp, JSObject* envChain, Value* ret,
                             jsbytecode* pc, ImmutableScriptData* isd,
                             jsbytecode* restartEntryPC,
                             BaselineFrame* restartFrame,
                             StackVal* restartEntryFrame,
                             PBIResult restartCode);

#ifdef ENABLE_JS_PBL_WEVAL
#  define INVOKE_PBI(result, script, interp, ...)                              \
    if (script->hasWeval() && script->weval().func) {                          \
      result = (reinterpret_cast<PBIFunc>(script->weval().func))(__VA_ARGS__); \
    } else {                                                                   \
      result = interp(__VA_ARGS__);                                            \
    }
#else
#  define INVOKE_PBI(result, script, interp, ...) result = interp(__VA_ARGS__);
#endif

// Interpreter for CacheIR.
template <bool Specialized>
PBIResult MOZ_NEVER_INLINE ICInterpretOps(ICCtx& ctx, ICStub* stub,
                                          const CacheIRStubInfo* stubInfo,
                                          const uint8_t* code, uint64_t arg0,
                                          uint64_t arg1, uint64_t arg2,
                                          uint64_t* ret) {
  ICCacheIRStub* cstub = stub->toCacheIRStub();

  if (!Specialized) {
    // Set `stubInfo` and `code`, which will have been `nullptr` in
    // the initial call.
    stubInfo = cstub->stubInfo();
    code = stubInfo->code();
  }

#ifdef ENABLE_JS_PBL_WEVAL
  if (!Specialized) {
    CacheIRStubInfo* s = const_cast<CacheIRStubInfo*>(stubInfo);
    // Lazily propagate function pointer from weval request to stub.
    if (s->hasWeval() && s->weval().func) {
      stub->updateRawJitCode(reinterpret_cast<uint8_t*>(s->weval().func));
      PBIResult result;
      CALL_IC(ctx, stub, result, arg0, arg1, arg2, ret);
      return result;
    }
  }
#endif

  CacheIRReader cacheIRReader(code, nullptr);
  jsbytecode* pc = ctx.pc;
  StackVal* sp = ctx.sp;

  // dispatch logic: non-WASI version does direct threading; WASI
  // version uses a conventional switch (because Wasm lowers to
  // this anyway, and it makes following transforms easier).

#ifndef __wasi__

#  define CACHEOP_CASE(name) cacheop_##name : CACHEOP_TRACE(name)
#  define CACHEOP_CASE_FALLTHROUGH(name) CACHEOP_CASE(name)
#  define CACHEOP_CASE_UNIMPL(name) cacheop_##name:

#  define DISPATCH_CACHEOP()          \
    cacheop = cacheIRReader.readOp(); \
    goto* addresses[long(cacheop)];

#else  // !__wasi__

#  define CACHEOP_CASE(name) \
    case CacheOp::name:      \
      cacheop_##name : CACHEOP_TRACE(name)
#  define CACHEOP_CASE_FALLTHROUGH(name) \
    [[fallthrough]];                     \
    CACHEOP_CASE(name)
#  define CACHEOP_CASE_UNIMPL(name) \
    case CacheOp::name:             \
      cacheop_##name:

#  define DISPATCH_CACHEOP()          \
    cacheop = cacheIRReader.readOp(); \
    WEVAL_UPDATE_IC_CTX();            \
    goto dispatch;

#endif  // __wasi__

  // Define the computed-goto table regardless of dispatch strategy so
  // we don't get unused-label errors. (We need some of the labels
  // even without this for the predict-next mechanism, so we can't
  // conditionally elide labels either.)
  static const void* const addresses[long(CacheOp::NumOpcodes)] = {
#define OP(name, ...) &&cacheop_##name,
      CACHE_IR_OPS(OP)
#undef OP
  };
  (void)addresses;

#define CACHEOP_TRACE(name)                                                    \
  TRACE_PRINTF("cacheop (frame %p pc %p stub %p): " #name "\n", ctx.frame, pc, \
               cstub);

#define FAIL_IC() goto next_ic;

// We set a fixed bound on the number of icVals which is smaller than what IC
// generators may use. As a result we can't evaluate an IC if it defines too
// many values. Note that we don't need to check this when reading from icVals
// because we should have bailed out before the earlier write which defined the
// same value. Similarly, we don't need to check writes to locations which we've
// just read from.
#define BOUNDSCHECK(resultId) \
  if (resultId.id() >= ICRegs::kMaxICVals) FAIL_IC();

#define PREDICT_NEXT(name)                                       \
  if (!Specialized && cacheIRReader.peekOp() == CacheOp::name) { \
    cacheIRReader.readOp();                                      \
    goto cacheop_##name;                                         \
  }

#define PREDICT_RETURN()                                                 \
  if (!Specialized && cacheIRReader.peekOp() == CacheOp::ReturnFromIC) { \
    TRACE_PRINTF("stub successful, predicted return\n");                 \
    return PBIResult::Ok;                                                \
  }

#ifdef ENABLE_JS_PBL_WEVAL

#  define READ_REG(reg) \
    (Specialized ? weval_read_reg((reg)) : ctx.icregs.icVals[(reg)])
#  define WRITE_REG(reg, value)           \
    if (Specialized) {                    \
      weval_write_reg((reg), (value));    \
    } else {                              \
      ctx.icregs.icVals[(reg)] = (value); \
    }

#  define WEVAL_UPDATE_IC_CTX()                                         \
    if (Specialized) {                                                  \
      weval::update_context(                                            \
          reinterpret_cast<uint32_t>(cacheIRReader.currentPosition())); \
    }

#else  // ENABLE_JS_PBL_WEVAL

#  define READ_REG(reg) ctx.icregs.icVals[(reg)]
#  define WRITE_REG(reg, value) ctx.icregs.icVals[(reg)] = (value)

#  define WEVAL_UPDATE_IC_CTX() ;

#endif  // !ENABLE_JS_PBL_WEVAL

  CacheOp cacheop;

#ifdef ENABLE_JS_PBL_WEVAL
  weval::push_context(
      reinterpret_cast<uint32_t>(cacheIRReader.currentPosition()));
#endif

  WRITE_REG(0, arg0);
  WRITE_REG(1, arg1);
  WRITE_REG(2, arg2);

  DISPATCH_CACHEOP();

  while (true) {
#ifdef __wasi__
  dispatch:
    switch (cacheop)
#endif
    {

      CACHEOP_CASE(ReturnFromIC) {
        TRACE_PRINTF("stub successful!\n");
        return PBIResult::Ok;
      }

      CACHEOP_CASE(GuardToObject) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        TRACE_PRINTF("GuardToObject: icVal %" PRIx64 "\n",
                     READ_REG(inputId.id()));
        if (!v.isObject()) {
          FAIL_IC();
        }
        WRITE_REG(inputId.id(), reinterpret_cast<uint64_t>(&v.toObject()));
        PREDICT_NEXT(GuardShape);
        PREDICT_NEXT(GuardSpecificFunction);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNullOrUndefined) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isNullOrUndefined()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNull) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isNull()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsUndefined) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isUndefined()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNotUninitializedLexical) {
        ValOperandId valId = cacheIRReader.valOperandId();
        Value val = Value::fromRawBits(READ_REG(valId.id()));
        if (val == MagicValue(JS_UNINITIALIZED_LEXICAL)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToBoolean) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isBoolean()) {
          FAIL_IC();
        }
        WRITE_REG(inputId.id(), v.toBoolean() ? 1 : 0);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToString) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isString()) {
          FAIL_IC();
        }
        WRITE_REG(inputId.id(), reinterpret_cast<uint64_t>(v.toString()));
        PREDICT_NEXT(GuardToString);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToSymbol) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isSymbol()) {
          FAIL_IC();
        }
        WRITE_REG(inputId.id(), reinterpret_cast<uint64_t>(v.toSymbol()));
        PREDICT_NEXT(GuardSpecificSymbol);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToBigInt) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isBigInt()) {
          FAIL_IC();
        }
        WRITE_REG(inputId.id(), reinterpret_cast<uint64_t>(v.toBigInt()));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNumber) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isNumber()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToInt32) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        TRACE_PRINTF("GuardToInt32 (%d): icVal %" PRIx64 "\n", inputId.id(),
                     READ_REG(inputId.id()));
        if (!v.isInt32()) {
          FAIL_IC();
        }
        // N.B.: we don't need to unbox because the low 32 bits are
        // already the int32 itself, and we are careful when using
        // `Int32Operand`s to only use those bits.

        PREDICT_NEXT(GuardToInt32);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToNonGCThing) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value input = Value::fromRawBits(READ_REG(inputId.id()));
        if (input.isGCThing()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardBooleanToInt32) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        Value v = Value::fromRawBits(READ_REG(inputId.id()));
        if (!v.isBoolean()) {
          FAIL_IC();
        }
        WRITE_REG(resultId.id(), v.toBoolean() ? 1 : 0);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToInt32Index) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        Value val = Value::fromRawBits(READ_REG(inputId.id()));
        if (val.isInt32()) {
          WRITE_REG(resultId.id(), val.toInt32());
          DISPATCH_CACHEOP();
        } else if (val.isDouble()) {
          double doubleVal = val.toDouble();
          if (doubleVal >= double(INT32_MIN) &&
              doubleVal <= double(INT32_MAX)) {
            WRITE_REG(resultId.id(), int32_t(doubleVal));
            DISPATCH_CACHEOP();
          }
        }
        FAIL_IC();
      }

      CACHEOP_CASE(Int32ToIntPtr) {
        Int32OperandId inputId = cacheIRReader.int32OperandId();
        IntPtrOperandId resultId = cacheIRReader.intPtrOperandId();
        BOUNDSCHECK(resultId);
        int32_t input = int32_t(READ_REG(inputId.id()));
        // Note that this must sign-extend to pointer width:
        WRITE_REG(resultId.id(), intptr_t(input));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardToInt32ModUint32) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        Value input = Value::fromRawBits(READ_REG(inputId.id()));
        if (input.isInt32()) {
          WRITE_REG(resultId.id(), Int32Value(input.toInt32()).asRawBits());
          DISPATCH_CACHEOP();
        } else if (input.isDouble()) {
          double doubleVal = input.toDouble();
          // Accept any double that fits in an int64_t but truncate the top 32
          // bits.
          if (doubleVal >= double(INT64_MIN) &&
              doubleVal <= double(INT64_MAX)) {
            WRITE_REG(resultId.id(),
                      Int32Value(int64_t(doubleVal)).asRawBits());
            DISPATCH_CACHEOP();
          }
        }
        FAIL_IC();
      }

      CACHEOP_CASE(GuardNonDoubleType) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        ValueType type = cacheIRReader.valueType();
        Value val = Value::fromRawBits(READ_REG(inputId.id()));
        switch (type) {
          case ValueType::String:
            if (!val.isString()) {
              FAIL_IC();
            }
            break;
          case ValueType::Symbol:
            if (!val.isSymbol()) {
              FAIL_IC();
            }
            break;
          case ValueType::BigInt:
            if (!val.isBigInt()) {
              FAIL_IC();
            }
            break;
          case ValueType::Int32:
            if (!val.isInt32()) {
              FAIL_IC();
            }
            break;
          case ValueType::Boolean:
            if (!val.isBoolean()) {
              FAIL_IC();
            }
            break;
          case ValueType::Undefined:
            if (!val.isUndefined()) {
              FAIL_IC();
            }
            break;
          case ValueType::Null:
            if (!val.isNull()) {
              FAIL_IC();
            }
            break;
          default:
            MOZ_CRASH("Unexpected type");
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardShape) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t shapeOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uintptr_t expectedShape =
            cstub->stubInfo()->getStubRawWord(cstub, shapeOffset);
        if (reinterpret_cast<uintptr_t>(obj->shape()) != expectedShape) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFuse) {
        RealmFuses::FuseIndex fuseIndex = cacheIRReader.realmFuseIndex();
        if (!ctx.frameMgr.cxForLocalUseOnly()
                 ->realm()
                 ->realmFuses.getFuseByIndex(fuseIndex)
                 ->intact()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardProto) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t protoOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSObject* proto = reinterpret_cast<JSObject*>(
            cstub->stubInfo()->getStubRawWord(cstub, protoOffset));
        if (obj->staticPrototype() != proto) {
          FAIL_IC();
        }
        PREDICT_NEXT(LoadProto);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardNullProto) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->taggedProto().raw()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardClass) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        GuardClassKind kind = cacheIRReader.guardClassKind();
        JSObject* object = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        switch (kind) {
          case GuardClassKind::Array:
            if (object->getClass() != &ArrayObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::PlainObject:
            if (object->getClass() != &PlainObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::FixedLengthArrayBuffer:
            if (object->getClass() != &FixedLengthArrayBufferObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::FixedLengthSharedArrayBuffer:
            if (object->getClass() !=
                &FixedLengthSharedArrayBufferObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::FixedLengthDataView:
            if (object->getClass() != &FixedLengthDataViewObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::MappedArguments:
            if (object->getClass() != &MappedArgumentsObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::UnmappedArguments:
            if (object->getClass() != &UnmappedArgumentsObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::WindowProxy:
            if (object->getClass() != ctx.frameMgr.cxForLocalUseOnly()
                                          ->runtime()
                                          ->maybeWindowProxyClass()) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::JSFunction:
            if (!object->is<JSFunction>()) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::Set:
            if (object->getClass() != &SetObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::Map:
            if (object->getClass() != &MapObject::class_) {
              FAIL_IC();
            }
            break;
          case GuardClassKind::BoundFunction:
            if (object->getClass() != &BoundFunctionObject::class_) {
              FAIL_IC();
            }
            break;
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardAnyClass) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t claspOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSClass* clasp = reinterpret_cast<JSClass*>(
            cstub->stubInfo()->getStubRawWord(cstub, claspOffset));
        if (obj->getClass() != clasp) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardGlobalGeneration) {
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        uint32_t generationAddrOffset = cacheIRReader.stubOffset();
        uint32_t expected =
            cstub->stubInfo()->getStubRawInt32(cstub, expectedOffset);
        uint32_t* generationAddr = reinterpret_cast<uint32_t*>(
            cstub->stubInfo()->getStubRawWord(cstub, generationAddrOffset));
        if (*generationAddr != expected) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(HasClassResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t claspOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSClass* clasp = reinterpret_cast<JSClass*>(
            cstub->stubInfo()->getStubRawWord(cstub, claspOffset));
        *ret = BooleanValue(obj->getClass() == clasp).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardCompartment) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t globalOffset = cacheIRReader.stubOffset();
        uint32_t compartmentOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSObject* global = reinterpret_cast<JSObject*>(
            cstub->stubInfo()->getStubRawWord(cstub, globalOffset));
        JS::Compartment* compartment = reinterpret_cast<JS::Compartment*>(
            cstub->stubInfo()->getStubRawWord(cstub, compartmentOffset));
        if (IsDeadProxyObject(global)) {
          FAIL_IC();
        }
        if (obj->compartment() != compartment) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsExtensible) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->nonProxyIsExtensible()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNativeObject) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (!obj->is<NativeObject>()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsProxy) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (!obj->is<ProxyObject>()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNotProxy) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->is<ProxyObject>()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNotArrayBufferMaybeShared) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        const JSClass* clasp = obj->getClass();
        if (clasp == &ArrayBufferObject::protoClass_ ||
            clasp == &SharedArrayBufferObject::protoClass_) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsTypedArray) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (!IsTypedArrayClass(obj->getClass())) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardHasProxyHandler) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t handlerOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        BaseProxyHandler* handler = reinterpret_cast<BaseProxyHandler*>(
            cstub->stubInfo()->getStubRawWord(cstub, handlerOffset));
        if (obj->as<ProxyObject>().handler() != handler) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardIsNotDOMProxy) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->as<ProxyObject>().handler()->family() ==
            GetDOMProxyHandlerFamily()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardSpecificObject) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSObject* expected = reinterpret_cast<JSObject*>(
            cstub->stubInfo()->getStubRawWord(cstub, expectedOffset));
        if (obj != expected) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardObjectIdentity) {
        ObjOperandId obj1Id = cacheIRReader.objOperandId();
        ObjOperandId obj2Id = cacheIRReader.objOperandId();
        JSObject* obj1 = reinterpret_cast<JSObject*>(READ_REG(obj1Id.id()));
        JSObject* obj2 = reinterpret_cast<JSObject*>(READ_REG(obj2Id.id()));
        if (obj1 != obj2) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardSpecificFunction) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        uint32_t nargsAndFlagsOffset = cacheIRReader.stubOffset();
        (void)nargsAndFlagsOffset;  // Unused.
        uintptr_t expected =
            cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
        if (expected != READ_REG(funId.id())) {
          FAIL_IC();
        }
        PREDICT_NEXT(LoadArgumentFixedSlot);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFunctionScript) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        uint32_t nargsAndFlagsOffset = cacheIRReader.stubOffset();
        JSFunction* fun = reinterpret_cast<JSFunction*>(READ_REG(objId.id()));
        BaseScript* expected = reinterpret_cast<BaseScript*>(
            cstub->stubInfo()->getStubRawWord(cstub, expectedOffset));
        (void)nargsAndFlagsOffset;

        if (!fun->hasBaseScript() || fun->baseScript() != expected) {
          FAIL_IC();
        }

        PREDICT_NEXT(CallScriptedFunction);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardSpecificAtom) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        uintptr_t expected =
            cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
        if (expected != READ_REG(strId.id())) {
          // TODO: BaselineCacheIRCompiler also checks for equal strings
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardSpecificSymbol) {
        SymbolOperandId symId = cacheIRReader.symbolOperandId();
        uint32_t expectedOffset = cacheIRReader.stubOffset();
        uintptr_t expected =
            cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
        if (expected != READ_REG(symId.id())) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardSpecificInt32) {
        Int32OperandId numId = cacheIRReader.int32OperandId();
        int32_t expected = cacheIRReader.int32Immediate();
        if (expected != int32_t(READ_REG(numId.id()))) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardNoDenseElements) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->as<NativeObject>().getDenseInitializedLength() != 0) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardStringToIndex) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        JSString* str = reinterpret_cast<JSString*>(READ_REG(strId.id()));
        int32_t result;
        if (str->hasIndexValue()) {
          uint32_t index = str->getIndexValue();
          MOZ_ASSERT(index <= INT32_MAX);
          result = index;
        } else {
          result = GetIndexFromString(str);
          if (result < 0) {
            FAIL_IC();
          }
        }
        WRITE_REG(resultId.id(), result);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardStringToInt32) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        JSString* str = reinterpret_cast<JSString*>(READ_REG(strId.id()));
        int32_t result;
        // Use indexed value as fast path if possible.
        if (str->hasIndexValue()) {
          uint32_t index = str->getIndexValue();
          MOZ_ASSERT(index <= INT32_MAX);
          result = index;
        } else {
          if (!GetInt32FromStringPure(ctx.frameMgr.cxForLocalUseOnly(), str,
                                      &result)) {
            FAIL_IC();
          }
        }
        WRITE_REG(resultId.id(), result);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardStringToNumber) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        NumberOperandId resultId = cacheIRReader.numberOperandId();
        BOUNDSCHECK(resultId);
        JSString* str = reinterpret_cast<JSString*>(READ_REG(strId.id()));
        Value result;
        // Use indexed value as fast path if possible.
        if (str->hasIndexValue()) {
          uint32_t index = str->getIndexValue();
          MOZ_ASSERT(index <= INT32_MAX);
          result = Int32Value(index);
        } else {
          double value;
          if (!StringToNumberPure(ctx.frameMgr.cxForLocalUseOnly(), str,
                                  &value)) {
            FAIL_IC();
          }
          result = DoubleValue(value);
        }
        WRITE_REG(resultId.id(), result.asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(BooleanToNumber) {
        BooleanOperandId booleanId = cacheIRReader.booleanOperandId();
        NumberOperandId resultId = cacheIRReader.numberOperandId();
        BOUNDSCHECK(resultId);
        uint64_t boolean = READ_REG(booleanId.id());
        MOZ_ASSERT((boolean & ~1) == 0);
        WRITE_REG(resultId.id(), Int32Value(boolean).asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardHasGetterSetter) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t idOffset = cacheIRReader.stubOffset();
        uint32_t getterSetterOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        jsid id = jsid::fromRawBits(
            cstub->stubInfo()->getStubRawWord(cstub, idOffset));
        GetterSetter* getterSetter = reinterpret_cast<GetterSetter*>(
            cstub->stubInfo()->getStubRawWord(cstub, getterSetterOffset));
        if (!ObjectHasGetterSetterPure(ctx.frameMgr.cxForLocalUseOnly(), obj,
                                       id, getterSetter)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardInt32IsNonNegative) {
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        int32_t index = int32_t(READ_REG(indexId.id()));
        if (index < 0) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardDynamicSlotIsSpecificObject) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ObjOperandId expectedId = cacheIRReader.objOperandId();
        uint32_t slotOffset = cacheIRReader.stubOffset();
        JSObject* expected =
            reinterpret_cast<JSObject*>(READ_REG(expectedId.id()));
        uintptr_t slot = cstub->stubInfo()->getStubRawInt32(cstub, slotOffset);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        HeapSlot* slots = nobj->getSlotsUnchecked();
        // Note that unlike similar opcodes, GuardDynamicSlotIsSpecificObject
        // takes a slot index rather than a byte offset.
        Value actual = slots[slot];
        if (actual != ObjectValue(*expected)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardDynamicSlotIsNotObject) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t slotOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t slot = cstub->stubInfo()->getStubRawInt32(cstub, slotOffset);
        NativeObject* nobj = &obj->as<NativeObject>();
        HeapSlot* slots = nobj->getSlotsUnchecked();
        // Note that unlike similar opcodes, GuardDynamicSlotIsNotObject takes a
        // slot index rather than a byte offset.
        Value actual = slots[slot];
        if (actual.isObject()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFixedSlotValue) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        uint32_t valOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        Value val = Value::fromRawBits(
            cstub->stubInfo()->getStubRawInt64(cstub, valOffset));
        GCPtr<Value>* slot = reinterpret_cast<GCPtr<Value>*>(
            reinterpret_cast<uintptr_t>(obj) + offset);
        Value actual = slot->get();
        if (actual != val) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardDynamicSlotValue) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        uint32_t valOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        Value val = Value::fromRawBits(
            cstub->stubInfo()->getStubRawInt64(cstub, valOffset));
        NativeObject* nobj = &obj->as<NativeObject>();
        HeapSlot* slots = nobj->getSlotsUnchecked();
        Value actual = slots[offset / sizeof(Value)];
        if (actual != val) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadFixedSlot) {
        ValOperandId resultId = cacheIRReader.valOperandId();
        BOUNDSCHECK(resultId);
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        GCPtr<Value>* slot = reinterpret_cast<GCPtr<Value>*>(
            reinterpret_cast<uintptr_t>(obj) + offset);
        Value actual = slot->get();
        WRITE_REG(resultId.id(), actual.asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadDynamicSlot) {
        ValOperandId resultId = cacheIRReader.valOperandId();
        BOUNDSCHECK(resultId);
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t slotOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t slot = cstub->stubInfo()->getStubRawInt32(cstub, slotOffset);
        NativeObject* nobj = &obj->as<NativeObject>();
        HeapSlot* slots = nobj->getSlotsUnchecked();
        // Note that unlike similar opcodes, LoadDynamicSlot takes a slot index
        // rather than a byte offset.
        Value actual = slots[slot];
        WRITE_REG(resultId.id(), actual.asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardNoAllocationMetadataBuilder) {
        uint32_t builderAddrOffset = cacheIRReader.stubOffset();
        uintptr_t builderAddr =
            cstub->stubInfo()->getStubRawWord(cstub, builderAddrOffset);
        if (*reinterpret_cast<uintptr_t*>(builderAddr) != 0) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFunctionHasJitEntry) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        bool constructing = cacheIRReader.readBool();
        JSObject* fun = reinterpret_cast<JSObject*>(READ_REG(funId.id()));
        uint16_t flags = FunctionFlags::HasJitEntryFlags(constructing);
        if (!fun->as<JSFunction>().flags().hasFlags(flags)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFunctionHasNoJitEntry) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        JSObject* fun = reinterpret_cast<JSObject*>(READ_REG(funId.id()));
        uint16_t flags =
            FunctionFlags::HasJitEntryFlags(/*constructing =*/false);
        if (fun->as<JSFunction>().flags().hasFlags(flags)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFunctionIsNonBuiltinCtor) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        JSObject* fun = reinterpret_cast<JSObject*>(READ_REG(funId.id()));
        if (!fun->as<JSFunction>().isNonBuiltinConstructor()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardFunctionIsConstructor) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        JSObject* fun = reinterpret_cast<JSObject*>(READ_REG(funId.id()));
        if (!fun->as<JSFunction>().isConstructor()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardNotClassConstructor) {
        ObjOperandId funId = cacheIRReader.objOperandId();
        JSObject* fun = reinterpret_cast<JSObject*>(READ_REG(funId.id()));
        if (fun->as<JSFunction>().isClassConstructor()) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardArrayIsPacked) {
        ObjOperandId arrayId = cacheIRReader.objOperandId();
        JSObject* array = reinterpret_cast<JSObject*>(READ_REG(arrayId.id()));
        if (!IsPackedArray(array)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(GuardArgumentsObjectFlags) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint8_t flags = cacheIRReader.readByte();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        if (obj->as<ArgumentsObject>().hasFlags(flags)) {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadObject) {
        ObjOperandId resultId = cacheIRReader.objOperandId();
        BOUNDSCHECK(resultId);
        uint32_t objOffset = cacheIRReader.stubOffset();
        intptr_t obj = cstub->stubInfo()->getStubRawWord(cstub, objOffset);
        WRITE_REG(resultId.id(), obj);
        PREDICT_NEXT(GuardShape);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadProtoObject) {
        ObjOperandId resultId = cacheIRReader.objOperandId();
        BOUNDSCHECK(resultId);
        uint32_t protoObjOffset = cacheIRReader.stubOffset();
        ObjOperandId receiverObjId = cacheIRReader.objOperandId();
        (void)receiverObjId;
        intptr_t obj = cstub->stubInfo()->getStubRawWord(cstub, protoObjOffset);
        WRITE_REG(resultId.id(), obj);
        PREDICT_NEXT(GuardShape);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadProto) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ObjOperandId resultId = cacheIRReader.objOperandId();
        BOUNDSCHECK(resultId);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        WRITE_REG(resultId.id(),
                  reinterpret_cast<uintptr_t>(nobj->staticPrototype()));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadEnclosingEnvironment) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ObjOperandId resultId = cacheIRReader.objOperandId();
        BOUNDSCHECK(resultId);
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSObject* env = &obj->as<EnvironmentObject>().enclosingEnvironment();
        WRITE_REG(resultId.id(), reinterpret_cast<uintptr_t>(env));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadWrapperTarget) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ObjOperandId resultId = cacheIRReader.objOperandId();
        BOUNDSCHECK(resultId);
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        JSObject* target = &obj->as<ProxyObject>().private_().toObject();
        WRITE_REG(resultId.id(), reinterpret_cast<uintptr_t>(target));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadValueTag) {
        ValOperandId valId = cacheIRReader.valOperandId();
        ValueTagOperandId resultId = cacheIRReader.valueTagOperandId();
        BOUNDSCHECK(resultId);
        Value val = Value::fromRawBits(READ_REG(valId.id()));
        WRITE_REG(resultId.id(), val.extractNonDoubleType());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadArgumentFixedSlot) {
        ValOperandId resultId = cacheIRReader.valOperandId();
        BOUNDSCHECK(resultId);
        uint8_t slotIndex = cacheIRReader.readByte();
        Value val = sp[slotIndex].asValue();
        TRACE_PRINTF(" -> slot %d: val %" PRIx64 "\n", int(slotIndex),
                     val.asRawBits());
        WRITE_REG(resultId.id(), val.asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadArgumentDynamicSlot) {
        ValOperandId resultId = cacheIRReader.valOperandId();
        BOUNDSCHECK(resultId);
        Int32OperandId argcId = cacheIRReader.int32OperandId();
        uint8_t slotIndex = cacheIRReader.readByte();
        int32_t argc = int32_t(READ_REG(argcId.id()));
        Value val = sp[slotIndex + argc].asValue();
        WRITE_REG(resultId.id(), val.asRawBits());
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(TruncateDoubleToUInt32) {
        NumberOperandId inputId = cacheIRReader.numberOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        Value input = Value::fromRawBits(READ_REG(inputId.id()));
        WRITE_REG(resultId.id(), JS::ToInt32(input.toNumber()));
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(MegamorphicLoadSlotResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t nameOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        jsid name = jsid::fromRawBits(
            cstub->stubInfo()->getStubRawWord(cstub, nameOffset));
        if (!obj->shape()->isNative()) {
          FAIL_IC();
        }
        Value result;
        if (!GetNativeDataPropertyPureWithCacheLookup(
                ctx.frameMgr.cxForLocalUseOnly(), obj, name, nullptr,
                &result)) {
          FAIL_IC();
        }
        *ret = result.asRawBits();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(MegamorphicLoadSlotByValueResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ValOperandId idId = cacheIRReader.valOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        Value id = Value::fromRawBits(READ_REG(idId.id()));
        if (!obj->shape()->isNative()) {
          FAIL_IC();
        }
        Value values[2] = {id};
        if (!GetNativeDataPropertyByValuePure(ctx.frameMgr.cxForLocalUseOnly(),
                                              obj, nullptr, values)) {
          FAIL_IC();
        }
        *ret = values[1].asRawBits();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(MegamorphicSetElement) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ValOperandId idId = cacheIRReader.valOperandId();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        bool strict = cacheIRReader.readBool();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        Value id = Value::fromRawBits(READ_REG(idId.id()));
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        {
          PUSH_IC_FRAME();
          ReservedRooted<JSObject*> obj0(&ctx.state.obj0, obj);
          ReservedRooted<Value> value0(&ctx.state.value0, id);
          ReservedRooted<Value> value1(&ctx.state.value1, rhs);
          if (!SetElementMegamorphic<false>(cx, obj0, value0, value1, strict)) {
            return PBIResult::Error;
          }
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(StoreFixedSlot) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        uintptr_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        GCPtr<Value>* slot = reinterpret_cast<GCPtr<Value>*>(
            reinterpret_cast<uintptr_t>(nobj) + offset);
        Value val = Value::fromRawBits(READ_REG(rhsId.id()));
        slot->set(val);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(StoreDynamicSlot) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        uint32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        HeapSlot* slots = nobj->getSlotsUnchecked();
        Value val = Value::fromRawBits(READ_REG(rhsId.id()));
        size_t dynSlot = offset / sizeof(Value);
        size_t slot = dynSlot + nobj->numFixedSlots();
        slots[dynSlot].set(nobj, HeapSlot::Slot, slot, val);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(AddAndStoreFixedSlot) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        uint32_t newShapeOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        int32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        Shape* newShape = reinterpret_cast<Shape*>(
            cstub->stubInfo()->getStubRawWord(cstub, newShapeOffset));
        obj->setShape(newShape);
        GCPtr<Value>* slot = reinterpret_cast<GCPtr<Value>*>(
            reinterpret_cast<uintptr_t>(obj) + offset);
        slot->init(rhs);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(AddAndStoreDynamicSlot) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        uint32_t newShapeOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        int32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        Shape* newShape = reinterpret_cast<Shape*>(
            cstub->stubInfo()->getStubRawWord(cstub, newShapeOffset));
        NativeObject* nobj = &obj->as<NativeObject>();
        obj->setShape(newShape);
        HeapSlot* slots = nobj->getSlotsUnchecked();
        size_t dynSlot = offset / sizeof(Value);
        size_t slot = dynSlot + nobj->numFixedSlots();
        slots[dynSlot].init(nobj, HeapSlot::Slot, slot, rhs);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(AllocateAndStoreDynamicSlot) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        uint32_t newShapeOffset = cacheIRReader.stubOffset();
        uint32_t numNewSlotsOffset = cacheIRReader.stubOffset();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        int32_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        Shape* newShape = reinterpret_cast<Shape*>(
            cstub->stubInfo()->getStubRawWord(cstub, newShapeOffset));
        int32_t numNewSlots =
            cstub->stubInfo()->getStubRawInt32(cstub, numNewSlotsOffset);
        NativeObject* nobj = &obj->as<NativeObject>();
        // We have to (re)allocate dynamic slots. Do this first, as it's the
        // only fallible operation here. Note that growSlotsPure is fallible but
        // does not GC. Otherwise this is the same as AddAndStoreDynamicSlot
        // above.
        if (!NativeObject::growSlotsPure(ctx.frameMgr.cxForLocalUseOnly(), nobj,
                                         numNewSlots)) {
          FAIL_IC();
        }
        obj->setShape(newShape);
        HeapSlot* slots = nobj->getSlotsUnchecked();
        size_t dynSlot = offset / sizeof(Value);
        size_t slot = dynSlot + nobj->numFixedSlots();
        slots[dynSlot].init(nobj, HeapSlot::Slot, slot, rhs);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(StoreDenseElement) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        ObjectElements* elems = nobj->getElementsHeader();
        int32_t index = int32_t(READ_REG(indexId.id()));
        if (index < 0 || uint32_t(index) >= nobj->getDenseInitializedLength()) {
          FAIL_IC();
        }
        HeapSlot* slot = &elems->elements()[index];
        if (slot->get().isMagic()) {
          FAIL_IC();
        }
        Value val = Value::fromRawBits(READ_REG(rhsId.id()));
        slot->set(nobj, HeapSlot::Element, index + elems->numShiftedElements(),
                  val);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(StoreDenseElementHole) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        bool handleAdd = cacheIRReader.readBool();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t index = uint32_t(READ_REG(indexId.id()));
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        NativeObject* nobj = &obj->as<NativeObject>();
        uint32_t initLength = nobj->getDenseInitializedLength();
        if (index < initLength) {
          nobj->setDenseElement(index, rhs);
        } else if (!handleAdd || index > initLength) {
          FAIL_IC();
        } else {
          if (index >= nobj->getDenseCapacity()) {
            if (!NativeObject::addDenseElementPure(
                    ctx.frameMgr.cxForLocalUseOnly(), nobj)) {
              FAIL_IC();
            }
          }
          nobj->setDenseInitializedLength(initLength + 1);

          // Baseline always updates the length field by directly accessing its
          // offset in ObjectElements. If the object is not an ArrayObject then
          // this field is never read, so it's okay to skip the update here in
          // that case.
          if (nobj->is<ArrayObject>()) {
            ArrayObject* aobj = &nobj->as<ArrayObject>();
            uint32_t len = aobj->length();
            if (len <= index) {
              aobj->setLength(len + 1);
            }
          }

          nobj->initDenseElement(index, rhs);
        }
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(ArrayPush) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ValOperandId rhsId = cacheIRReader.valOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        Value rhs = Value::fromRawBits(READ_REG(rhsId.id()));
        ArrayObject* aobj = &obj->as<ArrayObject>();
        uint32_t initLength = aobj->getDenseInitializedLength();
        if (aobj->length() != initLength) {
          FAIL_IC();
        }
        if (initLength >= aobj->getDenseCapacity()) {
          if (!NativeObject::addDenseElementPure(
                  ctx.frameMgr.cxForLocalUseOnly(), aobj)) {
            FAIL_IC();
          }
        }
        aobj->setDenseInitializedLength(initLength + 1);
        aobj->setLength(initLength + 1);
        aobj->initDenseElement(initLength, rhs);
        *ret = Int32Value(initLength + 1).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(IsObjectResult) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value val = Value::fromRawBits(READ_REG(inputId.id()));
        *ret = BooleanValue(val.isObject()).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(Int32MinMax) {
        bool isMax = cacheIRReader.readBool();
        Int32OperandId firstId = cacheIRReader.int32OperandId();
        Int32OperandId secondId = cacheIRReader.int32OperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        int32_t lhs = int32_t(READ_REG(firstId.id()));
        int32_t rhs = int32_t(READ_REG(secondId.id()));
        int32_t result = ((lhs > rhs) ^ isMax) ? rhs : lhs;
        WRITE_REG(resultId.id(), result);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(StoreTypedArrayElement) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Scalar::Type elementType = cacheIRReader.scalarType();
        IntPtrOperandId indexId = cacheIRReader.intPtrOperandId();
        uint32_t rhsId = cacheIRReader.rawOperandId();
        bool handleOOB = cacheIRReader.readBool();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uintptr_t index = uintptr_t(READ_REG(indexId.id()));
        uint64_t rhs = READ_REG(rhsId);
        if (obj->as<TypedArrayObject>().length().isNothing()) {
          FAIL_IC();
        }
        if (index >= obj->as<TypedArrayObject>().length().value()) {
          if (!handleOOB) {
            FAIL_IC();
          }
        } else {
          Value v;
          switch (elementType) {
            case Scalar::Int8:
            case Scalar::Uint8:
            case Scalar::Int16:
            case Scalar::Uint16:
            case Scalar::Int32:
            case Scalar::Uint32:
            case Scalar::Uint8Clamped:
              v = Int32Value(rhs);
              break;

            case Scalar::Float32:
            case Scalar::Float64:
              v = Value::fromRawBits(rhs);
              MOZ_ASSERT(v.isNumber());
              break;

            case Scalar::BigInt64:
            case Scalar::BigUint64:
              v = BigIntValue(reinterpret_cast<JS::BigInt*>(rhs));
              break;

            case Scalar::MaxTypedArrayViewType:
            case Scalar::Int64:
            case Scalar::Simd128:
              MOZ_CRASH("Unsupported TypedArray type");
          }

          // SetTypedArrayElement doesn't do anything that can actually GC or
          // need a new context when the value can only be Int32, Double, or
          // BigInt, as the above switch statement enforces.
          FakeRooted<TypedArrayObject*> obj0(nullptr,
                                             &obj->as<TypedArrayObject>());
          FakeRooted<Value> value0(nullptr, v);
          ObjectOpResult result;
          MOZ_ASSERT(elementType == obj0->type());
          MOZ_ALWAYS_TRUE(SetTypedArrayElement(ctx.frameMgr.cxForLocalUseOnly(),
                                               obj0, index, value0, result));
          MOZ_ALWAYS_TRUE(result.ok());
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CallInt32ToString) {
        Int32OperandId inputId = cacheIRReader.int32OperandId();
        StringOperandId resultId = cacheIRReader.stringOperandId();
        BOUNDSCHECK(resultId);
        int32_t input = int32_t(READ_REG(inputId.id()));
        JSLinearString* str =
            Int32ToStringPure(ctx.frameMgr.cxForLocalUseOnly(), input);
        if (str) {
          WRITE_REG(resultId.id(), reinterpret_cast<uintptr_t>(str));
        } else {
          FAIL_IC();
        }
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CallScriptedFunction)
      CACHEOP_CASE_FALLTHROUGH(CallNativeFunction) {
        bool isNative = cacheop == CacheOp::CallNativeFunction;
        TRACE_PRINTF("CallScriptedFunction / CallNativeFunction (native: %d)\n",
                     isNative);
        ObjOperandId calleeId = cacheIRReader.objOperandId();
        Int32OperandId argcId = cacheIRReader.int32OperandId();
        CallFlags flags = cacheIRReader.callFlags();
        uint32_t argcFixed = cacheIRReader.uint32Immediate();
        bool ignoresRv = false;
        if (isNative) {
          ignoresRv = cacheIRReader.readBool();
        }

        JSFunction* callee =
            reinterpret_cast<JSFunction*>(READ_REG(calleeId.id()));
        uint32_t argc = uint32_t(READ_REG(argcId.id()));
        (void)argcFixed;

        if (!isNative) {
          if (!callee->hasBaseScript() ||
              !callee->baseScript()->hasBytecode() ||
              !callee->baseScript()->hasJitScript()) {
            FAIL_IC();
          }
        }

        // For now, fail any constructing or different-realm cases.
        if (flags.isConstructing()) {
          TRACE_PRINTF("failing: constructing\n");
          FAIL_IC();
        }
        if (!flags.isSameRealm()) {
          TRACE_PRINTF("failing: not same realm\n");
          FAIL_IC();
        }
        // And support only "standard" arg formats.
        if (flags.getArgFormat() != CallFlags::Standard) {
          TRACE_PRINTF("failing: not standard arg format\n");
          FAIL_IC();
        }

        // For now, fail any arg-underflow case.
        if (argc < callee->nargs()) {
          TRACE_PRINTF("failing: too few args\n");
          FAIL_IC();
        }

        uint32_t extra = 1 + flags.isConstructing() + isNative;
        uint32_t totalArgs = argc + extra;
        StackVal* origArgs = sp;

        {
          PUSH_IC_FRAME();

          if (!ctx.stack.check(sp, sizeof(StackVal) * (totalArgs + 6))) {
            ReportOverRecursed(ctx.frameMgr.cxForLocalUseOnly());
            return PBIResult::Error;
          }

          // This will not be an Exit frame but a BaselineStub frame, so
          // replace the ExitFrameType with the ICStub pointer.
          POPNNATIVE(1);
          PUSHNATIVE(StackValNative(cstub));

          // Push args.
          for (uint32_t i = 0; i < totalArgs; i++) {
            PUSH(origArgs[i]);
          }
          Value* args = reinterpret_cast<Value*>(sp);

          TRACE_PRINTF("pushing callee: %p\n", callee);
          PUSHNATIVE(StackValNative(
              CalleeToToken(callee, /* isConstructing = */ false)));

          if (isNative) {
            PUSHNATIVE(StackValNative(argc));
            PUSHNATIVE(StackValNative(
                MakeFrameDescriptorForJitCall(FrameType::BaselineStub, 0)));

            // We *also* need an exit frame (the native baseline
            // execution would invoke a trampoline here).
            StackVal* trampolinePrevFP = ctx.stack.fp;
            PUSHNATIVE(StackValNative(nullptr));  // fake return address.
            PUSHNATIVE(StackValNative(ctx.stack.fp));
            ctx.stack.fp = sp;
            PUSHNATIVE(StackValNative(uint32_t(ExitFrameType::CallNative)));
            cx.getCx()->activation()->asJit()->setJSExitFP(
                reinterpret_cast<uint8_t*>(ctx.stack.fp));
            cx.getCx()->portableBaselineStack().top =
                reinterpret_cast<void*>(sp);

            JSNative native = ignoresRv
                                  ? callee->jitInfo()->ignoresReturnValueMethod
                                  : callee->native();
            bool success = native(cx, argc, args);

            ctx.stack.fp = trampolinePrevFP;
            POPNNATIVE(4);

            if (!success) {
              return PBIResult::Error;
            }
            *ret = args[0].asRawBits();
          } else {
            PUSHNATIVE(StackValNative(
                MakeFrameDescriptorForJitCall(FrameType::BaselineStub, argc)));

            JSScript* script = callee->nonLazyScript();
            jsbytecode* pc = script->code();
            ImmutableScriptData* isd = script->immutableScriptData();
            PBIResult result;
            INVOKE_PBI(result, script,
                       (PortableBaselineInterpret<false, true, kHybridICs>), cx,
                       ctx.state, ctx.stack, sp, /* envChain = */ nullptr,
                       reinterpret_cast<Value*>(&*ret), pc, isd, nullptr,
                       nullptr, nullptr, PBIResult::Ok);
            if (result != PBIResult::Ok) {
              return result;
            }
          }
        }

        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(MetaScriptedThisShape) {
        uint32_t thisShapeOffset = cacheIRReader.stubOffset();
        // This op is only metadata for the Warp Transpiler and should be
        // ignored.
        (void)thisShapeOffset;
        PREDICT_NEXT(CallScriptedFunction);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadFixedSlotResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        uintptr_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        Value* slot = reinterpret_cast<Value*>(
            reinterpret_cast<uintptr_t>(nobj) + offset);
        TRACE_PRINTF(
            "LoadFixedSlotResult: obj %p offsetOffset %d offset %d slotPtr %p "
            "slot %" PRIx64 "\n",
            nobj, int(offsetOffset), int(offset), slot, slot->asRawBits());
        *ret = slot->asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadDynamicSlotResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t offsetOffset = cacheIRReader.stubOffset();
        uintptr_t offset =
            cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        HeapSlot* slots = nobj->getSlotsUnchecked();
        *ret = slots[offset / sizeof(Value)].get().asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadDenseElementResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        NativeObject* nobj =
            reinterpret_cast<NativeObject*>(READ_REG(objId.id()));
        ObjectElements* elems = nobj->getElementsHeader();
        int32_t index = int32_t(READ_REG(indexId.id()));
        if (index < 0 || uint32_t(index) >= nobj->getDenseInitializedLength()) {
          FAIL_IC();
        }
        HeapSlot* slot = &elems->elements()[index];
        Value val = slot->get();
        if (val.isMagic()) {
          FAIL_IC();
        }
        *ret = val.asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadInt32ArrayLengthResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        ArrayObject* aobj =
            reinterpret_cast<ArrayObject*>(READ_REG(objId.id()));
        uint32_t length = aobj->length();
        if (length > uint32_t(INT32_MAX)) {
          FAIL_IC();
        }
        *ret = Int32Value(length).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadInt32ArrayLength) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        ArrayObject* aobj =
            reinterpret_cast<ArrayObject*>(READ_REG(objId.id()));
        uint32_t length = aobj->length();
        if (length > uint32_t(INT32_MAX)) {
          FAIL_IC();
        }
        WRITE_REG(resultId.id(), length);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadArgumentsObjectArgResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        uint32_t index = uint32_t(READ_REG(indexId.id()));
        ArgumentsObject* args = &obj->as<ArgumentsObject>();
        if (index >= args->initialLength() || args->hasOverriddenElement()) {
          FAIL_IC();
        }
        if (args->argIsForwarded(index)) {
          FAIL_IC();
        }
        *ret = args->arg(index).asRawBits();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LinearizeForCharAccess) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        StringOperandId resultId = cacheIRReader.stringOperandId();
        BOUNDSCHECK(resultId);
        JSString* str = reinterpret_cast<JSLinearString*>(READ_REG(strId.id()));
        (void)indexId;

        if (!str->isRope()) {
          WRITE_REG(resultId.id(), reinterpret_cast<uintptr_t>(str));
        } else {
          PUSH_IC_FRAME();
          JSLinearString* result = LinearizeForCharAccess(cx, str);
          if (!result) {
            return PBIResult::Error;
          }
          WRITE_REG(resultId.id(), reinterpret_cast<uintptr_t>(result));
        }
        PREDICT_NEXT(LoadStringCharResult);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadStringCharResult) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        bool handleOOB = cacheIRReader.readBool();

        JSString* str = reinterpret_cast<JSLinearString*>(READ_REG(strId.id()));
        int32_t index = int32_t(READ_REG(indexId.id()));
        JSString* result = nullptr;
        if (index < 0 || size_t(index) >= str->length()) {
          if (handleOOB) {
            // Return an empty string.
            result = ctx.frameMgr.cxForLocalUseOnly()->names().empty_;
          } else {
            FAIL_IC();
          }
        } else {
          char16_t c;
          // Guaranteed to be always work because this CacheIR op is
          // always preceded by LinearizeForCharAccess.
          MOZ_ALWAYS_TRUE(str->getChar(/* cx = */ nullptr, index, &c));
          StaticStrings& sstr =
              ctx.frameMgr.cxForLocalUseOnly()->staticStrings();
          if (sstr.hasUnit(c)) {
            result = sstr.getUnit(c);
          } else {
            PUSH_IC_FRAME();
            result = StringFromCharCode(cx, c);
            if (!result) {
              return PBIResult::Error;
            }
          }
        }
        *ret = StringValue(result).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadStringCharCodeResult) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        Int32OperandId indexId = cacheIRReader.int32OperandId();
        bool handleOOB = cacheIRReader.readBool();

        JSString* str = reinterpret_cast<JSLinearString*>(READ_REG(strId.id()));
        int32_t index = int32_t(READ_REG(indexId.id()));
        Value result;
        if (index < 0 || size_t(index) >= str->length()) {
          if (handleOOB) {
            // Return NaN.
            result = JS::NaNValue();
          } else {
            FAIL_IC();
          }
        } else {
          char16_t c;
          // Guaranteed to be always work because this CacheIR op is
          // always preceded by LinearizeForCharAccess.
          MOZ_ALWAYS_TRUE(str->getChar(/* cx = */ nullptr, index, &c));
          result = Int32Value(c);
        }
        *ret = result.asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadStringLengthResult) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        JSString* str = reinterpret_cast<JSString*>(READ_REG(strId.id()));
        size_t length = str->length();
        if (length > size_t(INT32_MAX)) {
          FAIL_IC();
        }
        *ret = Int32Value(length).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadObjectResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        *ret = ObjectValue(*reinterpret_cast<JSObject*>(READ_REG(objId.id())))
                   .asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadStringResult) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        *ret = StringValue(reinterpret_cast<JSString*>(READ_REG(strId.id())))
                   .asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadSymbolResult) {
        SymbolOperandId symId = cacheIRReader.symbolOperandId();
        *ret = SymbolValue(reinterpret_cast<JS::Symbol*>(READ_REG(symId.id())))
                   .asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadInt32Result) {
        Int32OperandId valId = cacheIRReader.int32OperandId();
        *ret = Int32Value(READ_REG(valId.id())).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadDoubleResult) {
        NumberOperandId valId = cacheIRReader.numberOperandId();
        Value val = Value::fromRawBits(READ_REG(valId.id()));
        if (val.isInt32()) {
          val = DoubleValue(val.toInt32());
        }
        *ret = val.asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadBigIntResult) {
        BigIntOperandId valId = cacheIRReader.bigIntOperandId();
        *ret = BigIntValue(reinterpret_cast<JS::BigInt*>(READ_REG(valId.id())))
                   .asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadBooleanResult) {
        bool val = cacheIRReader.readBool();
        *ret = BooleanValue(val).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadInt32Constant) {
        uint32_t valOffset = cacheIRReader.stubOffset();
        Int32OperandId resultId = cacheIRReader.int32OperandId();
        BOUNDSCHECK(resultId);
        uint32_t value = cstub->stubInfo()->getStubRawInt32(cstub, valOffset);
        WRITE_REG(resultId.id(), value);
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadConstantStringResult) {
        uint32_t strOffset = cacheIRReader.stubOffset();
        JSString* str = reinterpret_cast<JSString*>(
            cstub->stubInfo()->getStubRawWord(cstub, strOffset));
        *ret = StringValue(str).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

#define INT32_OP(name, op, extra_check)                    \
  CACHEOP_CASE(Int32##name##Result) {                      \
    Int32OperandId lhsId = cacheIRReader.int32OperandId(); \
    Int32OperandId rhsId = cacheIRReader.int32OperandId(); \
    int64_t lhs = int64_t(int32_t(READ_REG(lhsId.id())));  \
    int64_t rhs = int64_t(int32_t(READ_REG(rhsId.id())));  \
    extra_check;                                           \
    int64_t result = lhs op rhs;                           \
    if (result < INT32_MIN || result > INT32_MAX) {        \
      FAIL_IC();                                           \
    }                                                      \
    *ret = Int32Value(int32_t(result)).asRawBits();        \
    PREDICT_RETURN();                                      \
    DISPATCH_CACHEOP();                                    \
  }

      // clang-format off
  INT32_OP(Add, +, {});
  INT32_OP(Sub, -, {});
      // clang-format on
      INT32_OP(Mul, *, {
        if (rhs * lhs == 0 && ((rhs < 0) ^ (lhs < 0))) {
          FAIL_IC();
        }
      });
      INT32_OP(Div, /, {
        if (rhs == 0 || (lhs == INT32_MIN && rhs == -1)) {
          FAIL_IC();
        }
        if (lhs == 0 && rhs < 0) {
          FAIL_IC();
        }
        if (lhs % rhs != 0) {
          FAIL_IC();
        }
      });
      INT32_OP(Mod, %, {
        if (rhs == 0 || (lhs == INT32_MIN && rhs == -1)) {
          FAIL_IC();
        }
        if (lhs % rhs == 0 && lhs < 0) {
          FAIL_IC();
        }
      });
      // clang-format off
  INT32_OP(BitOr, |, {});
  INT32_OP(BitXor, ^, {});
  INT32_OP(BitAnd, &, {});
      // clang-format on

      CACHEOP_CASE(Int32PowResult) {
        Int32OperandId lhsId = cacheIRReader.int32OperandId();
        Int32OperandId rhsId = cacheIRReader.int32OperandId();
        int64_t lhs = int64_t(int32_t(READ_REG(lhsId.id())));
        int64_t rhs = int64_t(int32_t(READ_REG(rhsId.id())));
        int64_t result;

        if (lhs == 1) {
          result = 1;
        } else if (rhs < 0) {
          FAIL_IC();
        } else {
          result = 1;
          int64_t runningSquare = lhs;
          while (rhs) {
            if (rhs & 1) {
              result *= runningSquare;
              if (result > int64_t(INT32_MAX)) {
                FAIL_IC();
              }
            }
            rhs >>= 1;
            if (rhs == 0) {
              break;
            }
            runningSquare *= runningSquare;
            if (runningSquare > int64_t(INT32_MAX)) {
              FAIL_IC();
            }
          }
        }

        *ret = Int32Value(int32_t(result)).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(Int32IncResult) {
        Int32OperandId inputId = cacheIRReader.int32OperandId();
        int64_t value = int64_t(int32_t(READ_REG(inputId.id())));
        value++;
        if (value > INT32_MAX) {
          FAIL_IC();
        }
        *ret = Int32Value(int32_t(value)).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadInt32TruthyResult) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        int32_t val = int32_t(READ_REG(inputId.id()));
        *ret = BooleanValue(val != 0).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadStringTruthyResult) {
        StringOperandId strId = cacheIRReader.stringOperandId();
        JSString* str = reinterpret_cast<JSLinearString*>(READ_REG(strId.id()));
        *ret = BooleanValue(str->length() > 0).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadObjectTruthyResult) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        JSObject* obj = reinterpret_cast<JSObject*>(READ_REG(objId.id()));
        const JSClass* cls = obj->getClass();
        if (cls->isProxyObject()) {
          FAIL_IC();
        }
        *ret = BooleanValue(!cls->emulatesUndefined()).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadValueResult) {
        uint32_t valOffset = cacheIRReader.stubOffset();
        *ret = cstub->stubInfo()->getStubRawInt64(cstub, valOffset);
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(LoadOperandResult) {
        ValOperandId inputId = cacheIRReader.valOperandId();
        *ret = READ_REG(inputId.id());
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CallStringConcatResult) {
        StringOperandId lhsId = cacheIRReader.stringOperandId();
        StringOperandId rhsId = cacheIRReader.stringOperandId();
        // We don't push a frame and do a CanGC invocation here; we do a
        // pure (NoGC) invocation only, because it's cheaper.
        FakeRooted<JSString*> lhs(
            nullptr, reinterpret_cast<JSString*>(READ_REG(lhsId.id())));
        FakeRooted<JSString*> rhs(
            nullptr, reinterpret_cast<JSString*>(READ_REG(rhsId.id())));
        JSString* result =
            ConcatStrings<NoGC>(ctx.frameMgr.cxForLocalUseOnly(), lhs, rhs);
        if (result) {
          *ret = StringValue(result).asRawBits();
        } else {
          FAIL_IC();
        }
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CompareStringResult) {
        JSOp op = cacheIRReader.jsop();
        StringOperandId lhsId = cacheIRReader.stringOperandId();
        StringOperandId rhsId = cacheIRReader.stringOperandId();
        {
          PUSH_IC_FRAME();
          ReservedRooted<JSString*> lhs(
              &ctx.state.str0,
              reinterpret_cast<JSString*>(READ_REG(lhsId.id())));
          ReservedRooted<JSString*> rhs(
              &ctx.state.str1,
              reinterpret_cast<JSString*>(READ_REG(rhsId.id())));
          bool result;
          if (lhs == rhs) {
            // If operands point to the same instance, the strings are trivially
            // equal.
            result = op == JSOp::Eq || op == JSOp::StrictEq || op == JSOp::Le ||
                     op == JSOp::Ge;
          } else {
            switch (op) {
              case JSOp::Eq:
              case JSOp::StrictEq:
                if (lhs->isAtom() && rhs->isAtom()) {
                  result = false;
                  break;
                }
                if (lhs->length() != rhs->length()) {
                  result = false;
                  break;
                }
                if (!StringsEqual<EqualityKind::Equal>(cx, lhs, rhs, &result)) {
                  return PBIResult::Error;
                }
                break;
              case JSOp::Ne:
              case JSOp::StrictNe:
                if (lhs->isAtom() && rhs->isAtom()) {
                  result = true;
                  break;
                }
                if (lhs->length() != rhs->length()) {
                  result = true;
                  break;
                }
                if (!StringsEqual<EqualityKind::NotEqual>(cx, lhs, rhs,
                                                          &result)) {
                  return PBIResult::Error;
                }
                break;
              case JSOp::Lt:
                if (!StringsCompare<ComparisonKind::LessThan>(cx, lhs, rhs,
                                                              &result)) {
                  return PBIResult::Error;
                }
                break;
              case JSOp::Ge:
                if (!StringsCompare<ComparisonKind::GreaterThanOrEqual>(
                        cx, lhs, rhs, &result)) {
                  return PBIResult::Error;
                }
                break;
              case JSOp::Le:
                if (!StringsCompare<ComparisonKind::GreaterThanOrEqual>(
                        cx, /* N.B. swapped order */ rhs, lhs, &result)) {
                  return PBIResult::Error;
                }
                break;
              case JSOp::Gt:
                if (!StringsCompare<ComparisonKind::LessThan>(
                        cx, /* N.B. swapped order */ rhs, lhs, &result)) {
                  return PBIResult::Error;
                }
                break;
              default:
                MOZ_CRASH("bad opcode");
            }
          }
          *ret = BooleanValue(result).asRawBits();
        }
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CompareInt32Result) {
        JSOp op = cacheIRReader.jsop();
        Int32OperandId lhsId = cacheIRReader.int32OperandId();
        Int32OperandId rhsId = cacheIRReader.int32OperandId();
        int64_t lhs = int64_t(int32_t(READ_REG(lhsId.id())));
        int64_t rhs = int64_t(int32_t(READ_REG(rhsId.id())));
        TRACE_PRINTF("lhs (%d) = %" PRIi64 " rhs (%d) = %" PRIi64 "\n",
                     lhsId.id(), lhs, rhsId.id(), rhs);
        bool result;
        switch (op) {
          case JSOp::Eq:
          case JSOp::StrictEq:
            result = lhs == rhs;
            break;
          case JSOp::Ne:
          case JSOp::StrictNe:
            result = lhs != rhs;
            break;
          case JSOp::Lt:
            result = lhs < rhs;
            break;
          case JSOp::Le:
            result = lhs <= rhs;
            break;
          case JSOp::Gt:
            result = lhs > rhs;
            break;
          case JSOp::Ge:
            result = lhs >= rhs;
            break;
          default:
            MOZ_CRASH("Unexpected opcode");
        }
        *ret = BooleanValue(result).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(CompareNullUndefinedResult) {
        JSOp op = cacheIRReader.jsop();
        bool isUndefined = cacheIRReader.readBool();
        ValOperandId inputId = cacheIRReader.valOperandId();
        Value val = Value::fromRawBits(READ_REG(inputId.id()));
        if (val.isObject() && val.toObject().getClass()->isProxyObject()) {
          FAIL_IC();
        }

        bool result;
        switch (op) {
          case JSOp::Eq:
            result = val.isUndefined() || val.isNull() ||
                     (val.isObject() &&
                      val.toObject().getClass()->emulatesUndefined());
            break;
          case JSOp::Ne:
            result = !(val.isUndefined() || val.isNull() ||
                       (val.isObject() &&
                        val.toObject().getClass()->emulatesUndefined()));
            break;
          case JSOp::StrictEq:
            result = isUndefined ? val.isUndefined() : val.isNull();
            break;
          case JSOp::StrictNe:
            result = !(isUndefined ? val.isUndefined() : val.isNull());
            break;
          default:
            MOZ_CRASH("bad opcode");
        }
        *ret = BooleanValue(result).asRawBits();
        PREDICT_RETURN();
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE(AssertPropertyLookup) {
        ObjOperandId objId = cacheIRReader.objOperandId();
        uint32_t idOffset = cacheIRReader.stubOffset();
        uint32_t slotOffset = cacheIRReader.stubOffset();
        // Debug-only assertion; we can ignore.
        (void)objId;
        (void)idOffset;
        (void)slotOffset;
        DISPATCH_CACHEOP();
      }

      CACHEOP_CASE_UNIMPL(GuardNumberToIntPtrIndex)
      CACHEOP_CASE_UNIMPL(GuardToUint8Clamped)
      CACHEOP_CASE_UNIMPL(GuardMultipleShapes)
      CACHEOP_CASE_UNIMPL(CallRegExpMatcherResult)
      CACHEOP_CASE_UNIMPL(CallRegExpSearcherResult)
      CACHEOP_CASE_UNIMPL(RegExpSearcherLastLimitResult)
      CACHEOP_CASE_UNIMPL(RegExpHasCaptureGroupsResult)
      CACHEOP_CASE_UNIMPL(RegExpBuiltinExecMatchResult)
      CACHEOP_CASE_UNIMPL(RegExpBuiltinExecTestResult)
      CACHEOP_CASE_UNIMPL(RegExpFlagResult)
      CACHEOP_CASE_UNIMPL(CallSubstringKernelResult)
      CACHEOP_CASE_UNIMPL(StringReplaceStringResult)
      CACHEOP_CASE_UNIMPL(StringSplitStringResult)
      CACHEOP_CASE_UNIMPL(RegExpPrototypeOptimizableResult)
      CACHEOP_CASE_UNIMPL(RegExpInstanceOptimizableResult)
      CACHEOP_CASE_UNIMPL(GetFirstDollarIndexResult)
      CACHEOP_CASE_UNIMPL(GuardIsFixedLengthTypedArray)
      CACHEOP_CASE_UNIMPL(StringToAtom)
      CACHEOP_CASE_UNIMPL(GuardIndexIsValidUpdateOrAdd)
      CACHEOP_CASE_UNIMPL(GuardIndexIsNotDenseElement)
      CACHEOP_CASE_UNIMPL(GuardTagNotEqual)
      CACHEOP_CASE_UNIMPL(GuardXrayExpandoShapeAndDefaultProto)
      CACHEOP_CASE_UNIMPL(GuardXrayNoExpando)
      CACHEOP_CASE_UNIMPL(LoadScriptedProxyHandler)
      CACHEOP_CASE_UNIMPL(IdToStringOrSymbol)
      CACHEOP_CASE_UNIMPL(DoubleToUint8Clamped)
      CACHEOP_CASE_UNIMPL(MegamorphicStoreSlot)
      CACHEOP_CASE_UNIMPL(MegamorphicHasPropResult)
      CACHEOP_CASE_UNIMPL(SmallObjectVariableKeyHasOwnResult)
      CACHEOP_CASE_UNIMPL(ObjectToIteratorResult)
      CACHEOP_CASE_UNIMPL(ValueToIteratorResult)
      CACHEOP_CASE_UNIMPL(LoadDOMExpandoValue)
      CACHEOP_CASE_UNIMPL(LoadDOMExpandoValueGuardGeneration)
      CACHEOP_CASE_UNIMPL(LoadDOMExpandoValueIgnoreGeneration)
      CACHEOP_CASE_UNIMPL(GuardDOMExpandoMissingOrGuardShape)
      CACHEOP_CASE_UNIMPL(AddSlotAndCallAddPropHook)
      CACHEOP_CASE_UNIMPL(ArrayJoinResult)
      CACHEOP_CASE_UNIMPL(ObjectKeysResult)
      CACHEOP_CASE_UNIMPL(PackedArrayPopResult)
      CACHEOP_CASE_UNIMPL(PackedArrayShiftResult)
      CACHEOP_CASE_UNIMPL(PackedArraySliceResult)
      CACHEOP_CASE_UNIMPL(ArgumentsSliceResult)
      CACHEOP_CASE_UNIMPL(IsArrayResult)
      CACHEOP_CASE_UNIMPL(StoreFixedSlotUndefinedResult)
      CACHEOP_CASE_UNIMPL(IsPackedArrayResult)
      CACHEOP_CASE_UNIMPL(IsCallableResult)
      CACHEOP_CASE_UNIMPL(IsConstructorResult)
      CACHEOP_CASE_UNIMPL(IsCrossRealmArrayConstructorResult)
      CACHEOP_CASE_UNIMPL(IsTypedArrayResult)
      CACHEOP_CASE_UNIMPL(IsTypedArrayConstructorResult)
      CACHEOP_CASE_UNIMPL(ArrayBufferViewByteOffsetInt32Result)
      CACHEOP_CASE_UNIMPL(ArrayBufferViewByteOffsetDoubleResult)
      CACHEOP_CASE_UNIMPL(TypedArrayByteLengthInt32Result)
      CACHEOP_CASE_UNIMPL(TypedArrayByteLengthDoubleResult)
      CACHEOP_CASE_UNIMPL(TypedArrayElementSizeResult)
      CACHEOP_CASE_UNIMPL(GuardHasAttachedArrayBuffer)
      CACHEOP_CASE_UNIMPL(NewArrayIteratorResult)
      CACHEOP_CASE_UNIMPL(NewStringIteratorResult)
      CACHEOP_CASE_UNIMPL(NewRegExpStringIteratorResult)
      CACHEOP_CASE_UNIMPL(ObjectCreateResult)
      CACHEOP_CASE_UNIMPL(NewArrayFromLengthResult)
      CACHEOP_CASE_UNIMPL(NewTypedArrayFromLengthResult)
      CACHEOP_CASE_UNIMPL(NewTypedArrayFromArrayBufferResult)
      CACHEOP_CASE_UNIMPL(NewTypedArrayFromArrayResult)
      CACHEOP_CASE_UNIMPL(NewStringObjectResult)
      CACHEOP_CASE_UNIMPL(StringFromCharCodeResult)
      CACHEOP_CASE_UNIMPL(StringFromCodePointResult)
      CACHEOP_CASE_UNIMPL(StringIncludesResult)
      CACHEOP_CASE_UNIMPL(StringIndexOfResult)
      CACHEOP_CASE_UNIMPL(StringLastIndexOfResult)
      CACHEOP_CASE_UNIMPL(StringStartsWithResult)
      CACHEOP_CASE_UNIMPL(StringEndsWithResult)
      CACHEOP_CASE_UNIMPL(StringToLowerCaseResult)
      CACHEOP_CASE_UNIMPL(StringToUpperCaseResult)
      CACHEOP_CASE_UNIMPL(StringTrimResult)
      CACHEOP_CASE_UNIMPL(StringTrimStartResult)
      CACHEOP_CASE_UNIMPL(StringTrimEndResult)
      CACHEOP_CASE_UNIMPL(LinearizeForCodePointAccess)
      CACHEOP_CASE_UNIMPL(LoadStringAtResult)
      CACHEOP_CASE_UNIMPL(LoadStringCodePointResult)
      CACHEOP_CASE_UNIMPL(ToRelativeStringIndex)
      CACHEOP_CASE_UNIMPL(MathAbsInt32Result)
      CACHEOP_CASE_UNIMPL(MathAbsNumberResult)
      CACHEOP_CASE_UNIMPL(MathClz32Result)
      CACHEOP_CASE_UNIMPL(MathSignInt32Result)
      CACHEOP_CASE_UNIMPL(MathSignNumberResult)
      CACHEOP_CASE_UNIMPL(MathSignNumberToInt32Result)
      CACHEOP_CASE_UNIMPL(MathImulResult)
      CACHEOP_CASE_UNIMPL(MathSqrtNumberResult)
      CACHEOP_CASE_UNIMPL(MathFRoundNumberResult)
      CACHEOP_CASE_UNIMPL(MathRandomResult)
      CACHEOP_CASE_UNIMPL(MathHypot2NumberResult)
      CACHEOP_CASE_UNIMPL(MathHypot3NumberResult)
      CACHEOP_CASE_UNIMPL(MathHypot4NumberResult)
      CACHEOP_CASE_UNIMPL(MathAtan2NumberResult)
      CACHEOP_CASE_UNIMPL(MathFloorNumberResult)
      CACHEOP_CASE_UNIMPL(MathCeilNumberResult)
      CACHEOP_CASE_UNIMPL(MathTruncNumberResult)
      CACHEOP_CASE_UNIMPL(MathFloorToInt32Result)
      CACHEOP_CASE_UNIMPL(MathCeilToInt32Result)
      CACHEOP_CASE_UNIMPL(MathTruncToInt32Result)
      CACHEOP_CASE_UNIMPL(MathRoundToInt32Result)
      CACHEOP_CASE_UNIMPL(NumberMinMax)
      CACHEOP_CASE_UNIMPL(Int32MinMaxArrayResult)
      CACHEOP_CASE_UNIMPL(NumberMinMaxArrayResult)
      CACHEOP_CASE_UNIMPL(MathFunctionNumberResult)
      CACHEOP_CASE_UNIMPL(NumberParseIntResult)
      CACHEOP_CASE_UNIMPL(DoubleParseIntResult)
      CACHEOP_CASE_UNIMPL(ObjectToStringResult)
      CACHEOP_CASE_UNIMPL(ReflectGetPrototypeOfResult)
      CACHEOP_CASE_UNIMPL(AtomicsCompareExchangeResult)
      CACHEOP_CASE_UNIMPL(AtomicsExchangeResult)
      CACHEOP_CASE_UNIMPL(AtomicsAddResult)
      CACHEOP_CASE_UNIMPL(AtomicsSubResult)
      CACHEOP_CASE_UNIMPL(AtomicsAndResult)
      CACHEOP_CASE_UNIMPL(AtomicsOrResult)
      CACHEOP_CASE_UNIMPL(AtomicsXorResult)
      CACHEOP_CASE_UNIMPL(AtomicsLoadResult)
      CACHEOP_CASE_UNIMPL(AtomicsStoreResult)
      CACHEOP_CASE_UNIMPL(AtomicsIsLockFreeResult)
      CACHEOP_CASE_UNIMPL(CallNativeSetter)
      CACHEOP_CASE_UNIMPL(CallScriptedSetter)
      CACHEOP_CASE_UNIMPL(CallInlinedSetter)
      CACHEOP_CASE_UNIMPL(CallDOMSetter)
      CACHEOP_CASE_UNIMPL(CallSetArrayLength)
      CACHEOP_CASE_UNIMPL(ProxySet)
      CACHEOP_CASE_UNIMPL(ProxySetByValue)
      CACHEOP_CASE_UNIMPL(CallAddOrUpdateSparseElementHelper)
      CACHEOP_CASE_UNIMPL(CallNumberToString)
      CACHEOP_CASE_UNIMPL(Int32ToStringWithBaseResult)
      CACHEOP_CASE_UNIMPL(BooleanToString)
      CACHEOP_CASE_UNIMPL(CallBoundScriptedFunction)
      CACHEOP_CASE_UNIMPL(CallWasmFunction)
      CACHEOP_CASE_UNIMPL(GuardWasmArg)
      CACHEOP_CASE_UNIMPL(CallDOMFunction)
      CACHEOP_CASE_UNIMPL(CallClassHook)
      CACHEOP_CASE_UNIMPL(CallInlinedFunction)
#ifdef JS_PUNBOX64
      CACHEOP_CASE_UNIMPL(CallScriptedProxyGetResult)
      CACHEOP_CASE_UNIMPL(CallScriptedProxyGetByValueResult)
#endif
      CACHEOP_CASE_UNIMPL(BindFunctionResult)
      CACHEOP_CASE_UNIMPL(SpecializedBindFunctionResult)
      CACHEOP_CASE_UNIMPL(LoadFixedSlotTypedResult)
      CACHEOP_CASE_UNIMPL(LoadDenseElementHoleResult)
      CACHEOP_CASE_UNIMPL(CallGetSparseElementResult)
      CACHEOP_CASE_UNIMPL(LoadDenseElementExistsResult)
      CACHEOP_CASE_UNIMPL(LoadTypedArrayElementExistsResult)
      CACHEOP_CASE_UNIMPL(LoadDenseElementHoleExistsResult)
      CACHEOP_CASE_UNIMPL(LoadTypedArrayElementResult)
      CACHEOP_CASE_UNIMPL(LoadDataViewValueResult)
      CACHEOP_CASE_UNIMPL(StoreDataViewValueResult)
      CACHEOP_CASE_UNIMPL(LoadArgumentsObjectArgHoleResult)
      CACHEOP_CASE_UNIMPL(LoadArgumentsObjectArgExistsResult)
      CACHEOP_CASE_UNIMPL(LoadArgumentsObjectLengthResult)
      CACHEOP_CASE_UNIMPL(LoadArgumentsObjectLength)
      CACHEOP_CASE_UNIMPL(LoadFunctionLengthResult)
      CACHEOP_CASE_UNIMPL(LoadFunctionNameResult)
      CACHEOP_CASE_UNIMPL(LoadBoundFunctionNumArgs)
      CACHEOP_CASE_UNIMPL(LoadBoundFunctionTarget)
      CACHEOP_CASE_UNIMPL(GuardBoundFunctionIsConstructor)
      CACHEOP_CASE_UNIMPL(LoadArrayBufferByteLengthInt32Result)
      CACHEOP_CASE_UNIMPL(LoadArrayBufferByteLengthDoubleResult)
      CACHEOP_CASE_UNIMPL(LoadArrayBufferViewLengthInt32Result)
      CACHEOP_CASE_UNIMPL(LoadArrayBufferViewLengthDoubleResult)
      CACHEOP_CASE_UNIMPL(FrameIsConstructingResult)
      CACHEOP_CASE_UNIMPL(CallScriptedGetterResult)
      CACHEOP_CASE_UNIMPL(CallInlinedGetterResult)
      CACHEOP_CASE_UNIMPL(CallNativeGetterResult)
      CACHEOP_CASE_UNIMPL(CallDOMGetterResult)
      CACHEOP_CASE_UNIMPL(ProxyGetResult)
      CACHEOP_CASE_UNIMPL(ProxyGetByValueResult)
      CACHEOP_CASE_UNIMPL(ProxyHasPropResult)
      CACHEOP_CASE_UNIMPL(CallObjectHasSparseElementResult)
      CACHEOP_CASE_UNIMPL(CallNativeGetElementResult)
      CACHEOP_CASE_UNIMPL(CallNativeGetElementSuperResult)
      CACHEOP_CASE_UNIMPL(GetNextMapSetEntryForIteratorResult)
      CACHEOP_CASE_UNIMPL(LoadUndefinedResult)
      CACHEOP_CASE_UNIMPL(LoadDoubleConstant)
      CACHEOP_CASE_UNIMPL(LoadBooleanConstant)
      CACHEOP_CASE_UNIMPL(LoadUndefined)
      CACHEOP_CASE_UNIMPL(LoadConstantString)
      CACHEOP_CASE_UNIMPL(LoadInstanceOfObjectResult)
      CACHEOP_CASE_UNIMPL(LoadTypeOfObjectResult)
      CACHEOP_CASE_UNIMPL(DoubleAddResult)
      CACHEOP_CASE_UNIMPL(DoubleSubResult)
      CACHEOP_CASE_UNIMPL(DoubleMulResult)
      CACHEOP_CASE_UNIMPL(DoubleDivResult)
      CACHEOP_CASE_UNIMPL(DoubleModResult)
      CACHEOP_CASE_UNIMPL(DoublePowResult)
      CACHEOP_CASE_UNIMPL(BigIntAddResult)
      CACHEOP_CASE_UNIMPL(BigIntSubResult)
      CACHEOP_CASE_UNIMPL(BigIntMulResult)
      CACHEOP_CASE_UNIMPL(BigIntDivResult)
      CACHEOP_CASE_UNIMPL(BigIntModResult)
      CACHEOP_CASE_UNIMPL(BigIntPowResult)
      CACHEOP_CASE_UNIMPL(Int32LeftShiftResult)
      CACHEOP_CASE_UNIMPL(Int32RightShiftResult)
      CACHEOP_CASE_UNIMPL(Int32URightShiftResult)
      CACHEOP_CASE_UNIMPL(Int32NotResult)
      CACHEOP_CASE_UNIMPL(BigIntBitOrResult)
      CACHEOP_CASE_UNIMPL(BigIntBitXorResult)
      CACHEOP_CASE_UNIMPL(BigIntBitAndResult)
      CACHEOP_CASE_UNIMPL(BigIntLeftShiftResult)
      CACHEOP_CASE_UNIMPL(BigIntRightShiftResult)
      CACHEOP_CASE_UNIMPL(BigIntNotResult)
      CACHEOP_CASE_UNIMPL(Int32NegationResult)
      CACHEOP_CASE_UNIMPL(DoubleNegationResult)
      CACHEOP_CASE_UNIMPL(BigIntNegationResult)
      CACHEOP_CASE_UNIMPL(Int32DecResult)
      CACHEOP_CASE_UNIMPL(DoubleIncResult)
      CACHEOP_CASE_UNIMPL(DoubleDecResult)
      CACHEOP_CASE_UNIMPL(BigIntIncResult)
      CACHEOP_CASE_UNIMPL(BigIntDecResult)
      CACHEOP_CASE_UNIMPL(LoadDoubleTruthyResult)
      CACHEOP_CASE_UNIMPL(LoadBigIntTruthyResult)
      CACHEOP_CASE_UNIMPL(LoadValueTruthyResult)
      CACHEOP_CASE_UNIMPL(NewPlainObjectResult)
      CACHEOP_CASE_UNIMPL(NewArrayObjectResult)
      CACHEOP_CASE_UNIMPL(CallStringObjectConcatResult)
      CACHEOP_CASE_UNIMPL(CallIsSuspendedGeneratorResult)
      CACHEOP_CASE_UNIMPL(CompareObjectResult)
      CACHEOP_CASE_UNIMPL(CompareSymbolResult)
      CACHEOP_CASE_UNIMPL(CompareDoubleResult)
      CACHEOP_CASE_UNIMPL(CompareBigIntResult)
      CACHEOP_CASE_UNIMPL(CompareBigIntInt32Result)
      CACHEOP_CASE_UNIMPL(CompareBigIntNumberResult)
      CACHEOP_CASE_UNIMPL(CompareBigIntStringResult)
      CACHEOP_CASE_UNIMPL(CompareDoubleSameValueResult)
      CACHEOP_CASE_UNIMPL(SameValueResult)
      CACHEOP_CASE_UNIMPL(IndirectTruncateInt32Result)
      CACHEOP_CASE_UNIMPL(BigIntAsIntNResult)
      CACHEOP_CASE_UNIMPL(BigIntAsUintNResult)
      CACHEOP_CASE_UNIMPL(SetHasResult)
      CACHEOP_CASE_UNIMPL(SetHasNonGCThingResult)
      CACHEOP_CASE_UNIMPL(SetHasStringResult)
      CACHEOP_CASE_UNIMPL(SetHasSymbolResult)
      CACHEOP_CASE_UNIMPL(SetHasBigIntResult)
      CACHEOP_CASE_UNIMPL(SetHasObjectResult)
      CACHEOP_CASE_UNIMPL(SetSizeResult)
      CACHEOP_CASE_UNIMPL(MapHasResult)
      CACHEOP_CASE_UNIMPL(MapHasNonGCThingResult)
      CACHEOP_CASE_UNIMPL(MapHasStringResult)
      CACHEOP_CASE_UNIMPL(MapHasSymbolResult)
      CACHEOP_CASE_UNIMPL(MapHasBigIntResult)
      CACHEOP_CASE_UNIMPL(MapHasObjectResult)
      CACHEOP_CASE_UNIMPL(MapGetResult)
      CACHEOP_CASE_UNIMPL(MapGetNonGCThingResult)
      CACHEOP_CASE_UNIMPL(MapGetStringResult)
      CACHEOP_CASE_UNIMPL(MapGetSymbolResult)
      CACHEOP_CASE_UNIMPL(MapGetBigIntResult)
      CACHEOP_CASE_UNIMPL(MapGetObjectResult)
      CACHEOP_CASE_UNIMPL(MapSizeResult)
      CACHEOP_CASE_UNIMPL(ArrayFromArgumentsObjectResult)
      CACHEOP_CASE_UNIMPL(CloseIterScriptedResult)
      CACHEOP_CASE_UNIMPL(CallPrintString)
      CACHEOP_CASE_UNIMPL(Breakpoint)
      CACHEOP_CASE_UNIMPL(WrapResult)
      CACHEOP_CASE_UNIMPL(Bailout)
      CACHEOP_CASE_UNIMPL(AssertRecoveredOnBailoutResult) {
        TRACE_PRINTF("unknown CacheOp: %s\n", CacheIROpNames[int(cacheop)]);
        FAIL_IC();
      }

#undef CACHEOP_CASE
#undef CACHEOP_CASE_FALLTHROUGH
#undef CACHEOP_CASE_UNIMPL
#undef DISPATCH_CACHEOP
#undef FAIL_IC
#undef BOUNDSCHECK
#undef PREDICT_NEXT
#undef PREDICT_RETURN

#ifdef __wasi__
      case CacheOp::NumOpcodes:
        MOZ_CRASH("Invalid CacheOp::NumOpcodes opcode");
#endif
    }
  }

next_ic:
  stub = stub->maybeNext();
  MOZ_ASSERT(stub);
  PBIResult result;
  CALL_IC(ctx, stub, result, arg0, arg1, arg2, ret);
  return result;
}

/*
 * -----------------------------------------------
 * IC callsite logic, and fallback stubs
 * -----------------------------------------------
 */

#define DEFINE_IC(kind, arity, fallback_body)                           \
  static PBIResult MOZ_NEVER_INLINE IC##kind##Fallback(                 \
      ICCtx& ctx, ICStub* stub, const CacheIRStubInfo* stubInfo,        \
      const uint8_t* code, uint64_t arg0, uint64_t arg1, uint64_t arg2, \
      uint64_t* ret) {                                                  \
    jsbytecode* pc = ctx.pc;                                            \
    StackVal* sp = ctx.sp;                                              \
    ICFallbackStub* fallback = stub->toFallbackStub();                  \
    fallback_body;                                                      \
    *ret = ctx.state.res.asRawBits();                                   \
    ctx.state.res = UndefinedValue();                                   \
    return PBIResult::Ok;                                               \
  error:                                                                \
    return PBIResult::Error;                                            \
  }

#define DEFINE_IC_ALIAS(kind, target)                                        \
  static PBIResult MOZ_NEVER_INLINE IC##kind##Fallback(                      \
      ICCtx& ctx, ICStub* stub, const CacheIRStubInfo* stubInfo,             \
      const uint8_t* code, uint64_t arg0, uint64_t arg1, uint64_t arg2,      \
      uint64_t* ret) {                                                       \
    return IC##target##Fallback(ctx, stub, stubInfo, code, arg0, arg1, arg2, \
                                ret);                                        \
  }

#define IC_LOAD_VAL(state_elem, index)                    \
  ReservedRooted<Value> state_elem(&ctx.state.state_elem, \
                                   Value::fromRawBits(arg##index))
#define IC_LOAD_OBJ(state_elem, index)  \
  ReservedRooted<JSObject*> state_elem( \
      &ctx.state.state_elem, reinterpret_cast<JSObject*>(arg##index))

DEFINE_IC(TypeOf, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoTypeOfFallback(cx, ctx.frame, fallback, value0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetName, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetNameFallback(cx, ctx.frame, fallback, obj0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(Call, 1, {
  uint32_t argc = uint32_t(arg0);
  uint32_t totalArgs =
      argc + ctx.icregs.extraArgs;  // this, callee, (constructing?), func args
  Value* args = reinterpret_cast<Value*>(&ctx.sp[0]);
  TRACE_PRINTF("Call fallback: argc %d totalArgs %d args %p\n", argc, totalArgs,
               args);
  // Reverse values on the stack.
  std::reverse(args, args + totalArgs);
  {
    PUSH_FALLBACK_IC_FRAME();
    if (!DoCallFallback(cx, ctx.frame, fallback, argc, args, &ctx.state.res)) {
      std::reverse(args, args + totalArgs);
      goto error;
    }
  }
});

DEFINE_IC_ALIAS(CallConstructing, Call);

DEFINE_IC(SpreadCall, 1, {
  uint32_t argc = uint32_t(arg0);
  uint32_t totalArgs =
      argc + ctx.icregs.extraArgs;  // this, callee, (constructing?), func args
  Value* args = reinterpret_cast<Value*>(&ctx.sp[0]);
  TRACE_PRINTF("Call fallback: argc %d totalArgs %d args %p\n", argc, totalArgs,
               args);
  // Reverse values on the stack.
  std::reverse(args, args + totalArgs);
  {
    PUSH_FALLBACK_IC_FRAME();
    if (!DoSpreadCallFallback(cx, ctx.frame, fallback, args, &ctx.state.res)) {
      std::reverse(args, args + totalArgs);
      goto error;
    }
  }
});

DEFINE_IC_ALIAS(SpreadCallConstructing, SpreadCall);

DEFINE_IC(UnaryArith, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoUnaryArithFallback(cx, ctx.frame, fallback, value0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(BinaryArith, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoBinaryArithFallback(cx, ctx.frame, fallback, value0, value1,
                             &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(ToBool, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoToBoolFallback(cx, ctx.frame, fallback, value0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(Compare, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCompareFallback(cx, ctx.frame, fallback, value0, value1,
                         &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(InstanceOf, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoInstanceOfFallback(cx, ctx.frame, fallback, value0, value1,
                            &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(In, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoInFallback(cx, ctx.frame, fallback, value0, value1, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(BindName, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoBindNameFallback(cx, ctx.frame, fallback, obj0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(SetProp, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoSetPropFallback(cx, ctx.frame, fallback, nullptr, value0, value1)) {
    goto error;
  }
});

DEFINE_IC(NewObject, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoNewObjectFallback(cx, ctx.frame, fallback, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetProp, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetPropFallback(cx, ctx.frame, fallback, &value0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetPropSuper, 2, {
  IC_LOAD_VAL(value0, 1);
  IC_LOAD_VAL(value1, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetPropSuperFallback(cx, ctx.frame, fallback, value0, &value1,
                              &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetElem, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetElemFallback(cx, ctx.frame, fallback, value0, value1,
                         &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetElemSuper, 3, {
  IC_LOAD_VAL(value0, 0);  // receiver
  IC_LOAD_VAL(value1, 1);  // obj (lhs)
  IC_LOAD_VAL(value2, 2);  // key (rhs)
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetElemSuperFallback(cx, ctx.frame, fallback, value1, value2, value0,
                              &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(NewArray, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoNewArrayFallback(cx, ctx.frame, fallback, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetIntrinsic, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetIntrinsicFallback(cx, ctx.frame, fallback, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(SetElem, 3, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  IC_LOAD_VAL(value2, 2);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoSetElemFallback(cx, ctx.frame, fallback, nullptr, value0, value1,
                         value2)) {
    goto error;
  }
});

DEFINE_IC(HasOwn, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoHasOwnFallback(cx, ctx.frame, fallback, value0, value1,
                        &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(CheckPrivateField, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCheckPrivateFieldFallback(cx, ctx.frame, fallback, value0, value1,
                                   &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(GetIterator, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetIteratorFallback(cx, ctx.frame, fallback, value0, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(ToPropertyKey, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoToPropertyKeyFallback(cx, ctx.frame, fallback, value0,
                               &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(OptimizeSpreadCall, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoOptimizeSpreadCallFallback(cx, ctx.frame, fallback, value0,
                                    &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(OptimizeGetIterator, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoOptimizeGetIteratorFallback(cx, ctx.frame, fallback, value0,
                                     &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(Rest, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoRestFallback(cx, ctx.frame, fallback, &ctx.state.res)) {
    goto error;
  }
});

DEFINE_IC(CloseIter, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCloseIterFallback(cx, ctx.frame, fallback, obj0)) {
    goto error;
  }
});

uint8_t* GetPortableFallbackStub(BaselineICFallbackKind kind) {
  switch (kind) {
#define _(ty)                      \
  case BaselineICFallbackKind::ty: \
    return reinterpret_cast<uint8_t*>(&IC##ty##Fallback);
    IC_BASELINE_FALLBACK_CODE_KIND_LIST(_)
#undef _
    case BaselineICFallbackKind::Count:
      MOZ_CRASH("Invalid kind");
  }
}

uint8_t* GetICInterpreter() {
  return reinterpret_cast<uint8_t*>(&ICInterpretOps<false>);
}

/*
 * -----------------------------------------------
 * Main JSOp interpreter
 * -----------------------------------------------
 */

static EnvironmentObject& getEnvironmentFromCoordinate(
    BaselineFrame* frame, EnvironmentCoordinate ec) {
  JSObject* env = frame->environmentChain();
  for (unsigned i = ec.hops(); i; i--) {
    if (env->is<EnvironmentObject>()) {
      env = &env->as<EnvironmentObject>().enclosingEnvironment();
    } else {
      MOZ_ASSERT(env->is<DebugEnvironmentProxy>());
      env = &env->as<DebugEnvironmentProxy>().enclosingEnvironment();
    }
  }
  return env->is<EnvironmentObject>()
             ? env->as<EnvironmentObject>()
             : env->as<DebugEnvironmentProxy>().environment();
}

#ifndef __wasi__
#  define DEBUG_CHECK()                                                   \
    if (frame->isDebuggee()) {                                            \
      TRACE_PRINTF(                                                       \
          "Debug check: frame is debuggee, checking for debug script\n"); \
      if (script->hasDebugScript()) {                                     \
        goto debug;                                                       \
      }                                                                   \
    }
#else
#  define DEBUG_CHECK()
#endif

#ifdef ENABLE_JS_PBL_WEVAL
#  define WEVAL_UPDATE_CONTEXT() \
    weval::update_context(reinterpret_cast<uint32_t>(pc));
#  define WEVAL_POP_CONTEXT() \
    weval::pop_context();
#else
#  define WEVAL_UPDATE_CONTEXT() ;
#  define WEVAL_POP_CONTEXT() ;
#endif

#define LABEL(op) (&&label_##op)
#if !defined(TRACE_INTERP) && !defined(__wasi__)
#  define CASE(op) label_##op:
#  define DISPATCH() \
    DEBUG_CHECK();   \
    goto* addresses[*pc]
#else
#  define CASE(op) label_##op : case JSOp::op:
#  define DISPATCH()        \
    DEBUG_CHECK();          \
    WEVAL_UPDATE_CONTEXT(); \
    goto dispatch
#endif

#define ADVANCE(delta) pc += (delta);
#define ADVANCE_AND_DISPATCH(delta) \
  ADVANCE(delta);                   \
  DISPATCH();

#define END_OP(op) ADVANCE_AND_DISPATCH(JSOpLength_##op);

#define IC_SET_ARG_FROM_STACK(index, stack_index) \
  ic_arg##index = sp[(stack_index)].asUInt64();
#define IC_POP_ARG(index) ic_arg##index = (*sp++).asUInt64();
#define IC_SET_VAL_ARG(index, expr) ic_arg##index = (expr).asRawBits();
#define IC_SET_OBJ_ARG(index, expr) \
  ic_arg##index = reinterpret_cast<uint64_t>(expr);
#define IC_ZERO_ARG(index) ic_arg##index = 0;
#define IC_PUSH_RESULT() PUSH(StackVal(ic_ret));

#if !defined(TRACE_INTERP)
#  define PREDICT_NEXT(op)       \
    if (JSOp(*pc) == JSOp::op) { \
      DEBUG_CHECK();             \
      goto label_##op;           \
    }
#else
#  define PREDICT_NEXT(op)
#endif

#ifndef __wasi__
#  define COUNT_COVERAGE_PC(PC)                        \
    if (script->hasScriptCounts()) {                   \
      PCCounts* counts = script->maybeGetPCCounts(PC); \
      MOZ_ASSERT(counts);                              \
      counts->numExec()++;                             \
    }
#  define COUNT_COVERAGE_MAIN()                                        \
    {                                                                  \
      jsbytecode* main = script->main();                               \
      if (!BytecodeIsJumpTarget(JSOp(*main))) COUNT_COVERAGE_PC(main); \
    }
#else
#  define COUNT_COVERAGE_PC(PC) ;
#  define COUNT_COVERAGE_MAIN() ;
#endif

#define NEXT_IC() frame->interpreterICEntry()++;

#define INVOKE_IC(kind)                                                      \
  ctx.pc = pc;                                                               \
  ctx.sp = sp;                                                               \
  CALL_IC(ctx, frame->interpreterICEntry()->firstStub(), ic_result, ic_arg0, \
          ic_arg1, ic_arg2, &ic_ret);                                        \
  if (ic_result != PBIResult::Ok) {                                          \
    WEVAL_POP_CONTEXT();                                                     \
    goto ic_fail;                                                            \
  }                                                                          \
  NEXT_IC();

template <bool IsRestart, bool InlineCalls, bool HybridICs>
PBIResult PortableBaselineInterpret(
    JSContext* cx_, State& state, Stack& stack, StackVal* sp,
    JSObject* envChain, Value* ret, jsbytecode* pc, ImmutableScriptData* isd,
    jsbytecode* restartEntryPC, BaselineFrame* restartFrame,
    StackVal* restartEntryFrame, PBIResult restartCode) {
#define RESTART(code)                                                          \
  if (!IsRestart) {                                                            \
    return PortableBaselineInterpret<true, true, HybridICs>(                   \
        ctx.frameMgr.cxForLocalUseOnly(), state, stack, sp, envChain, ret, pc, \
        isd, entryPC, frame, entryFrame, code);                                \
  }
#define GOTO_ERROR()           \
  do {                         \
    RESTART(PBIResult::Error); \
    goto error;                \
  } while (0)

#define OPCODE_LABEL(op, ...) LABEL(op),
#define TRAILING_LABEL(v) LABEL(default),

  static const void* const addresses[EnableInterruptsPseudoOpcode + 1] = {
      FOR_EACH_OPCODE(OPCODE_LABEL)
          FOR_EACH_TRAILING_UNUSED_OPCODE(TRAILING_LABEL)};

#undef OPCODE_LABEL
#undef TRAILING_LABEL

  BaselineFrame* frame = restartFrame;
  StackVal* entryFrame = restartEntryFrame;
  jsbytecode* entryPC = restartEntryPC;

  if (!IsRestart) {
    PUSHNATIVE(StackValNative(nullptr));  // Fake return address.
    frame = stack.pushFrame(sp, cx_, envChain);
    MOZ_ASSERT(frame);  // safety: stack margin.
    sp = reinterpret_cast<StackVal*>(frame);
    // Save the entry frame so that when unwinding, we know when to
    // return from this C++ frame.
    entryFrame = sp;
    // Save the entry PC so that we can compute offsets locally.
    entryPC = pc;
  }

  RootedScript script(cx_, frame->script());
  bool from_unwind = false;
  PBIResult ic_result = PBIResult::Ok;
  uint64_t ic_arg0 = 0, ic_arg1 = 0, ic_arg2 = 0, ic_ret = 0;
  ICCtx ctx(cx_, frame, state, stack);
  auto* icEntries = frame->icScript()->icEntries();

  if (IsRestart) {
    ic_result = restartCode;
    goto ic_fail;
  } else {
    AutoCheckRecursionLimit recursion(ctx.frameMgr.cxForLocalUseOnly());
    if (!recursion.checkDontReport(ctx.frameMgr.cxForLocalUseOnly())) {
      PUSH_EXIT_FRAME();
      ReportOverRecursed(ctx.frameMgr.cxForLocalUseOnly());
      return PBIResult::Error;
    }
  }

  // Check max stack depth once, so we don't need to check it
  // otherwise below for ordinary stack-manipulation opcodes (just for
  // exit frames).
  if (!ctx.stack.check(sp, sizeof(StackVal) * script->nslots())) {
    PUSH_EXIT_FRAME();
    ReportOverRecursed(ctx.frameMgr.cxForLocalUseOnly());
    return PBIResult::Error;
  }

  for (uint32_t i = 0; i < script->nfixed(); i++) {
    PUSH(StackVal(UndefinedValue()));
  }
  ret->setUndefined();

  // Check if we are being debugged, and set a flag in the frame if so. This
  // flag must be set before calling InitFunctionEnvironmentObjects.
  if (script->isDebuggee()) {
    TRACE_PRINTF("Script is debuggee\n");
    frame->setIsDebuggee();
  }

  if (CalleeTokenIsFunction(frame->calleeToken())) {
    JSFunction* func = CalleeTokenToFunction(frame->calleeToken());
    frame->setEnvironmentChain(func->environment());
    if (func->needsFunctionEnvironmentObjects()) {
      PUSH_EXIT_FRAME();
      if (!js::InitFunctionEnvironmentObjects(cx, frame)) {
        GOTO_ERROR();
      }
      TRACE_PRINTF("callee is func %p; created environment object: %p\n", func,
                   frame->environmentChain());
    }
  }

  // The debug prologue can't run until the function environment is set up.
  if (script->isDebuggee()) {
    PUSH_EXIT_FRAME();
    if (!DebugPrologue(cx, frame)) {
      GOTO_ERROR();
    }
  }

  if (!script->hasScriptCounts()) {
    if (ctx.frameMgr.cxForLocalUseOnly()->realm()->collectCoverageForDebug()) {
      PUSH_EXIT_FRAME();
      if (!script->initScriptCounts(cx)) {
        GOTO_ERROR();
      }
    }
  }
  COUNT_COVERAGE_MAIN();

#ifndef __wasi__
  if (ctx.frameMgr.cxForLocalUseOnly()->hasAnyPendingInterrupt()) {
    PUSH_EXIT_FRAME();
    if (!InterruptCheck(cx)) {
      GOTO_ERROR();
    }
  }
#endif

  TRACE_PRINTF("Entering: sp = %p fp = %p frame = %p, script = %p, pc = %p\n",
               sp, ctx.stack.fp, frame, script.get(), pc);
  TRACE_PRINTF("nslots = %d nfixed = %d\n", int(script->nslots()),
               int(script->nfixed()));

#ifdef ENABLE_JS_PBL_WEVAL
  weval::push_context(reinterpret_cast<uint32_t>(pc));
#endif

  while (true) {
    DEBUG_CHECK();

  dispatch:
#ifdef TRACE_INTERP
  {
    JSOp op = JSOp(*pc);
    printf("sp[0] = %" PRIx64 " sp[1] = %" PRIx64 " sp[2] = %" PRIx64 "\n",
           sp[0].asUInt64(), sp[1].asUInt64(), sp[2].asUInt64());
    printf("script = %p pc = %p: %s (ic %d) pending = %d\n", script.get(), pc,
           CodeName(op),
           (int)(frame->interpreterICEntry() -
                 script->jitScript()->icScript()->icEntries()),
           ctx.frameMgr.cxForLocalUseOnly()->isExceptionPending());
    printf("sp = %p fp = %p\n", sp, ctx.stack.fp);
    printf("TOS tag: %d\n", int(sp[0].asValue().asRawBits() >> 47));
    fflush(stdout);
  }
#endif

#if !defined(__wasi__) && !defined(TRACE_INTERP)
    goto* addresses[*pc];
#else
    (void)addresses;  // Avoid unused-local error. We keep the table
                      // itself to avoid warnings (see note in IC
                      // interpreter above).
    switch (JSOp(*pc))
#endif
    {
      CASE(Nop) { END_OP(Nop); }
      CASE(NopIsAssignOp) { END_OP(NopIsAssignOp); }
      CASE(Undefined) {
        PUSH(StackVal(UndefinedValue()));
        END_OP(Undefined);
      }
      CASE(Null) {
        PUSH(StackVal(NullValue()));
        END_OP(Null);
      }
      CASE(False) {
        PUSH(StackVal(BooleanValue(false)));
        END_OP(False);
      }
      CASE(True) {
        PUSH(StackVal(BooleanValue(true)));
        END_OP(True);
      }
      CASE(Int32) {
        PUSH(StackVal(Int32Value(GET_INT32(pc))));
        END_OP(Int32);
      }
      CASE(Zero) {
        PUSH(StackVal(Int32Value(0)));
        END_OP(Zero);
      }
      CASE(One) {
        PUSH(StackVal(Int32Value(1)));
        END_OP(One);
      }
      CASE(Int8) {
        PUSH(StackVal(Int32Value(GET_INT8(pc))));
        END_OP(Int8);
      }
      CASE(Uint16) {
        PUSH(StackVal(Int32Value(GET_UINT16(pc))));
        END_OP(Uint16);
      }
      CASE(Uint24) {
        PUSH(StackVal(Int32Value(GET_UINT24(pc))));
        END_OP(Uint24);
      }
      CASE(Double) {
        PUSH(StackVal(GET_INLINE_VALUE(pc)));
        END_OP(Double);
      }
      CASE(BigInt) {
        PUSH(StackVal(JS::BigIntValue(script->getBigInt(pc))));
        END_OP(BigInt);
      }
      CASE(String) {
        PUSH(StackVal(StringValue(script->getString(pc))));
        END_OP(String);
      }
      CASE(Symbol) {
        PUSH(StackVal(SymbolValue(
            ctx.frameMgr.cxForLocalUseOnly()->wellKnownSymbols().get(
                GET_UINT8(pc)))));
        END_OP(Symbol);
      }
      CASE(Void) {
        sp[0] = StackVal(JS::UndefinedValue());
        END_OP(Void);
      }

      CASE(Typeof)
      CASE(TypeofExpr) {
        static_assert(JSOpLength_Typeof == JSOpLength_TypeofExpr);
        if (HybridICs) {
          sp[0] = StackVal(StringValue(TypeOfOperation(
              Stack::handle(sp), ctx.frameMgr.cxForLocalUseOnly()->runtime())));
          NEXT_IC();
        } else {
          IC_POP_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(Typeof);
          IC_PUSH_RESULT();
        }
        END_OP(Typeof);
      }

      CASE(Pos) {
        if (sp[0].asValue().isNumber()) {
          // Nothing!
          NEXT_IC();
          END_OP(Pos);
        } else {
          goto generic_unary;
        }
      }
      CASE(Neg) {
        if (sp[0].asValue().isInt32()) {
          int32_t i = sp[0].asValue().toInt32();
          if (i != 0 && i != INT32_MIN) {
            sp[0] = StackVal(Int32Value(-i));
            NEXT_IC();
            END_OP(Neg);
          }
        }
        if (sp[0].asValue().isNumber()) {
          sp[0] = StackVal(NumberValue(-sp[0].asValue().toNumber()));
          NEXT_IC();
          END_OP(Neg);
        }
        goto generic_unary;
      }

      CASE(Inc) {
        if (sp[0].asValue().isInt32()) {
          int32_t i = sp[0].asValue().toInt32();
          if (i != INT32_MAX) {
            sp[0] = StackVal(Int32Value(i + 1));
            NEXT_IC();
            END_OP(Inc);
          }
        }
        if (sp[0].asValue().isNumber()) {
          sp[0] = StackVal(NumberValue(sp[0].asValue().toNumber() + 1));
          NEXT_IC();
          END_OP(Inc);
        }
        goto generic_unary;
      }
      CASE(Dec) {
        if (sp[0].asValue().isInt32()) {
          int32_t i = sp[0].asValue().toInt32();
          if (i != INT32_MIN) {
            sp[0] = StackVal(Int32Value(i - 1));
            NEXT_IC();
            END_OP(Dec);
          }
        }
        if (sp[0].asValue().isNumber()) {
          sp[0] = StackVal(NumberValue(sp[0].asValue().toNumber() - 1));
          NEXT_IC();
          END_OP(Dec);
        }
        goto generic_unary;
      }

      CASE(BitNot) {
        if (sp[0].asValue().isInt32()) {
          int32_t i = sp[0].asValue().toInt32();
          sp[0] = StackVal(Int32Value(~i));
          NEXT_IC();
          END_OP(Inc);
        }
        goto generic_unary;
      }

      CASE(ToNumeric) {
        if (HybridICs) {
          if (sp[0].asValue().isNumeric()) {
            NEXT_IC();
          } else {
            MutableHandleValue val = Stack::handleMut(&sp[0]);
            PUSH_EXIT_FRAME();
            if (!ToNumeric(cx, val)) {
              GOTO_ERROR();
            }
            NEXT_IC();
          }
        } else {
          goto generic_unary;
        }
        END_OP(ToNumeric);
      }

    generic_unary: {
      static_assert(JSOpLength_Pos == JSOpLength_Neg);
      static_assert(JSOpLength_Pos == JSOpLength_BitNot);
      static_assert(JSOpLength_Pos == JSOpLength_Inc);
      static_assert(JSOpLength_Pos == JSOpLength_Dec);
      static_assert(JSOpLength_Pos == JSOpLength_ToNumeric);
      IC_POP_ARG(0);
      IC_ZERO_ARG(1);
      IC_ZERO_ARG(2);
      INVOKE_IC(UnaryArith);
      IC_PUSH_RESULT();
      END_OP(Pos);
    }

      CASE(Not) {
        if (HybridICs) {
          sp[0] = StackVal(BooleanValue(!ToBoolean(Stack::handle(sp))));
          NEXT_IC();
        } else {
          IC_POP_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(ToBool);
          PUSH(StackVal(BooleanValue(!Value::fromRawBits(ic_ret).toBoolean())));
        }
        END_OP(Not);
      }

      CASE(And) {
        bool result;
        if (HybridICs) {
          result = ToBoolean(Stack::handle(sp));
          NEXT_IC();
        } else {
          IC_SET_ARG_FROM_STACK(0, 0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(ToBool);
          result = Value::fromRawBits(ic_ret).toBoolean();
        }
        int32_t jumpOffset = GET_JUMP_OFFSET(pc);
        if (!result) {
          ADVANCE(jumpOffset);
          PREDICT_NEXT(JumpTarget);
          PREDICT_NEXT(LoopHead);
        } else {
          ADVANCE(JSOpLength_And);
        }
        DISPATCH();
      }
      CASE(Or) {
        bool result;
        if (HybridICs) {
          result = ToBoolean(Stack::handle(sp));
          NEXT_IC();
        } else {
          IC_SET_ARG_FROM_STACK(0, 0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(ToBool);
          result = Value::fromRawBits(ic_ret).toBoolean();
        }
        int32_t jumpOffset = GET_JUMP_OFFSET(pc);
        if (result) {
          ADVANCE(jumpOffset);
          PREDICT_NEXT(JumpTarget);
          PREDICT_NEXT(LoopHead);
        } else {
          ADVANCE(JSOpLength_Or);
        }
        DISPATCH();
      }
      CASE(JumpIfTrue) {
        bool result;
        if (HybridICs) {
          result = ToBoolean(Stack::handle(sp));
          POP();
          NEXT_IC();
        } else {
          IC_POP_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(ToBool);
          result = Value::fromRawBits(ic_ret).toBoolean();
        }
        int32_t jumpOffset = GET_JUMP_OFFSET(pc);
        if (result) {
          ADVANCE(jumpOffset);
          PREDICT_NEXT(JumpTarget);
          PREDICT_NEXT(LoopHead);
        } else {
          ADVANCE(JSOpLength_JumpIfTrue);
        }
        DISPATCH();
      }
      CASE(JumpIfFalse) {
        bool result;
        if (HybridICs) {
          result = ToBoolean(Stack::handle(sp));
          POP();
          NEXT_IC();
        } else {
          IC_POP_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(ToBool);
          result = Value::fromRawBits(ic_ret).toBoolean();
        }
        int32_t jumpOffset = GET_JUMP_OFFSET(pc);
        if (!result) {
          ADVANCE(jumpOffset);
          PREDICT_NEXT(JumpTarget);
          PREDICT_NEXT(LoopHead);
        } else {
          ADVANCE(JSOpLength_JumpIfFalse);
        }
        DISPATCH();
      }

      CASE(Add) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int64_t lhs = sp[1].asValue().toInt32();
            int64_t rhs = sp[0].asValue().toInt32();
            if (lhs + rhs >= int64_t(INT32_MIN) &&
                lhs + rhs <= int64_t(INT32_MAX)) {
              POP();
              sp[0] = StackVal(Int32Value(int32_t(lhs + rhs)));
              NEXT_IC();
              END_OP(Add);
            }
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(NumberValue(lhs + rhs));
            NEXT_IC();
            END_OP(Add);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!AddOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Add);
        }
        goto generic_binary;
      }

      CASE(Sub) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int64_t lhs = sp[1].asValue().toInt32();
            int64_t rhs = sp[0].asValue().toInt32();
            if (lhs - rhs >= int64_t(INT32_MIN) &&
                lhs - rhs <= int64_t(INT32_MAX)) {
              POP();
              sp[0] = StackVal(Int32Value(int32_t(lhs - rhs)));
              NEXT_IC();
              END_OP(Sub);
            }
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(NumberValue(lhs - rhs));
            NEXT_IC();
            END_OP(Add);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!SubOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Sub);
        }
        goto generic_binary;
      }

      CASE(Mul) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int64_t lhs = sp[1].asValue().toInt32();
            int64_t rhs = sp[0].asValue().toInt32();
            int64_t product = lhs * rhs;
            if (product >= int64_t(INT32_MIN) &&
                product <= int64_t(INT32_MAX) &&
                (product != 0 || !((lhs < 0) ^ (rhs < 0)))) {
              POP();
              sp[0] = StackVal(Int32Value(int32_t(product)));
              NEXT_IC();
              END_OP(Mul);
            }
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(NumberValue(lhs * rhs));
            NEXT_IC();
            END_OP(Mul);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!MulOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Mul);
        }
        goto generic_binary;
      }
      CASE(Div) {
        if (HybridICs) {
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(NumberValue(NumberDiv(lhs, rhs)));
            NEXT_IC();
            END_OP(Div);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!DivOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Div);
        }
        goto generic_binary;
      }
      CASE(Mod) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int64_t lhs = sp[1].asValue().toInt32();
            int64_t rhs = sp[0].asValue().toInt32();
            if (lhs > 0 && rhs > 0) {
              int64_t mod = lhs % rhs;
              POP();
              sp[0] = StackVal(Int32Value(int32_t(mod)));
              NEXT_IC();
              END_OP(Mod);
            }
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(DoubleValue(NumberMod(lhs, rhs)));
            NEXT_IC();
            END_OP(Mod);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!ModOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Mod);
        }
        goto generic_binary;
      }
      CASE(Pow) {
        if (HybridICs) {
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            POP();
            sp[0] = StackVal(NumberValue(ecmaPow(lhs, rhs)));
            NEXT_IC();
            END_OP(Pow);
          }

          MutableHandleValue lhs = Stack::handleMut(sp + 1);
          MutableHandleValue rhs = Stack::handleMut(sp);
          MutableHandleValue result = Stack::handleMut(sp + 1);
          {
            PUSH_EXIT_FRAME();
            if (!PowOperation(cx, lhs, rhs, result)) {
              GOTO_ERROR();
            }
          }
          POP();
          NEXT_IC();
          END_OP(Pow);
        }
        goto generic_binary;
      }
      CASE(BitOr) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int32_t lhs = sp[1].asValue().toInt32();
            int32_t rhs = sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(Int32Value(lhs | rhs));
            NEXT_IC();
            END_OP(BitOr);
          }
        }
        goto generic_binary;
      }
      CASE(BitAnd) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int32_t lhs = sp[1].asValue().toInt32();
            int32_t rhs = sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(Int32Value(lhs & rhs));
            NEXT_IC();
            END_OP(BitAnd);
          }
        }
        goto generic_binary;
      }
      CASE(BitXor) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int32_t lhs = sp[1].asValue().toInt32();
            int32_t rhs = sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(Int32Value(lhs ^ rhs));
            NEXT_IC();
            END_OP(BitXor);
          }
        }
        goto generic_binary;
      }
      CASE(Lsh) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            // Unsigned to avoid undefined behavior on left-shift overflow
            // (see comment in BitLshOperation in Interpreter.cpp).
            uint32_t lhs = uint32_t(sp[1].asValue().toInt32());
            uint32_t rhs = uint32_t(sp[0].asValue().toInt32());
            POP();
            rhs &= 31;
            sp[0] = StackVal(Int32Value(int32_t(lhs << rhs)));
            NEXT_IC();
            END_OP(Lsh);
          }
        }
        goto generic_binary;
      }
      CASE(Rsh) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            int32_t lhs = sp[1].asValue().toInt32();
            int32_t rhs = sp[0].asValue().toInt32();
            POP();
            rhs &= 31;
            sp[0] = StackVal(Int32Value(lhs >> rhs));
            NEXT_IC();
            END_OP(Rsh);
          }
        }
        goto generic_binary;
      }
      CASE(Ursh) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            uint32_t lhs = uint32_t(sp[1].asValue().toInt32());
            int32_t rhs = sp[0].asValue().toInt32();
            POP();
            rhs &= 31;
            uint32_t result = lhs >> rhs;
            if (result <= uint32_t(INT32_MAX)) {
              sp[0] = StackVal(Int32Value(int32_t(result)));
            } else {
              sp[0] = StackVal(NumberValue(double(result)));
            }
            NEXT_IC();
            END_OP(Ursh);
          }
        }
        goto generic_binary;
      }

    generic_binary: {
      static_assert(JSOpLength_BitOr == JSOpLength_BitXor);
      static_assert(JSOpLength_BitOr == JSOpLength_BitAnd);
      static_assert(JSOpLength_BitOr == JSOpLength_Lsh);
      static_assert(JSOpLength_BitOr == JSOpLength_Rsh);
      static_assert(JSOpLength_BitOr == JSOpLength_Ursh);
      static_assert(JSOpLength_BitOr == JSOpLength_Add);
      static_assert(JSOpLength_BitOr == JSOpLength_Sub);
      static_assert(JSOpLength_BitOr == JSOpLength_Mul);
      static_assert(JSOpLength_BitOr == JSOpLength_Div);
      static_assert(JSOpLength_BitOr == JSOpLength_Mod);
      static_assert(JSOpLength_BitOr == JSOpLength_Pow);
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      IC_ZERO_ARG(2);
      INVOKE_IC(BinaryArith);
      IC_PUSH_RESULT();
      END_OP(Div);
    }

      CASE(Eq) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result =
                sp[0].asValue().toInt32() == sp[1].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Eq);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs == rhs;
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Eq);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            bool result =
                sp[0].asValue().toNumber() == sp[1].asValue().toNumber();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Eq);
          }
        }
        goto generic_cmp;
      }

      CASE(Ne) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result =
                sp[0].asValue().toInt32() != sp[1].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Ne);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs != rhs;
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Ne);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            bool result =
                sp[0].asValue().toNumber() != sp[1].asValue().toNumber();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Eq);
          }
        }
        goto generic_cmp;
      }

      CASE(Lt) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result = sp[1].asValue().toInt32() < sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Lt);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs < rhs;
            if (std::isnan(lhs) || std::isnan(rhs)) {
              result = false;
            }
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Lt);
          }
        }
        goto generic_cmp;
      }
      CASE(Le) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result =
                sp[1].asValue().toInt32() <= sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Le);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs <= rhs;
            if (std::isnan(lhs) || std::isnan(rhs)) {
              result = false;
            }
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Le);
          }
        }
        goto generic_cmp;
      }
      CASE(Gt) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result = sp[1].asValue().toInt32() > sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Gt);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs > rhs;
            if (std::isnan(lhs) || std::isnan(rhs)) {
              result = false;
            }
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Gt);
          }
        }
        goto generic_cmp;
      }
      CASE(Ge) {
        if (HybridICs) {
          if (sp[0].asValue().isInt32() && sp[1].asValue().isInt32()) {
            bool result =
                sp[1].asValue().toInt32() >= sp[0].asValue().toInt32();
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Ge);
          }
          if (sp[0].asValue().isNumber() && sp[1].asValue().isNumber()) {
            double lhs = sp[1].asValue().toNumber();
            double rhs = sp[0].asValue().toNumber();
            bool result = lhs >= rhs;
            if (std::isnan(lhs) || std::isnan(rhs)) {
              result = false;
            }
            POP();
            sp[0] = StackVal(BooleanValue(result));
            NEXT_IC();
            END_OP(Ge);
          }
        }
        goto generic_cmp;
      }

      CASE(StrictEq)
      CASE(StrictNe) {
        if (HybridICs) {
          bool result;
          HandleValue lval = Stack::handle(sp + 1);
          HandleValue rval = Stack::handle(sp);
          if (sp[0].asValue().isString() && sp[1].asValue().isString()) {
            PUSH_EXIT_FRAME();
            if (!js::StrictlyEqual(cx, lval, rval, &result)) {
              GOTO_ERROR();
            }
          } else {
            if (!js::StrictlyEqual(nullptr, lval, rval, &result)) {
              GOTO_ERROR();
            }
          }
          POP();
          sp[0] = StackVal(
              BooleanValue((JSOp(*pc) == JSOp::StrictEq) ? result : !result));
          NEXT_IC();
          END_OP(StrictEq);
        } else {
          goto generic_cmp;
        }
      }

    generic_cmp: {
      static_assert(JSOpLength_Eq == JSOpLength_Ne);
      static_assert(JSOpLength_Eq == JSOpLength_StrictEq);
      static_assert(JSOpLength_Eq == JSOpLength_StrictNe);
      static_assert(JSOpLength_Eq == JSOpLength_Lt);
      static_assert(JSOpLength_Eq == JSOpLength_Gt);
      static_assert(JSOpLength_Eq == JSOpLength_Le);
      static_assert(JSOpLength_Eq == JSOpLength_Ge);
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      IC_ZERO_ARG(2);
      INVOKE_IC(Compare);
      IC_PUSH_RESULT();
      END_OP(Eq);
    }

      CASE(Instanceof) {
        IC_POP_ARG(1);
        IC_POP_ARG(0);
        IC_ZERO_ARG(2);
        INVOKE_IC(InstanceOf);
        IC_PUSH_RESULT();
        END_OP(Instanceof);
      }

      CASE(In) {
        IC_POP_ARG(1);
        IC_POP_ARG(0);
        IC_ZERO_ARG(2);
        INVOKE_IC(In);
        IC_PUSH_RESULT();
        END_OP(In);
      }

      CASE(ToPropertyKey) {
        IC_POP_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(ToPropertyKey);
        IC_PUSH_RESULT();
        END_OP(ToPropertyKey);
      }

      CASE(ToString) {
        if (sp[0].asValue().isString()) {
          END_OP(ToString);
        }
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          if (JSString* result = ToStringSlow<NoGC>(
                  ctx.frameMgr.cxForLocalUseOnly(), value0)) {
            PUSH(StackVal(StringValue(result)));
          } else {
            {
              PUSH_EXIT_FRAME();
              result = ToString<CanGC>(cx, value0);
              if (!result) {
                GOTO_ERROR();
              }
            }
            PUSH(StackVal(StringValue(result)));
          }
        }
        END_OP(ToString);
      }

      CASE(IsNullOrUndefined) {
        bool result = sp[0].asValue().isNull() || sp[0].asValue().isUndefined();
        PUSH(StackVal(BooleanValue(result)));
        END_OP(IsNullOrUndefined);
      }

      CASE(GlobalThis) {
        PUSH(StackVal(ObjectValue(*ctx.frameMgr.cxForLocalUseOnly()
                                       ->global()
                                       ->lexicalEnvironment()
                                       .thisObject())));
        END_OP(GlobalThis);
      }

      CASE(NonSyntacticGlobalThis) {
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          ReservedRooted<Value> value0(&state.value0);
          {
            PUSH_EXIT_FRAME();
            js::GetNonSyntacticGlobalThis(cx, obj0, &value0);
          }
          PUSH(StackVal(value0));
        }
        END_OP(NonSyntacticGlobalThis);
      }

      CASE(NewTarget) {
        PUSH(StackVal(frame->newTarget()));
        END_OP(NewTarget);
      }

      CASE(DynamicImport) {
        {
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // options
          ReservedRooted<Value> value1(&state.value1,
                                       POP().asValue());  // specifier
          JSObject* promise;
          {
            PUSH_EXIT_FRAME();
            promise = StartDynamicModuleImport(cx, script, value1, value0);
            if (!promise) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*promise)));
        }
        END_OP(DynamicImport);
      }

      CASE(ImportMeta) {
        JSObject* metaObject;
        {
          PUSH_EXIT_FRAME();
          metaObject = ImportMetaOperation(cx, script);
          if (!metaObject) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*metaObject)));
        END_OP(ImportMeta);
      }

      CASE(NewInit) {
        if (HybridICs) {
          JSObject* obj;
          {
            PUSH_EXIT_FRAME();
            obj = NewObjectOperation(cx, script, pc);
            if (!obj) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*obj)));
          NEXT_IC();
          END_OP(NewInit);
        } else {
          IC_ZERO_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(NewObject);
          IC_PUSH_RESULT();
          END_OP(NewInit);
        }
      }
      CASE(NewObject) {
        if (HybridICs) {
          JSObject* obj;
          {
            PUSH_EXIT_FRAME();
            obj = NewObjectOperation(cx, script, pc);
            if (!obj) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*obj)));
          NEXT_IC();
          END_OP(NewObject);
        } else {
          IC_ZERO_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(NewObject);
          IC_PUSH_RESULT();
          END_OP(NewObject);
        }
      }
      CASE(Object) {
        PUSH(StackVal(ObjectValue(*script->getObject(pc))));
        END_OP(Object);
      }
      CASE(ObjWithProto) {
        {
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());
          JSObject* obj;
          {
            PUSH_EXIT_FRAME();
            obj = ObjectWithProtoOperation(cx, value0);
            if (!obj) {
              GOTO_ERROR();
            }
          }
          sp[0] = StackVal(ObjectValue(*obj));
        }
        END_OP(ObjWithProto);
      }

      CASE(InitElem)
      CASE(InitHiddenElem)
      CASE(InitLockedElem)
      CASE(InitElemInc)
      CASE(SetElem)
      CASE(StrictSetElem) {
        static_assert(JSOpLength_InitElem == JSOpLength_InitHiddenElem);
        static_assert(JSOpLength_InitElem == JSOpLength_InitLockedElem);
        static_assert(JSOpLength_InitElem == JSOpLength_InitElemInc);
        static_assert(JSOpLength_InitElem == JSOpLength_SetElem);
        static_assert(JSOpLength_InitElem == JSOpLength_StrictSetElem);
        StackVal val = sp[0];
        IC_POP_ARG(2);
        IC_POP_ARG(1);
        IC_SET_ARG_FROM_STACK(0, 0);
        if (JSOp(*pc) == JSOp::SetElem || JSOp(*pc) == JSOp::StrictSetElem) {
          sp[0] = val;
        }
        INVOKE_IC(SetElem);
        if (JSOp(*pc) == JSOp::InitElemInc) {
          PUSH(StackVal(Int32Value(Value::fromRawBits(ic_arg1).toInt32() + 1)));
        }
        END_OP(InitElem);
      }

      CASE(InitPropGetter)
      CASE(InitHiddenPropGetter)
      CASE(InitPropSetter)
      CASE(InitHiddenPropSetter) {
        static_assert(JSOpLength_InitPropGetter ==
                      JSOpLength_InitHiddenPropGetter);
        static_assert(JSOpLength_InitPropGetter == JSOpLength_InitPropSetter);
        static_assert(JSOpLength_InitPropGetter ==
                      JSOpLength_InitHiddenPropSetter);
        {
          ReservedRooted<JSObject*> obj1(&state.obj1,
                                         &POP().asValue().toObject());  // val
          ReservedRooted<JSObject*> obj0(
              &state.obj0, &sp[0].asValue().toObject());  // obj; leave on stack
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          {
            PUSH_EXIT_FRAME();
            if (!InitPropGetterSetterOperation(cx, pc, obj0, name0, obj1)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(InitPropGetter);
      }

      CASE(InitElemGetter)
      CASE(InitHiddenElemGetter)
      CASE(InitElemSetter)
      CASE(InitHiddenElemSetter) {
        static_assert(JSOpLength_InitElemGetter ==
                      JSOpLength_InitHiddenElemGetter);
        static_assert(JSOpLength_InitElemGetter == JSOpLength_InitElemSetter);
        static_assert(JSOpLength_InitElemGetter ==
                      JSOpLength_InitHiddenElemSetter);
        {
          ReservedRooted<JSObject*> obj1(&state.obj1,
                                         &POP().asValue().toObject());  // val
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // idval
          ReservedRooted<JSObject*> obj0(
              &state.obj0, &sp[0].asValue().toObject());  // obj; leave on stack
          {
            PUSH_EXIT_FRAME();
            if (!InitElemGetterSetterOperation(cx, pc, obj0, value0, obj1)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(InitElemGetter);
      }

      CASE(GetProp)
      CASE(GetBoundName) {
        static_assert(JSOpLength_GetProp == JSOpLength_GetBoundName);
        IC_POP_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetProp);
        IC_PUSH_RESULT();
        END_OP(GetProp);
      }
      CASE(GetPropSuper) {
        IC_POP_ARG(0);
        IC_POP_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetPropSuper);
        IC_PUSH_RESULT();
        END_OP(GetPropSuper);
      }

      CASE(GetElem) {
        HandleValue lhs = Stack::handle(&sp[1]);
        HandleValue rhs = Stack::handle(&sp[0]);
        uint32_t index;
        if (IsDefinitelyIndex(rhs, &index)) {
          if (lhs.isString()) {
            JSString* str = lhs.toString();
            if (index < str->length() && str->isLinear()) {
              JSLinearString* linear = &str->asLinear();
              char16_t c = linear->latin1OrTwoByteChar(index);
              StaticStrings& sstr =
                  ctx.frameMgr.cxForLocalUseOnly()->staticStrings();
              if (sstr.hasUnit(c)) {
                sp[1] = StackVal(StringValue(sstr.getUnit(c)));
                POP();
                NEXT_IC();
                END_OP(GetElem);
              }
            }
          }
          if (lhs.isObject()) {
            JSObject* obj = &lhs.toObject();
            Value ret;
            if (GetElementNoGC(ctx.frameMgr.cxForLocalUseOnly(), obj, lhs,
                               index, &ret)) {
              sp[1] = StackVal(ret);
              POP();
              NEXT_IC();
              END_OP(GetElem);
            }
          }
        }

        IC_POP_ARG(1);
        IC_POP_ARG(0);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetElem);
        IC_PUSH_RESULT();
        END_OP(GetElem);
      }

      CASE(GetElemSuper) {
        // N.B.: second and third args are out of order! See the saga at
        // https://bugzilla.mozilla.org/show_bug.cgi?id=1709328; this is
        // an echo of that issue.
        IC_POP_ARG(1);
        IC_POP_ARG(2);
        IC_POP_ARG(0);
        INVOKE_IC(GetElemSuper);
        IC_PUSH_RESULT();
        END_OP(GetElemSuper);
      }

      CASE(DelProp) {
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          bool res = false;
          {
            PUSH_EXIT_FRAME();
            if (!DelPropOperation<false>(cx, value0, name0, &res)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(BooleanValue(res)));
        }
        END_OP(DelProp);
      }
      CASE(StrictDelProp) {
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          bool res = false;
          {
            PUSH_EXIT_FRAME();
            if (!DelPropOperation<true>(cx, value0, name0, &res)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(BooleanValue(res)));
        }
        END_OP(StrictDelProp);
      }
      CASE(DelElem) {
        {
          ReservedRooted<Value> value1(&state.value1, POP().asValue());
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          bool res = false;
          {
            PUSH_EXIT_FRAME();
            if (!DelElemOperation<false>(cx, value0, value1, &res)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(BooleanValue(res)));
        }
        END_OP(DelElem);
      }
      CASE(StrictDelElem) {
        {
          ReservedRooted<Value> value1(&state.value1, POP().asValue());
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          bool res = false;
          {
            PUSH_EXIT_FRAME();
            if (!DelElemOperation<true>(cx, value0, value1, &res)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(BooleanValue(res)));
        }
        END_OP(StrictDelElem);
      }

      CASE(HasOwn) {
        IC_POP_ARG(1);
        IC_POP_ARG(0);
        IC_ZERO_ARG(2);
        INVOKE_IC(HasOwn);
        IC_PUSH_RESULT();
        END_OP(HasOwn);
      }

      CASE(CheckPrivateField) {
        IC_SET_ARG_FROM_STACK(1, 0);
        IC_SET_ARG_FROM_STACK(0, 1);
        IC_ZERO_ARG(2);
        INVOKE_IC(CheckPrivateField);
        IC_PUSH_RESULT();
        END_OP(CheckPrivateField);
      }

      CASE(NewPrivateName) {
        {
          ReservedRooted<JSAtom*> atom0(&state.atom0, script->getAtom(pc));
          JS::Symbol* symbol;
          {
            PUSH_EXIT_FRAME();
            symbol = NewPrivateName(cx, atom0);
            if (!symbol) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(SymbolValue(symbol)));
        }
        END_OP(NewPrivateName);
      }

      CASE(SuperBase) {
        JSFunction& superEnvFunc = POP().asValue().toObject().as<JSFunction>();
        MOZ_ASSERT(superEnvFunc.allowSuperProperty());
        MOZ_ASSERT(superEnvFunc.baseScript()->needsHomeObject());
        const Value& homeObjVal = superEnvFunc.getExtendedSlot(
            FunctionExtended::METHOD_HOMEOBJECT_SLOT);

        JSObject* homeObj = &homeObjVal.toObject();
        JSObject* superBase = HomeObjectSuperBase(homeObj);

        PUSH(StackVal(ObjectOrNullValue(superBase)));
        END_OP(SuperBase);
      }

      CASE(SetPropSuper)
      CASE(StrictSetPropSuper) {
        // stack signature: receiver, lval, rval => rval
        static_assert(JSOpLength_SetPropSuper == JSOpLength_StrictSetPropSuper);
        bool strict = JSOp(*pc) == JSOp::StrictSetPropSuper;
        {
          ReservedRooted<Value> value2(&state.value2, POP().asValue());  // rval
          ReservedRooted<Value> value1(&state.value1, POP().asValue());  // lval
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // recevier
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          {
            PUSH_EXIT_FRAME();
            // SetPropertySuper(cx, lval, receiver, name, rval, strict)
            // (N.B.: lval and receiver are transposed!)
            if (!SetPropertySuper(cx, value1, value0, name0, value2, strict)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(value2));
        }
        END_OP(SetPropSuper);
      }

      CASE(SetElemSuper)
      CASE(StrictSetElemSuper) {
        // stack signature: receiver, key, lval, rval => rval
        static_assert(JSOpLength_SetElemSuper == JSOpLength_StrictSetElemSuper);
        bool strict = JSOp(*pc) == JSOp::StrictSetElemSuper;
        {
          ReservedRooted<Value> value3(&state.value3, POP().asValue());  // rval
          ReservedRooted<Value> value2(&state.value2, POP().asValue());  // lval
          ReservedRooted<Value> value1(&state.value1,
                                       POP().asValue());  // index
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // receiver
          {
            PUSH_EXIT_FRAME();
            // SetElementSuper(cx, lval, receiver, index, rval, strict)
            // (N.B.: lval, receiver and index are rotated!)
            if (!SetElementSuper(cx, value2, value0, value1, value3, strict)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(value3));  // value
        }
        END_OP(SetElemSuper);
      }

      CASE(Iter) {
        IC_POP_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetIterator);
        IC_PUSH_RESULT();
        END_OP(Iter);
      }

      CASE(MoreIter) {
        // iter => iter, name
        Value v = IteratorMore(&sp[0].asValue().toObject());
        PUSH(StackVal(v));
        END_OP(MoreIter);
      }

      CASE(IsNoIter) {
        // iter => iter, bool
        bool result = sp[0].asValue().isMagic(JS_NO_ITER_VALUE);
        PUSH(StackVal(BooleanValue(result)));
        END_OP(IsNoIter);
      }

      CASE(EndIter) {
        // iter, interval =>
        POP();
        CloseIterator(&POP().asValue().toObject());
        END_OP(EndIter);
      }

      CASE(CloseIter) {
        IC_SET_OBJ_ARG(0, &POP().asValue().toObject());
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(CloseIter);
        END_OP(CloseIter);
      }

      CASE(CheckIsObj) {
        if (!sp[0].asValue().isObject()) {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(
              js::ThrowCheckIsObject(cx, js::CheckIsObjectKind(GET_UINT8(pc))));
          /* abandon frame; error handler will re-establish sp */
          GOTO_ERROR();
        }
        END_OP(CheckIsObj);
      }

      CASE(CheckObjCoercible) {
        {
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());
          if (value0.isNullOrUndefined()) {
            PUSH_EXIT_FRAME();
            MOZ_ALWAYS_FALSE(ThrowObjectCoercible(cx, value0));
            /* abandon frame; error handler will re-establish sp */
            GOTO_ERROR();
          }
        }
        END_OP(CheckObjCoercible);
      }

      CASE(ToAsyncIter) {
        // iter, next => asynciter
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());  // next
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());  // iter
          JSObject* result;
          {
            PUSH_EXIT_FRAME();
            result = CreateAsyncFromSyncIterator(cx, obj0, value0);
            if (!result) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*result)));
        }
        END_OP(ToAsyncIter);
      }

      CASE(MutateProto) {
        // obj, protoVal => obj
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &sp[0].asValue().toObject());
          {
            PUSH_EXIT_FRAME();
            if (!MutatePrototype(cx, obj0.as<PlainObject>(), value0)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(MutateProto);
      }

      CASE(NewArray) {
        if (HybridICs) {
          ArrayObject* obj;
          {
            PUSH_EXIT_FRAME();
            uint32_t length = GET_UINT32(pc);
            obj = NewArrayOperation(cx, length);
            if (!obj) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*obj)));
          NEXT_IC();
          END_OP(NewArray);
        } else {
          IC_ZERO_ARG(0);
          IC_ZERO_ARG(1);
          IC_ZERO_ARG(2);
          INVOKE_IC(NewArray);
          IC_PUSH_RESULT();
          END_OP(NewArray);
        }
      }

      CASE(InitElemArray) {
        // array, val => array
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &sp[0].asValue().toObject());
          {
            PUSH_EXIT_FRAME();
            InitElemArrayOperation(cx, pc, obj0.as<ArrayObject>(), value0);
          }
        }
        END_OP(InitElemArray);
      }

      CASE(Hole) {
        PUSH(StackVal(MagicValue(JS_ELEMENTS_HOLE)));
        END_OP(Hole);
      }

      CASE(RegExp) {
        JSObject* obj;
        {
          PUSH_EXIT_FRAME();
          ReservedRooted<JSObject*> obj0(&state.obj0, script->getRegExp(pc));
          obj = CloneRegExpObject(cx, obj0.as<RegExpObject>());
          if (!obj) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*obj)));
        END_OP(RegExp);
      }

      CASE(Lambda) {
        {
          ReservedRooted<JSFunction*> fun0(&state.fun0,
                                           script->getFunction(pc));
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          JSObject* res;
          {
            PUSH_EXIT_FRAME();
            res = js::Lambda(cx, fun0, obj0);
            if (!res) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*res)));
        }
        END_OP(Lambda);
      }

      CASE(SetFunName) {
        // fun, name => fun
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());  // name
          ReservedRooted<JSFunction*> fun0(
              &state.fun0, &sp[0].asValue().toObject().as<JSFunction>());
          FunctionPrefixKind prefixKind = FunctionPrefixKind(GET_UINT8(pc));
          {
            PUSH_EXIT_FRAME();
            if (!SetFunctionName(cx, fun0, value0, prefixKind)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(SetFunName);
      }

      CASE(InitHomeObject) {
        // fun, homeObject => fun
        {
          ReservedRooted<JSObject*> obj0(
              &state.obj0, &POP().asValue().toObject());  // homeObject
          ReservedRooted<JSFunction*> fun0(
              &state.fun0, &sp[0].asValue().toObject().as<JSFunction>());
          MOZ_ASSERT(fun0->allowSuperProperty());
          MOZ_ASSERT(obj0->is<PlainObject>() || obj0->is<JSFunction>());
          fun0->setExtendedSlot(FunctionExtended::METHOD_HOMEOBJECT_SLOT,
                                ObjectValue(*obj0));
        }
        END_OP(InitHomeObject);
      }

      CASE(CheckClassHeritage) {
        {
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());
          {
            PUSH_EXIT_FRAME();
            if (!CheckClassHeritageOperation(cx, value0)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(CheckClassHeritage);
      }

      CASE(FunWithProto) {
        // proto => obj
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());  // proto
          ReservedRooted<JSObject*> obj1(&state.obj1,
                                         frame->environmentChain());
          ReservedRooted<JSFunction*> fun0(&state.fun0,
                                           script->getFunction(pc));
          JSObject* obj;
          {
            PUSH_EXIT_FRAME();
            obj = FunWithProtoOperation(cx, fun0, obj1, obj0);
            if (!obj) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(ObjectValue(*obj)));
        }
        END_OP(FunWithProto);
      }

      CASE(BuiltinObject) {
        auto kind = BuiltinObjectKind(GET_UINT8(pc));
        JSObject* builtin;
        {
          PUSH_EXIT_FRAME();
          builtin = BuiltinObjectOperation(cx, kind);
          if (!builtin) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*builtin)));
        END_OP(BuiltinObject);
      }

      CASE(Call)
      CASE(CallIgnoresRv)
      CASE(CallContent)
      CASE(CallIter)
      CASE(CallContentIter)
      CASE(Eval)
      CASE(StrictEval)
      CASE(SuperCall)
      CASE(New)
      CASE(NewContent) {
        static_assert(JSOpLength_Call == JSOpLength_CallIgnoresRv);
        static_assert(JSOpLength_Call == JSOpLength_CallContent);
        static_assert(JSOpLength_Call == JSOpLength_CallIter);
        static_assert(JSOpLength_Call == JSOpLength_CallContentIter);
        static_assert(JSOpLength_Call == JSOpLength_Eval);
        static_assert(JSOpLength_Call == JSOpLength_StrictEval);
        static_assert(JSOpLength_Call == JSOpLength_SuperCall);
        static_assert(JSOpLength_Call == JSOpLength_New);
        static_assert(JSOpLength_Call == JSOpLength_NewContent);
        JSOp op = JSOp(*pc);
        bool constructing = (op == JSOp::New || op == JSOp::NewContent ||
                             op == JSOp::SuperCall);
        uint32_t argc = GET_ARGC(pc);
        do {
          {
            if (!InlineCalls) {
              break;
            }

            // CallArgsFromSp would be called with
            // - numValues = argc + 2 + constructing
            // - stackSlots = argc + constructing
            // - sp = vp + numValues
            // CallArgs::create then gets
            // - argc_ = stackSlots - constructing = argc
            // - argv_ = sp - stackSlots = vp + 2
            // our arguments are in reverse order compared to what CallArgs
            // expects so we should subtract any array subscripts from (sp +
            // stackSlots - 1)
            StackVal* firstArg = sp + argc + constructing - 1;

            // callee is argv_[-2] -> sp + argc + constructing + 1
            // this is   argv_[-1] -> sp + argc + constructing
            // newTarget is argv_[argc_] -> sp + constructing - 1
            // but this/newTarget are only used when constructing is 1 so we can
            // simplify this is   argv_[-1] -> sp + argc + 1 newTarget is
            // argv_[argc_] -> sp

            HandleValue callee = Stack::handle(firstArg + 2);
            if (!callee.isObject() || !callee.toObject().is<JSFunction>()) {
              TRACE_PRINTF("missed fastpath: not a function\n");
              break;
            }
            ReservedRooted<JSFunction*> func(
                &state.fun0, &callee.toObject().as<JSFunction>());
            if (!func->hasBaseScript() || !func->isInterpreted()) {
              TRACE_PRINTF("missed fastpath: not an interpreted script\n");
              break;
            }
            if (!constructing && func->isClassConstructor()) {
              TRACE_PRINTF(
                  "missed fastpath: constructor called without `new`\n");
              break;
            }
            if (!func->baseScript()->hasBytecode()) {
              TRACE_PRINTF("missed fastpath: no bytecode\n");
              break;
            }
            ReservedRooted<JSScript*> calleeScript(
                &state.script0, func->baseScript()->asJSScript());
            if (!calleeScript->hasJitScript()) {
              TRACE_PRINTF("missed fastpath: no jit-script\n");
              break;
            }
            if (ctx.frameMgr.cxForLocalUseOnly()->realm() !=
                calleeScript->realm()) {
              TRACE_PRINTF("missed fastpath: mismatched realm\n");
              break;
            }
            if (argc < func->nargs()) {
              TRACE_PRINTF("missed fastpath: not enough arguments\n");
              break;
            }
#ifdef ENABLE_JS_PBL_WEVAL
            if (calleeScript->hasWeval() && calleeScript->weval().func) {
              TRACE_PRINTF("missed fastpath: specialized function exists\n");
              break;
            }
#endif

            // Fast-path: function, interpreted, has JitScript, same realm, no
            // argument underflow.

            // Include newTarget in the args if it exists; exclude callee
            uint32_t totalArgs = argc + 1 + constructing;
            StackVal* origArgs = sp;

            TRACE_PRINTF(
                "Call fastpath: argc = %d origArgs = %p callee = %" PRIx64 "\n",
                argc, origArgs, callee.get().asRawBits());

            if (!ctx.stack.check(sp, sizeof(StackVal) * (totalArgs + 3))) {
              TRACE_PRINTF("missed fastpath: would cause stack overrun\n");
              break;
            }

            if (constructing) {
              MutableHandleValue thisv = Stack::handleMut(firstArg + 1);
              if (!thisv.isObject()) {
                HandleValue newTarget = Stack::handle(firstArg - argc);
                ReservedRooted<JSObject*> obj0(&state.obj0,
                                               &newTarget.toObject());

                PUSH_EXIT_FRAME();
                // CreateThis might discard the JitScript but we're counting on
                // it continuing to exist while we evaluate the fastpath.
                AutoKeepJitScripts keepJitScript(cx);
                if (!CreateThis(cx, func, obj0, GenericObject, thisv)) {
                  GOTO_ERROR();
                }

                TRACE_PRINTF("created %" PRIx64 "\n", thisv.get().asRawBits());
              }
            }

            // 0. Save current PC in current frame, so we can retrieve
            // it later.
            frame->interpreterPC() = pc;

            // 1. Push a baseline stub frame. Don't use the frame manager
            // -- we don't want the frame to be auto-freed when we leave
            // this scope, and we don't want to shadow `sp`.
            StackVal* exitFP = ctx.stack.pushExitFrame(sp, frame);
            MOZ_ASSERT(exitFP);  // safety: stack margin.
            sp = exitFP;
            TRACE_PRINTF("exit frame at %p\n", exitFP);

            // 2. Modify exit code to nullptr (this is where ICStubReg is
            // normally saved; the tracing code can skip if null).
            PUSHNATIVE(StackValNative(nullptr));

            // 3. Push args in proper order (they are reversed in our
            // downward-growth stack compared to what the calling
            // convention expects).
            for (uint32_t i = 0; i < totalArgs; i++) {
              PUSH(origArgs[i]);
            }

            // 4. Push inter-frame content: callee token, descriptor for
            // above.
            PUSHNATIVE(StackValNative(CalleeToToken(func, constructing)));
            PUSHNATIVE(StackValNative(
                MakeFrameDescriptorForJitCall(FrameType::BaselineStub, argc)));

            // 5. Push fake return address, set script, push baseline frame.
            PUSHNATIVE(StackValNative(nullptr));
            script.set(calleeScript);
            BaselineFrame* newFrame =
                ctx.stack.pushFrame(sp, ctx.frameMgr.cxForLocalUseOnly(),
                                    /* envChain = */ func->environment());
            MOZ_ASSERT(newFrame);  // safety: stack margin.
            TRACE_PRINTF("callee frame at %p\n", newFrame);
            frame = newFrame;
            ctx.frameMgr.switchToFrame(frame);
            ctx.frame = frame;
            icEntries = frame->icScript()->icEntries();
            // 6. Set up PC and SP for callee.
            sp = reinterpret_cast<StackVal*>(frame);
            pc = calleeScript->code();
            entryPC = pc;
            isd = calleeScript->immutableScriptData();
            // 7. Check callee stack space for max stack depth.
            if (!ctx.stack.check(sp,
                                 sizeof(StackVal) * calleeScript->nslots())) {
              PUSH_EXIT_FRAME();
              ReportOverRecursed(ctx.frameMgr.cxForLocalUseOnly());
              GOTO_ERROR();
            }
            // 8. Push local slots, and set return value to `undefined` by
            // default.
            uint32_t nfixed = calleeScript->nfixed();
            for (uint32_t i = 0; i < nfixed; i++) {
              PUSH(StackVal(UndefinedValue()));
            }
            ret->setUndefined();
            // 9. Initialize environment objects.
            if (func->needsFunctionEnvironmentObjects()) {
              PUSH_EXIT_FRAME();
              if (!js::InitFunctionEnvironmentObjects(cx, frame)) {
                GOTO_ERROR();
              }
            }
            // 10. Set debug flag, if appropriate.
            if (script->isDebuggee()) {
              TRACE_PRINTF("Script is debuggee\n");
              frame->setIsDebuggee();

              PUSH_EXIT_FRAME();
              if (!DebugPrologue(cx, frame)) {
                GOTO_ERROR();
              }
            }
            // 11. Check for interrupts.
#ifndef __wasi__
            if (ctx.frameMgr.cxForLocalUseOnly()->hasAnyPendingInterrupt()) {
              PUSH_EXIT_FRAME();
              if (!InterruptCheck(cx)) {
                GOTO_ERROR();
              }
            }
#endif
            // 12. Initialize coverage tables, if needed.
            if (!script->hasScriptCounts()) {
              if (ctx.frameMgr.cxForLocalUseOnly()
                      ->realm()
                      ->collectCoverageForDebug()) {
                PUSH_EXIT_FRAME();
                if (!script->initScriptCounts(cx)) {
                  GOTO_ERROR();
                }
              }
            }
            COUNT_COVERAGE_MAIN();
          }

          // Everything is switched to callee context now -- dispatch!
          DISPATCH();
        } while (0);

        // Slow path: use the IC!
        ic_arg0 = argc;
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        ctx.icregs.extraArgs = 2 + constructing;
        INVOKE_IC(Call);
        POPN(argc + 2 + constructing);
        PUSH(StackVal(Value::fromRawBits(ic_ret)));
        END_OP(Call);
      }

      CASE(SpreadCall)
      CASE(SpreadEval)
      CASE(StrictSpreadEval) {
        static_assert(JSOpLength_SpreadCall == JSOpLength_SpreadEval);
        static_assert(JSOpLength_SpreadCall == JSOpLength_StrictSpreadEval);
        ic_arg0 = 1;
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        ctx.icregs.extraArgs = 2;
        INVOKE_IC(SpreadCall);
        POPN(3);
        PUSH(StackVal(Value::fromRawBits(ic_ret)));
        END_OP(SpreadCall);
      }

      CASE(SpreadSuperCall)
      CASE(SpreadNew) {
        static_assert(JSOpLength_SpreadSuperCall == JSOpLength_SpreadNew);
        ic_arg0 = 1;
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        ctx.icregs.extraArgs = 3;
        INVOKE_IC(SpreadCall);
        POPN(4);
        PUSH(StackVal(Value::fromRawBits(ic_ret)));
        END_OP(SpreadSuperCall);
      }

      CASE(OptimizeSpreadCall) {
        IC_POP_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(OptimizeSpreadCall);
        IC_PUSH_RESULT();
        END_OP(OptimizeSpreadCall);
      }

      CASE(OptimizeGetIterator) {
        IC_POP_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(OptimizeGetIterator);
        IC_PUSH_RESULT();
        END_OP(OptimizeGetIterator);
      }

      CASE(ImplicitThis) {
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          PUSH_EXIT_FRAME();
          if (!ImplicitThisOperation(cx, obj0, name0, &state.res)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(state.res));
        state.res.setUndefined();
        END_OP(ImplicitThis);
      }

      CASE(CallSiteObj) {
        JSObject* cso = script->getObject(pc);
        MOZ_ASSERT(!cso->as<ArrayObject>().isExtensible());
        MOZ_ASSERT(cso->as<ArrayObject>().containsPure(
            ctx.frameMgr.cxForLocalUseOnly()->names().raw));
        PUSH(StackVal(ObjectValue(*cso)));
        END_OP(CallSiteObj);
      }

      CASE(IsConstructing) {
        PUSH(StackVal(MagicValue(JS_IS_CONSTRUCTING)));
        END_OP(IsConstructing);
      }

      CASE(SuperFun) {
        JSObject* superEnvFunc = &POP().asValue().toObject();
        JSObject* superFun = SuperFunOperation(superEnvFunc);
        PUSH(StackVal(ObjectOrNullValue(superFun)));
        END_OP(SuperFun);
      }

      CASE(CheckThis) {
        if (sp[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowUninitializedThis(cx));
          GOTO_ERROR();
        }
        END_OP(CheckThis);
      }

      CASE(CheckThisReinit) {
        if (!sp[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowInitializedThis(cx));
          GOTO_ERROR();
        }
        END_OP(CheckThisReinit);
      }

      CASE(Generator) {
        JSObject* generator;
        {
          PUSH_EXIT_FRAME();
          generator = CreateGeneratorFromFrame(cx, frame);
          if (!generator) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*generator)));
        END_OP(Generator);
      }

      CASE(InitialYield) {
        // gen => rval, gen, resumeKind
        ReservedRooted<JSObject*> obj0(&state.obj0,
                                       &sp[0].asValue().toObject());
        uint32_t frameSize = ctx.stack.frameSize(sp, frame);
        {
          PUSH_EXIT_FRAME();
          if (!NormalSuspend(cx, obj0, frame, frameSize, pc)) {
            GOTO_ERROR();
          }
        }
        frame->setReturnValue(sp[0].asValue());
        goto do_return;
      }

      CASE(Await)
      CASE(Yield) {
        // rval1, gen => rval2, gen, resumeKind
        ReservedRooted<JSObject*> obj0(&state.obj0,
                                       &POP().asValue().toObject());
        uint32_t frameSize = ctx.stack.frameSize(sp, frame);
        {
          PUSH_EXIT_FRAME();
          if (!NormalSuspend(cx, obj0, frame, frameSize, pc)) {
            GOTO_ERROR();
          }
        }
        frame->setReturnValue(sp[0].asValue());
        goto do_return;
      }

      CASE(FinalYieldRval) {
        // gen =>
        ReservedRooted<JSObject*> obj0(&state.obj0,
                                       &POP().asValue().toObject());
        {
          PUSH_EXIT_FRAME();
          if (!FinalSuspend(cx, obj0, pc)) {
            GOTO_ERROR();
          }
        }
        goto do_return;
      }

      CASE(IsGenClosing) {
        bool result = sp[0].asValue() == MagicValue(JS_GENERATOR_CLOSING);
        PUSH(StackVal(BooleanValue(result)));
        END_OP(IsGenClosing);
      }

      CASE(AsyncAwait) {
        // value, gen => promise
        JSObject* promise;
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());  // gen
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // value
          PUSH_EXIT_FRAME();
          promise = AsyncFunctionAwait(
              cx, obj0.as<AsyncFunctionGeneratorObject>(), value0);
          if (!promise) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*promise)));
        END_OP(AsyncAwait);
      }

      CASE(AsyncResolve) {
        // value, gen => promise
        JSObject* promise;
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());  // gen
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // value
          PUSH_EXIT_FRAME();
          promise = AsyncFunctionResolve(
              cx, obj0.as<AsyncFunctionGeneratorObject>(), value0);
          if (!promise) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*promise)));
        END_OP(AsyncResolve);
      }

      CASE(AsyncReject) {
        // reason, gen => promise
        JSObject* promise;
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());  // gen
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // stack
          ReservedRooted<Value> value1(&state.value1,
                                       POP().asValue());  // reason
          PUSH_EXIT_FRAME();
          promise = AsyncFunctionReject(
              cx, obj0.as<AsyncFunctionGeneratorObject>(), value1, value0);
          if (!promise) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(ObjectValue(*promise)));
        END_OP(AsyncReject);
      }

      CASE(CanSkipAwait) {
        // value => value, can_skip
        bool result = false;
        {
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());
          PUSH_EXIT_FRAME();
          if (!CanSkipAwait(cx, value0, &result)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(BooleanValue(result)));
        END_OP(CanSkipAwait);
      }

      CASE(MaybeExtractAwaitValue) {
        // value, can_skip => value_or_resolved, can_skip
        {
          Value can_skip = POP().asValue();
          ReservedRooted<Value> value0(&state.value0,
                                       POP().asValue());  // value
          if (can_skip.toBoolean()) {
            PUSH_EXIT_FRAME();
            if (!ExtractAwaitValue(cx, value0, &value0)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(value0));
          PUSH(StackVal(can_skip));
        }
        END_OP(MaybeExtractAwaitValue);
      }

      CASE(ResumeKind) {
        GeneratorResumeKind resumeKind = ResumeKindFromPC(pc);
        PUSH(StackVal(Int32Value(int32_t(resumeKind))));
        END_OP(ResumeKind);
      }

      CASE(CheckResumeKind) {
        // rval, gen, resumeKind => rval
        {
          GeneratorResumeKind resumeKind =
              IntToResumeKind(POP().asValue().toInt32());
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         &POP().asValue().toObject());   // gen
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());  // rval
          if (resumeKind != GeneratorResumeKind::Next) {
            PUSH_EXIT_FRAME();
            MOZ_ALWAYS_FALSE(GeneratorThrowOrReturn(
                cx, frame, obj0.as<AbstractGeneratorObject>(), value0,
                resumeKind));
            GOTO_ERROR();
          }
        }
        END_OP(CheckResumeKind);
      }

      CASE(Resume) {
        Value gen = sp[2].asValue();
        Value* callerSP = reinterpret_cast<Value*>(sp);
        {
          ReservedRooted<Value> value0(&state.value0);
          ReservedRooted<JSObject*> obj0(&state.obj0, &gen.toObject());
          {
            PUSH_EXIT_FRAME();
            TRACE_PRINTF("Going to C++ interp for Resume\n");
            if (!InterpretResume(cx, obj0, callerSP, &value0)) {
              GOTO_ERROR();
            }
          }
          POPN(2);
          sp[0] = StackVal(value0);
        }
        END_OP(Resume);
      }

      CASE(JumpTarget) {
        int32_t icIndex = GET_INT32(pc);
        frame->interpreterICEntry() = icEntries + icIndex;
        COUNT_COVERAGE_PC(pc);
        END_OP(JumpTarget);
      }
      CASE(LoopHead) {
        int32_t icIndex = GET_INT32(pc);
        frame->interpreterICEntry() = icEntries + icIndex;
#ifndef __wasi__
        if (ctx.frameMgr.cxForLocalUseOnly()->hasAnyPendingInterrupt()) {
          PUSH_EXIT_FRAME();
          if (!InterruptCheck(cx)) {
            GOTO_ERROR();
          }
        }
#endif
        COUNT_COVERAGE_PC(pc);
        END_OP(LoopHead);
      }
      CASE(AfterYield) {
        int32_t icIndex = GET_INT32(pc);
        frame->interpreterICEntry() = icEntries + icIndex;
        if (script->isDebuggee()) {
          TRACE_PRINTF("doing DebugAfterYield\n");
          PUSH_EXIT_FRAME();
          if (DebugAPI::hasAnyBreakpointsOrStepMode(script) &&
              !HandleDebugTrap(cx, frame, pc)) {
            TRACE_PRINTF("HandleDebugTrap returned error\n");
            GOTO_ERROR();
          }
          if (!DebugAfterYield(cx, frame)) {
            TRACE_PRINTF("DebugAfterYield returned error\n");
            GOTO_ERROR();
          }
        }
        COUNT_COVERAGE_PC(pc);
        END_OP(AfterYield);
      }

      CASE(Goto) {
        ADVANCE(GET_JUMP_OFFSET(pc));
        PREDICT_NEXT(JumpTarget);
        PREDICT_NEXT(LoopHead);
        DISPATCH();
      }

      CASE(Coalesce) {
        if (!sp[0].asValue().isNullOrUndefined()) {
          ADVANCE(GET_JUMP_OFFSET(pc));
          DISPATCH();
        } else {
          END_OP(Coalesce);
        }
      }

      CASE(Case) {
        bool cond = POP().asValue().toBoolean();
        if (cond) {
          POP();
          ADVANCE(GET_JUMP_OFFSET(pc));
          DISPATCH();
        } else {
          END_OP(Case);
        }
      }

      CASE(Default) {
        POP();
        ADVANCE(GET_JUMP_OFFSET(pc));
        DISPATCH();
      }

      CASE(TableSwitch) {
        int32_t len = GET_JUMP_OFFSET(pc);
        int32_t low = GET_JUMP_OFFSET(pc + 1 * JUMP_OFFSET_LEN);
        int32_t high = GET_JUMP_OFFSET(pc + 2 * JUMP_OFFSET_LEN);
        Value v = POP().asValue();
        int32_t i = 0;
        if (v.isInt32()) {
          i = v.toInt32();
        } else if (!v.isDouble() ||
                   !mozilla::NumberEqualsInt32(v.toDouble(), &i)) {
          ADVANCE(len);
          DISPATCH();
        }

        i = uint32_t(i) - uint32_t(low);
#ifdef ENABLE_JS_PBL_WEVAL
        i = int32_t(
            weval_specialize_value(uint32_t(i), 0, uint32_t(high - low + 1)));
#endif
        if ((uint32_t(i) < uint32_t(high - low + 1))) {
          len = isd->tableSwitchCaseOffset(pc, uint32_t(i)) - (pc - entryPC);
#ifdef ENABLE_JS_PBL_WEVAL
          weval_assert_const32(len, __LINE__);
#endif
          ADVANCE(len);
          DISPATCH();
        }
        ADVANCE(len);
        DISPATCH();
      }

      CASE(Return) {
        frame->setReturnValue(POP().asValue());
        goto do_return;
      }

      CASE(GetRval) {
        PUSH(StackVal(frame->returnValue()));
        END_OP(GetRval);
      }

      CASE(SetRval) {
        frame->setReturnValue(POP().asValue());
        END_OP(SetRval);
      }

    do_return:
      CASE(RetRval) {
        bool ok = true;
        if (frame->isDebuggee() && !from_unwind) {
          TRACE_PRINTF("doing DebugEpilogueOnBaselineReturn\n");
          PUSH_EXIT_FRAME();
          ok = DebugEpilogueOnBaselineReturn(cx, frame, pc);
        }
        from_unwind = false;

        uint32_t argc = frame->numActualArgs();
        sp = ctx.stack.popFrame();

        // If FP is higher than the entry frame now, return; otherwise,
        // do an inline state update.
        if (!InlineCalls || ctx.stack.fp > entryFrame) {
          *ret = frame->returnValue();
          TRACE_PRINTF("ret = %" PRIx64 "\n", ret->asRawBits());
          return ok ? PBIResult::Ok : PBIResult::Error;
        } else {
          TRACE_PRINTF("Return fastpath\n");
          Value innerRet = frame->returnValue();
          TRACE_PRINTF("ret = %" PRIx64 "\n", innerRet.asRawBits());

          // Pop exit frame as well.
          sp = ctx.stack.popFrame();
          // Pop fake return address and descriptor.
          POPNNATIVE(2);

          // Set PC, frame, and current script.
          frame = reinterpret_cast<BaselineFrame*>(
              reinterpret_cast<uintptr_t>(ctx.stack.fp) -
              BaselineFrame::Size());
          TRACE_PRINTF(" sp -> %p, fp -> %p, frame -> %p\n", sp, ctx.stack.fp,
                       frame);
          ctx.frameMgr.switchToFrame(frame);
          ctx.frame = frame;
          icEntries = frame->icScript()->icEntries();
          pc = frame->interpreterPC();
          script.set(frame->script());
          entryPC = script->code();
          isd = script->immutableScriptData();

          // Adjust caller's stack to complete the call op that PC still points
          // to in that frame (pop args, push return value).
          JSOp op = JSOp(*pc);
          bool constructing = (op == JSOp::New || op == JSOp::NewContent ||
                               op == JSOp::SuperCall);
          // Fix-up return value; EnterJit would do this if we hadn't bypassed
          // it.
          if (constructing && innerRet.isPrimitive()) {
            innerRet = sp[argc + constructing].asValue();
            TRACE_PRINTF("updated ret = %" PRIx64 "\n", innerRet.asRawBits());
          }
          // Pop args -- this is 1 more than how many are pushed in the
          // `totalArgs` count during the call fastpath because it includes
          // the callee.
          POPN(argc + 2 + constructing);
          // Push return value.
          PUSH(StackVal(innerRet));

          if (!ok) {
            GOTO_ERROR();
          }

          // Advance past call instruction, and advance past IC.
          NEXT_IC();
          ADVANCE(JSOpLength_Call);

          DISPATCH();
        }
      }

      CASE(CheckReturn) {
        Value thisval = POP().asValue();
        // inlined version of frame->checkReturn(thisval, result)
        // (js/src/vm/Stack.cpp).
        HandleValue retVal = frame->returnValue();
        if (retVal.isObject()) {
          PUSH(StackVal(retVal));
        } else if (!retVal.isUndefined()) {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ReportValueError(cx, JSMSG_BAD_DERIVED_RETURN,
                                            JSDVG_IGNORE_STACK, retVal,
                                            nullptr));
          GOTO_ERROR();

        } else if (thisval.isMagic(JS_UNINITIALIZED_LEXICAL)) {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowUninitializedThis(cx));
          GOTO_ERROR();

        } else {
          PUSH(StackVal(thisval));
        }
        END_OP(CheckReturn);
      }

      CASE(Throw) {
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowOperation(cx, value0));
          GOTO_ERROR();
        }
        END_OP(Throw);
      }

      CASE(ThrowWithStack) {
        {
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          ReservedRooted<Value> value1(&state.value1, POP().asValue());
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowWithStackOperation(cx, value1, value0));
          GOTO_ERROR();
        }
        END_OP(ThrowWithStack);
      }

      CASE(ThrowMsg) {
        {
          PUSH_EXIT_FRAME();
          MOZ_ALWAYS_FALSE(ThrowMsgOperation(cx, GET_UINT8(pc)));
          GOTO_ERROR();
        }
        END_OP(ThrowMsg);
      }

      CASE(ThrowSetConst) {
        {
          PUSH_EXIT_FRAME();
          ReportRuntimeLexicalError(cx, JSMSG_BAD_CONST_ASSIGN, script, pc);
          GOTO_ERROR();
        }
        END_OP(ThrowSetConst);
      }

      CASE(Try)
      CASE(TryDestructuring) {
        static_assert(JSOpLength_Try == JSOpLength_TryDestructuring);
        END_OP(Try);
      }

      CASE(Exception) {
        {
          PUSH_EXIT_FRAME();
          if (!GetAndClearException(cx, &state.res)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(state.res));
        state.res.setUndefined();
        END_OP(Exception);
      }

      CASE(ExceptionAndStack) {
        {
          ReservedRooted<Value> value0(&state.value0);
          {
            PUSH_EXIT_FRAME();
            if (!cx.getCx()->getPendingExceptionStack(&value0)) {
              GOTO_ERROR();
            }
            if (!GetAndClearException(cx, &state.res)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(state.res));
          PUSH(StackVal(value0));
          state.res.setUndefined();
        }
        END_OP(ExceptionAndStack);
      }

      CASE(Finally) {
#ifndef __wasi__
        if (ctx.frameMgr.cxForLocalUseOnly()->hasAnyPendingInterrupt()) {
          PUSH_EXIT_FRAME();
          if (!InterruptCheck(cx)) {
            GOTO_ERROR();
          }
        }
#endif
        END_OP(Finally);
      }

      CASE(Uninitialized) {
        PUSH(StackVal(MagicValue(JS_UNINITIALIZED_LEXICAL)));
        END_OP(Uninitialized);
      }
      CASE(InitLexical) {
        uint32_t i = GET_LOCALNO(pc);
        frame->unaliasedLocal(i) = sp[0].asValue();
        END_OP(InitLexical);
      }

      CASE(InitAliasedLexical) {
        EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
        EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
        obj.setAliasedBinding(ec, sp[0].asValue());
        END_OP(InitAliasedLexical);
      }
      CASE(CheckLexical) {
        if (sp[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
          PUSH_EXIT_FRAME();
          ReportRuntimeLexicalError(cx, JSMSG_UNINITIALIZED_LEXICAL, script,
                                    pc);
          GOTO_ERROR();
        }
        END_OP(CheckLexical);
      }
      CASE(CheckAliasedLexical) {
        if (sp[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
          PUSH_EXIT_FRAME();
          ReportRuntimeLexicalError(cx, JSMSG_UNINITIALIZED_LEXICAL, script,
                                    pc);
          GOTO_ERROR();
        }
        END_OP(CheckAliasedLexical);
      }

      CASE(BindGName) {
        IC_SET_OBJ_ARG(
            0,
            &ctx.frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(BindName);
        IC_PUSH_RESULT();
        END_OP(BindGName);
      }
      CASE(BindName) {
        IC_SET_OBJ_ARG(0, frame->environmentChain());
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(BindName);
        IC_PUSH_RESULT();
        END_OP(BindName);
      }
      CASE(GetGName) {
        IC_SET_OBJ_ARG(
            0,
            &ctx.frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetName);
        IC_PUSH_RESULT();
        END_OP(GetGName);
      }
      CASE(GetName) {
        IC_SET_OBJ_ARG(0, frame->environmentChain());
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetName);
        IC_PUSH_RESULT();
        END_OP(GetName);
      }

      CASE(GetArg) {
        unsigned i = GET_ARGNO(pc);
        if (script->argsObjAliasesFormals()) {
          PUSH(StackVal(frame->argsObj().arg(i)));
        } else {
          PUSH(StackVal(frame->unaliasedFormal(i)));
        }
        END_OP(GetArg);
      }

      CASE(GetFrameArg) {
        uint32_t i = GET_ARGNO(pc);
        PUSH(StackVal(frame->unaliasedFormal(i, DONT_CHECK_ALIASING)));
        END_OP(GetFrameArg);
      }

      CASE(GetLocal) {
        uint32_t i = GET_LOCALNO(pc);
        TRACE_PRINTF(" -> local: %d\n", int(i));
        PUSH(StackVal(frame->unaliasedLocal(i)));
        END_OP(GetLocal);
      }

      CASE(ArgumentsLength) {
        PUSH(StackVal(Int32Value(frame->numActualArgs())));
        END_OP(ArgumentsLength);
      }

      CASE(GetActualArg) {
        MOZ_ASSERT(!script->needsArgsObj());
        uint32_t index = sp[0].asValue().toInt32();
        sp[0] = StackVal(frame->unaliasedActual(index));
        END_OP(GetActualArg);
      }

      CASE(GetAliasedVar)
      CASE(GetAliasedDebugVar) {
        static_assert(JSOpLength_GetAliasedVar ==
                      JSOpLength_GetAliasedDebugVar);
        EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
        EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
        PUSH(StackVal(obj.aliasedBinding(ec)));
        END_OP(GetAliasedVar);
      }

      CASE(GetImport) {
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          ReservedRooted<Value> value0(&state.value0);
          {
            PUSH_EXIT_FRAME();
            if (!GetImportOperation(cx, obj0, script, pc, &value0)) {
              GOTO_ERROR();
            }
          }
          PUSH(StackVal(value0));
        }
        END_OP(GetImport);
      }

      CASE(GetIntrinsic) {
        IC_ZERO_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(GetIntrinsic);
        IC_PUSH_RESULT();
        END_OP(GetIntrinsic);
      }

      CASE(Callee) {
        PUSH(StackVal(frame->calleev()));
        END_OP(Callee);
      }

      CASE(EnvCallee) {
        uint8_t numHops = GET_UINT8(pc);
        JSObject* env = &frame->environmentChain()->as<EnvironmentObject>();
        for (unsigned i = 0; i < numHops; i++) {
          env = &env->as<EnvironmentObject>().enclosingEnvironment();
        }
        PUSH(StackVal(ObjectValue(env->as<CallObject>().callee())));
        END_OP(EnvCallee);
      }

      CASE(SetProp)
      CASE(StrictSetProp)
      CASE(SetName)
      CASE(StrictSetName)
      CASE(SetGName)
      CASE(StrictSetGName) {
        static_assert(JSOpLength_SetProp == JSOpLength_StrictSetProp);
        static_assert(JSOpLength_SetProp == JSOpLength_SetName);
        static_assert(JSOpLength_SetProp == JSOpLength_StrictSetName);
        static_assert(JSOpLength_SetProp == JSOpLength_SetGName);
        static_assert(JSOpLength_SetProp == JSOpLength_StrictSetGName);
        IC_POP_ARG(1);
        IC_POP_ARG(0);
        IC_ZERO_ARG(2);
        PUSH(StackVal(ic_arg1));
        INVOKE_IC(SetProp);
        END_OP(SetProp);
      }

      CASE(InitProp)
      CASE(InitHiddenProp)
      CASE(InitLockedProp) {
        static_assert(JSOpLength_InitProp == JSOpLength_InitHiddenProp);
        static_assert(JSOpLength_InitProp == JSOpLength_InitLockedProp);
        IC_POP_ARG(1);
        IC_SET_ARG_FROM_STACK(0, 0);
        IC_ZERO_ARG(2);
        INVOKE_IC(SetProp);
        END_OP(InitProp);
      }
      CASE(InitGLexical) {
        IC_SET_ARG_FROM_STACK(1, 0);
        IC_SET_OBJ_ARG(
            0,
            &ctx.frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
        IC_ZERO_ARG(2);
        INVOKE_IC(SetProp);
        END_OP(InitGLexical);
      }

      CASE(SetArg) {
        unsigned i = GET_ARGNO(pc);
        if (script->argsObjAliasesFormals()) {
          frame->argsObj().setArg(i, sp[0].asValue());
        } else {
          frame->unaliasedFormal(i) = sp[0].asValue();
        }
        END_OP(SetArg);
      }

      CASE(SetLocal) {
        uint32_t i = GET_LOCALNO(pc);
        TRACE_PRINTF(" -> local: %d\n", int(i));
        frame->unaliasedLocal(i) = sp[0].asValue();
        END_OP(SetLocal);
      }

      CASE(SetAliasedVar) {
        EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
        EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
        MOZ_ASSERT(!IsUninitializedLexical(obj.aliasedBinding(ec)));
        obj.setAliasedBinding(ec, sp[0].asValue());
        END_OP(SetAliasedVar);
      }

      CASE(SetIntrinsic) {
        {
          ReservedRooted<Value> value0(&state.value0, sp[0].asValue());
          {
            PUSH_EXIT_FRAME();
            if (!SetIntrinsicOperation(cx, script, pc, value0)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(SetIntrinsic);
      }

      CASE(PushLexicalEnv) {
        {
          ReservedRooted<Scope*> scope0(&state.scope0, script->getScope(pc));
          {
            PUSH_EXIT_FRAME();
            if (!frame->pushLexicalEnvironment(cx, scope0.as<LexicalScope>())) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(PushLexicalEnv);
      }
      CASE(PopLexicalEnv) {
        if (frame->isDebuggee()) {
          TRACE_PRINTF("doing DebugLeaveThenPopLexicalEnv\n");
          PUSH_EXIT_FRAME();
          if (!DebugLeaveThenPopLexicalEnv(cx, frame, pc)) {
            GOTO_ERROR();
          }
        } else {
          frame->popOffEnvironmentChain<LexicalEnvironmentObject>();
        }
        END_OP(PopLexicalEnv);
      }
      CASE(DebugLeaveLexicalEnv) {
        if (frame->isDebuggee()) {
          TRACE_PRINTF("doing DebugLeaveLexicalEnv\n");
          PUSH_EXIT_FRAME();
          if (!DebugLeaveLexicalEnv(cx, frame, pc)) {
            GOTO_ERROR();
          }
        }
        END_OP(DebugLeaveLexicalEnv);
      }

      CASE(RecreateLexicalEnv) {
        {
          PUSH_EXIT_FRAME();
          if (frame->isDebuggee()) {
            TRACE_PRINTF("doing DebuggeeRecreateLexicalEnv\n");
            if (!DebuggeeRecreateLexicalEnv(cx, frame, pc)) {
              GOTO_ERROR();
            }
          } else {
            if (!frame->recreateLexicalEnvironment<false>(cx)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(RecreateLexicalEnv);
      }

      CASE(FreshenLexicalEnv) {
        {
          PUSH_EXIT_FRAME();
          if (frame->isDebuggee()) {
            TRACE_PRINTF("doing DebuggeeFreshenLexicalEnv\n");
            if (!DebuggeeFreshenLexicalEnv(cx, frame, pc)) {
              GOTO_ERROR();
            }
          } else {
            if (!frame->freshenLexicalEnvironment<false>(cx)) {
              GOTO_ERROR();
            }
          }
        }
        END_OP(FreshenLexicalEnv);
      }
      CASE(PushClassBodyEnv) {
        {
          ReservedRooted<Scope*> scope0(&state.scope0, script->getScope(pc));
          PUSH_EXIT_FRAME();
          if (!frame->pushClassBodyEnvironment(cx,
                                               scope0.as<ClassBodyScope>())) {
            GOTO_ERROR();
          }
        }
        END_OP(PushClassBodyEnv);
      }
      CASE(PushVarEnv) {
        {
          ReservedRooted<Scope*> scope0(&state.scope0, script->getScope(pc));
          PUSH_EXIT_FRAME();
          if (!frame->pushVarEnvironment(cx, scope0)) {
            GOTO_ERROR();
          }
        }
        END_OP(PushVarEnv);
      }
      CASE(EnterWith) {
        {
          ReservedRooted<Scope*> scope0(&state.scope0, script->getScope(pc));
          ReservedRooted<Value> value0(&state.value0, POP().asValue());
          PUSH_EXIT_FRAME();
          if (!EnterWithOperation(cx, frame, value0, scope0.as<WithScope>())) {
            GOTO_ERROR();
          }
        }
        END_OP(EnterWith);
      }
      CASE(LeaveWith) {
        frame->popOffEnvironmentChain<WithEnvironmentObject>();
        END_OP(LeaveWith);
      }
      CASE(BindVar) {
        JSObject* varObj;
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          PUSH_EXIT_FRAME();
          varObj = BindVarOperation(cx, obj0);
        }
        PUSH(StackVal(ObjectValue(*varObj)));
        END_OP(BindVar);
      }

      CASE(GlobalOrEvalDeclInstantiation) {
        GCThingIndex lastFun = GET_GCTHING_INDEX(pc);
        {
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          PUSH_EXIT_FRAME();
          if (!GlobalOrEvalDeclInstantiation(cx, obj0, script, lastFun)) {
            GOTO_ERROR();
          }
        }
        END_OP(GlobalOrEvalDeclInstantiation);
      }

      CASE(DelName) {
        {
          ReservedRooted<PropertyName*> name0(&state.name0,
                                              script->getName(pc));
          ReservedRooted<JSObject*> obj0(&state.obj0,
                                         frame->environmentChain());
          PUSH_EXIT_FRAME();
          if (!DeleteNameOperation(cx, name0, obj0, &state.res)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(state.res));
        state.res.setUndefined();
        END_OP(DelName);
      }

      CASE(Arguments) {
        {
          PUSH_EXIT_FRAME();
          if (!NewArgumentsObject(cx, frame, &state.res)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(state.res));
        state.res.setUndefined();
        END_OP(Arguments);
      }

      CASE(Rest) {
        IC_ZERO_ARG(0);
        IC_ZERO_ARG(1);
        IC_ZERO_ARG(2);
        INVOKE_IC(Rest);
        IC_PUSH_RESULT();
        END_OP(Rest);
      }

      CASE(FunctionThis) {
        {
          PUSH_EXIT_FRAME();
          if (!js::GetFunctionThis(cx, frame, &state.res)) {
            GOTO_ERROR();
          }
        }
        PUSH(StackVal(state.res));
        state.res.setUndefined();
        END_OP(FunctionThis);
      }

      CASE(Pop) {
        POP();
        END_OP(Pop);
      }
      CASE(PopN) {
        uint32_t n = GET_UINT16(pc);
        POPN(n);
        END_OP(PopN);
      }
      CASE(Dup) {
        StackVal value = sp[0];
        PUSH(value);
        END_OP(Dup);
      }
      CASE(Dup2) {
        StackVal value1 = sp[0];
        StackVal value2 = sp[1];
        PUSH(value2);
        PUSH(value1);
        END_OP(Dup2);
      }
      CASE(DupAt) {
        unsigned i = GET_UINT24(pc);
        StackVal value = sp[i];
        PUSH(value);
        END_OP(DupAt);
      }
      CASE(Swap) {
        std::swap(sp[0], sp[1]);
        END_OP(Swap);
      }
      CASE(Pick) {
        unsigned i = GET_UINT8(pc);
        StackVal tmp = sp[i];
        memmove(&sp[1], &sp[0], sizeof(StackVal) * i);
        sp[0] = tmp;
        END_OP(Pick);
      }
      CASE(Unpick) {
        unsigned i = GET_UINT8(pc);
        StackVal tmp = sp[0];
        memmove(&sp[0], &sp[1], sizeof(StackVal) * i);
        sp[i] = tmp;
        END_OP(Unpick);
      }
      CASE(DebugCheckSelfHosted) {
        HandleValue val = Stack::handle(&sp[0]);
        {
          PUSH_EXIT_FRAME();
          if (!Debug_CheckSelfHosted(cx, val)) {
            GOTO_ERROR();
          }
        }
        END_OP(DebugCheckSelfHosted);
      }
      CASE(Lineno) { END_OP(Lineno); }
      CASE(NopDestructuring) { END_OP(NopDestructuring); }
      CASE(ForceInterpreter) { END_OP(ForceInterpreter); }
      CASE(Debugger) {
        {
          PUSH_EXIT_FRAME();
          if (!OnDebuggerStatement(cx, frame)) {
            GOTO_ERROR();
          }
        }
        END_OP(Debugger);
      }

    label_default:
#ifdef __wasi__
    default:
#endif
      MOZ_CRASH("Bad opcode");
    }
  }

error:
  TRACE_PRINTF("HandleException: frame %p\n", frame);
  {
    ResumeFromException rfe;
    {
      PUSH_EXIT_FRAME();
      HandleException(&rfe);
    }

    switch (rfe.kind) {
      case ExceptionResumeKind::EntryFrame:
        TRACE_PRINTF(" -> Return from entry frame\n");
        frame->setReturnValue(MagicValue(JS_ION_ERROR));
        ctx.stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        ctx.stack.unwindingSP = reinterpret_cast<StackVal*>(rfe.stackPointer);
        goto unwind_error;
      case ExceptionResumeKind::Catch:
        pc = frame->interpreterPC();
        ctx.stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        ctx.stack.unwindingSP = reinterpret_cast<StackVal*>(rfe.stackPointer);
        TRACE_PRINTF(" -> catch to pc %p\n", pc);
        goto unwind;
      case ExceptionResumeKind::Finally:
        pc = frame->interpreterPC();
        ctx.stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        sp = reinterpret_cast<StackVal*>(rfe.stackPointer);
        TRACE_PRINTF(" -> finally to pc %p\n", pc);
        PUSH(StackVal(rfe.exception));
        PUSH(StackVal(rfe.exceptionStack));
        PUSH(StackVal(BooleanValue(true)));
        ctx.stack.unwindingSP = sp;
        goto unwind;
      case ExceptionResumeKind::ForcedReturnBaseline:
        pc = frame->interpreterPC();
        ctx.stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        ctx.stack.unwindingSP = reinterpret_cast<StackVal*>(rfe.stackPointer);
        TRACE_PRINTF(" -> forced return\n");
        goto unwind_ret;
      case ExceptionResumeKind::ForcedReturnIon:
        MOZ_CRASH(
            "Unexpected ForcedReturnIon exception-resume kind in Portable "
            "Baseline");
      case ExceptionResumeKind::Bailout:
        MOZ_CRASH(
            "Unexpected Bailout exception-resume kind in Portable Baseline");
      case ExceptionResumeKind::Wasm:
        MOZ_CRASH("Unexpected Wasm exception-resume kind in Portable Baseline");
      case ExceptionResumeKind::WasmCatch:
        MOZ_CRASH(
            "Unexpected WasmCatch exception-resume kind in Portable "
            "Baseline");
    }
  }

  DISPATCH();

ic_fail:
  RESTART(ic_result);
  switch (ic_result) {
    case PBIResult::Ok:
      MOZ_CRASH("Unreachable: ic_result must be an error if we reach ic_fail");
    case PBIResult::Error:
      goto error;
    case PBIResult::Unwind:
      goto unwind;
    case PBIResult::UnwindError:
      goto unwind_error;
    case PBIResult::UnwindRet:
      goto unwind_ret;
  }

unwind:
  TRACE_PRINTF("unwind: fp = %p entryFrame = %p\n", ctx.stack.fp, entryFrame);
  if (reinterpret_cast<uintptr_t>(ctx.stack.fp) >
      reinterpret_cast<uintptr_t>(entryFrame) + BaselineFrame::Size()) {
    TRACE_PRINTF(" -> returning\n");
    return PBIResult::Unwind;
  }
  sp = ctx.stack.unwindingSP;
  frame = reinterpret_cast<BaselineFrame*>(
      reinterpret_cast<uintptr_t>(ctx.stack.fp) - BaselineFrame::Size());
  TRACE_PRINTF(" -> setting sp to %p, frame to %p\n", sp, frame);
  ctx.frameMgr.switchToFrame(frame);
  ctx.frame = frame;
  icEntries = frame->icScript()->icEntries();
  pc = frame->interpreterPC();
  script.set(frame->script());
  DISPATCH();
unwind_error:
  TRACE_PRINTF("unwind_error: fp = %p entryFrame = %p\n", ctx.stack.fp,
               entryFrame);
  if (reinterpret_cast<uintptr_t>(ctx.stack.fp) >
      reinterpret_cast<uintptr_t>(entryFrame) + BaselineFrame::Size()) {
    return PBIResult::UnwindError;
  }
  if (reinterpret_cast<uintptr_t>(ctx.stack.fp) ==
      reinterpret_cast<uintptr_t>(entryFrame) + BaselineFrame::Size()) {
    return PBIResult::Error;
  }
  sp = ctx.stack.unwindingSP;
  frame = reinterpret_cast<BaselineFrame*>(
      reinterpret_cast<uintptr_t>(ctx.stack.fp) - BaselineFrame::Size());
  TRACE_PRINTF(" -> setting sp to %p, frame to %p\n", sp, frame);
  ctx.frameMgr.switchToFrame(frame);
  ctx.frame = frame;
  icEntries = frame->icScript()->icEntries();
  pc = frame->interpreterPC();
  script.set(frame->script());
  goto error;
unwind_ret:
  TRACE_PRINTF("unwind_ret: fp = %p entryFrame = %p\n", ctx.stack.fp,
               entryFrame);
  if (reinterpret_cast<uintptr_t>(ctx.stack.fp) >
      reinterpret_cast<uintptr_t>(entryFrame) + BaselineFrame::Size()) {
    return PBIResult::UnwindRet;
  }
  if (reinterpret_cast<uintptr_t>(ctx.stack.fp) ==
      reinterpret_cast<uintptr_t>(entryFrame) + BaselineFrame::Size()) {
    *ret = frame->returnValue();
    return PBIResult::Ok;
  }
  sp = ctx.stack.unwindingSP;
  frame = reinterpret_cast<BaselineFrame*>(
      reinterpret_cast<uintptr_t>(ctx.stack.fp) - BaselineFrame::Size());
  TRACE_PRINTF(" -> setting sp to %p, frame to %p\n", sp, frame);
  ctx.frameMgr.switchToFrame(frame);
  ctx.frame = frame;
  icEntries = frame->icScript()->icEntries();
  pc = frame->interpreterPC();
  script.set(frame->script());
  from_unwind = true;
  goto do_return;

#ifndef __wasi__
debug: {
  TRACE_PRINTF("hit debug point\n");
  PUSH_EXIT_FRAME();
  if (!HandleDebugTrap(cx, frame, pc)) {
    TRACE_PRINTF("HandleDebugTrap returned error\n");
    goto error;
  }
  pc = frame->interpreterPC();
  TRACE_PRINTF("HandleDebugTrap done\n");
}
  goto dispatch;
#endif
}

/*
 * -----------------------------------------------
 * Entry point
 * -----------------------------------------------
 */

bool PortableBaselineTrampoline(JSContext* cx, size_t argc, Value* argv,
                                size_t numFormals, size_t numActuals,
                                CalleeToken calleeToken, JSObject* envChain,
                                Value* result) {
  State state(cx);
  Stack stack(cx->portableBaselineStack());
  StackVal* sp = stack.top;

  TRACE_PRINTF("Trampoline: calleeToken %p env %p\n", calleeToken, envChain);

  // Expected stack frame:
  // - argN
  // - ...
  // - arg1
  // - this
  // - calleeToken
  // - descriptor
  // - "return address" (nullptr for top frame)

  // `argc` is the number of args *including* `this` (`N + 1`
  // above). `numFormals` is the minimum `N`; if less, we need to push
  // `UndefinedValue`s above. We need to pass an argc (including
  // `this`) accoundint for the extra undefs in the descriptor's argc.
  //
  // If constructing, there is an additional `newTarget` at the end.
  //
  // Note that `callee`, which is in the stack signature for a `Call`
  // JSOp, does *not* appear in this count: it is separately passed in
  // the `calleeToken`.

  bool constructing = CalleeTokenIsConstructing(calleeToken);
  size_t numCalleeActuals = std::max(numActuals, numFormals);
  size_t numUndefs = numCalleeActuals - numActuals;

  // N.B.: we already checked the stack in
  // PortableBaselineInterpreterStackCheck; we don't do it here
  // because we can't push an exit frame if we don't have an entry
  // frame, and we need a full activation to produce the backtrace
  // from ReportOverRecursed.

  if (constructing) {
    PUSH(StackVal(argv[argc]));
  }
  for (size_t i = 0; i < numUndefs; i++) {
    PUSH(StackVal(UndefinedValue()));
  }
  for (size_t i = 0; i < argc; i++) {
    PUSH(StackVal(argv[argc - 1 - i]));
  }
  PUSHNATIVE(StackValNative(calleeToken));
  PUSHNATIVE(StackValNative(
      MakeFrameDescriptorForJitCall(FrameType::CppToJSJit, numActuals)));

  JSScript* script = ScriptFromCalleeToken(calleeToken);
  jsbytecode* pc = script->code();
  ImmutableScriptData* isd = script->immutableScriptData();
  PBIResult ret;
  INVOKE_PBI(ret, script, (PortableBaselineInterpret<false, true, kHybridICs>),
             cx, state, stack, sp, envChain, result, pc, isd, nullptr, nullptr,
             nullptr, PBIResult::Ok);
  switch (ret) {
    case PBIResult::Ok:
    case PBIResult::UnwindRet:
      TRACE_PRINTF("PBI returned Ok/UnwindRet with result %" PRIx64 "\n",
                   result->asRawBits());
      break;
    case PBIResult::Error:
    case PBIResult::UnwindError:
      TRACE_PRINTF("PBI returned Error/UnwindError\n");
      return false;
    case PBIResult::Unwind:
      MOZ_CRASH("Should not unwind out of top / entry frame");
  }

  return true;
}

MethodStatus CanEnterPortableBaselineInterpreter(JSContext* cx,
                                                 RunState& state) {
  if (!JitOptions.portableBaselineInterpreter) {
    return MethodStatus::Method_CantCompile;
  }
  if (state.script()->hasJitScript()) {
    return MethodStatus::Method_Compiled;
  }
  if (state.script()->hasForceInterpreterOp()) {
    return MethodStatus::Method_CantCompile;
  }
  if (cx->runtime()->geckoProfiler().enabled()) {
    return MethodStatus::Method_CantCompile;
  }

  if (state.isInvoke()) {
    InvokeState& invoke = *state.asInvoke();
    if (TooManyActualArguments(invoke.args().length())) {
      return MethodStatus::Method_CantCompile;
    }
  } else {
    if (state.asExecute()->isDebuggerEval()) {
      return MethodStatus::Method_CantCompile;
    }
  }
  if (state.script()->getWarmUpCount() <=
      JitOptions.portableBaselineInterpreterWarmUpThreshold) {
    return MethodStatus::Method_Skipped;
  }
  if (!cx->zone()->ensureJitZoneExists(cx)) {
    return MethodStatus::Method_Error;
  }

  AutoKeepJitScripts keepJitScript(cx);
  if (!state.script()->ensureHasJitScript(cx, keepJitScript)) {
    return MethodStatus::Method_Error;
  }
  state.script()->updateJitCodeRaw(cx->runtime());
  return MethodStatus::Method_Compiled;
}

bool PortablebaselineInterpreterStackCheck(JSContext* cx, RunState& state,
                                           size_t numActualArgs) {
  auto& pbs = cx->portableBaselineStack();
  StackVal* base = reinterpret_cast<StackVal*>(pbs.base);
  StackVal* top = reinterpret_cast<StackVal*>(pbs.top);
  ssize_t margin = kStackMargin / sizeof(StackVal);
  ssize_t needed = numActualArgs + state.script()->nslots() + margin;
  return (top - base) >= needed;
}

#ifdef ENABLE_JS_PBL_WEVAL

// IDs for interpreter bodies that we weval, so that we can stably
// associate collected request bodies with interpreters even when
// SpiderMonkey is relinked and actual function pointer values may
// change.
static const uint32_t WEVAL_JSOP_ID = 1;
static const uint32_t WEVAL_IC_ID = 2;

WEVAL_DEFINE_TARGET(1, (PortableBaselineInterpret<false, false, true>));
WEVAL_DEFINE_TARGET(2, (ICInterpretOps<true>));

void EnqueueScriptSpecialization(JSScript* script) {
  Weval& weval = script->weval();
  if (!weval.req) {
    using weval::Runtime;
    using weval::SpecializeMemory;

    jsbytecode* pc = script->code();
    uint32_t pc_len = script->length();
    ImmutableScriptData* isd = script->immutableScriptData();
    uint32_t isd_len = isd->immutableData().Length();

    weval.req = weval::weval(
        reinterpret_cast<PBIFunc*>(&weval.func),
        &PortableBaselineInterpret<false, false, kHybridICs>, WEVAL_JSOP_ID,
        Runtime<JSContext*>(), Runtime<State&>(), Runtime<Stack&>(),
        Runtime<StackVal*>(), Runtime<JSObject*>(), Runtime<Value*>(),
        SpecializeMemory<jsbytecode*>(pc, pc_len),
        SpecializeMemory<ImmutableScriptData*>(isd, isd_len),
        Runtime<jsbytecode*>(), Runtime<BaselineFrame*>(), Runtime<StackVal*>(),
        Runtime<PBIResult>());
  }
}

void EnqueueICStubSpecialization(CacheIRStubInfo* stubInfo) {
  Weval& weval = stubInfo->weval();
  if (!weval.req) {
    using weval::Runtime;
    using weval::SpecializeMemory;

    // StubInfo length: do not include the `Weval` object pointer, as
    // it is nondeterministic.
    uint32_t len = sizeof(CacheIRStubInfo) - sizeof(void*);

    weval.req = weval::weval(reinterpret_cast<ICStubFunc*>(&weval.func),
                             &ICInterpretOps<true>, WEVAL_IC_ID,
                             Runtime<ICCtx&>(), Runtime<ICStub*>(),
                             SpecializeMemory<CacheIRStubInfo*>(stubInfo, len),
                             SpecializeMemory<const uint8_t*>(
                                 stubInfo->code(), stubInfo->codeLength()));
  }
}

#endif  // ENABLE_JS_PBL_WEVAL

}  // namespace pbl
}  // namespace js
