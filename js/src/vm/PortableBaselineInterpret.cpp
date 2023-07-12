/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*
 * JavaScript "portable baseline interpreter": an interpreter that is
 * capable of running ICs, but without any native code.
 */

#include "vm/PortableBaselineInterpret.h"

#include "mozilla/Maybe.h"

#include "jsapi.h"

#include "builtin/DataViewObject.h"
#include "builtin/MapObject.h"
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
#include "vm/AsyncFunction.h"
#include "vm/AsyncIteration.h"
#include "vm/EnvironmentObject.h"
#include "vm/GeneratorObject.h"
#include "vm/Interpreter.h"
#include "vm/Iteration.h"
#include "vm/JitActivation.h"
#include "vm/JSScript.h"
#include "vm/Opcodes.h"
#include "vm/PlainObject.h"
#include "vm/Shape.h"
#include "vm/Timing.h"

#include "jit/BaselineFrame-inl.h"
#include "jit/JitScript-inl.h"
#include "vm/EnvironmentObject-inl.h"
#include "vm/Interpreter-inl.h"
#include "vm/JSScript-inl.h"

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

using namespace js;
using namespace js::jit;

struct StackVal {
  uint64_t value;

  explicit StackVal(uint64_t v) : value(v) {}
  explicit StackVal(Value v) : value(v.asRawBits()) {}
  explicit StackVal(void* v) : value(reinterpret_cast<uint64_t>(v)) {}

  uint64_t asUInt64() const { return value; }
  Value asValue() const { return Value::fromRawBits(value); }
  void* asVoidPtr() const { return reinterpret_cast<void*>(value); }
  CalleeToken asCalleeToken() const {
    return reinterpret_cast<CalleeToken>(value);
  }
};

struct Stack {
  StackVal* sp;
  StackVal* fp;
  StackVal* base;
  StackVal* top;
  Stack* parentStack;

  Stack(PortableBaselineStack& pbs)
      : sp(reinterpret_cast<StackVal*>(pbs.top)),
        fp(sp),
        base(reinterpret_cast<StackVal*>(pbs.base)),
        top(reinterpret_cast<StackVal*>(pbs.top)),
        parentStack(nullptr) {}

  Stack(Stack& other) {
    *this = other;
    parentStack = &other;
  }

  ~Stack() {
    if (parentStack) {
      *parentStack = *this;
    }
  }

  Stack& operator=(const Stack& other) {
    sp = other.sp;
    fp = other.fp;
    base = other.base;
    top = other.top;
    return *this;
  }

  inline bool check(size_t size) {
    return reinterpret_cast<uintptr_t>(base) + size <=
           reinterpret_cast<uintptr_t>(sp);
  }

  [[nodiscard]] StackVal* allocate(size_t size) {
    if (!check(size)) {
      return nullptr;
    }
    sp = reinterpret_cast<StackVal*>(reinterpret_cast<uintptr_t>(sp) - size);
    return sp;
  }

  [[nodiscard]] bool push(StackVal v) {
    if (sp == base) {
      return false;
    }
    *--sp = v;
    return true;
  }
  void pushUnchecked(StackVal v) { *--sp = v; }
  StackVal pop() {
    MOZ_ASSERT(sp + 1 <= top);
    MOZ_ASSERT(sp < fp);
    return *sp++;
  }
  void popn(size_t len) {
    MOZ_ASSERT(sp + len <= top);
    sp += len;
    MOZ_ASSERT(sp <= fp);
  }

  StackVal* cur() const { return sp; }
  void restore(StackVal* s) { sp = s; }

  uint32_t frameSize(BaselineFrame* curFrame) const {
    return sizeof(StackVal) * (reinterpret_cast<StackVal*>(fp) - cur());
  }

  [[nodiscard]] BaselineFrame* pushFrame(JSContext* cx, JSObject* envChain) {
    TRACE_PRINTF("pushFrame: sp = %p fp = %p\n", sp, fp);
    if (!push(StackVal(fp))) {
      return nullptr;
    }
    fp = cur();
    TRACE_PRINTF("pushFrame: new fp = %p\n", fp);

    BaselineFrame* frame =
        reinterpret_cast<BaselineFrame*>(allocate(BaselineFrame::Size()));
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

  void popFrame(JSContext* cx) {
    StackVal* newTOS = fp + 1;
    fp = reinterpret_cast<StackVal*>(fp->asVoidPtr());
    MOZ_ASSERT(fp);
    TRACE_PRINTF("popFrame: fp = %p\n", fp);
    restore(newTOS);
  }

  void setFrameSize(BaselineFrame* prevFrame) {
#ifdef DEBUG
    MOZ_ASSERT(fp != nullptr);
    uintptr_t frameSize =
        reinterpret_cast<uintptr_t>(fp) - reinterpret_cast<uintptr_t>(cur());
    MOZ_ASSERT(reinterpret_cast<uintptr_t>(fp) >=
               reinterpret_cast<uintptr_t>(cur()));
    TRACE_PRINTF("pushExitFrame: fp = %p cur() = %p -> frameSize = %d\n", fp,
                 cur(), int(frameSize));
    MOZ_ASSERT(frameSize >= BaselineFrame::Size());
    prevFrame->setDebugFrameSize(frameSize);
#endif
  }

  [[nodiscard]] StackVal* pushExitFrame(BaselineFrame* prevFrame) {
    uint8_t* prevFP =
        reinterpret_cast<uint8_t*>(prevFrame) + BaselineFrame::Size();
    MOZ_ASSERT(reinterpret_cast<StackVal*>(prevFP) == fp);
    setFrameSize(prevFrame);

    if (!check(sizeof(StackVal) * 4)) {
      return nullptr;
    }

    pushUnchecked(
        StackVal(MakeFrameDescriptorForJitCall(FrameType::BaselineJS, 0)));
    pushUnchecked(StackVal(nullptr));  // fake return address.
    pushUnchecked(StackVal(prevFP));
    StackVal* exitFP = cur();
    fp = exitFP;
    TRACE_PRINTF(" -> fp = %p\n", fp);
    pushUnchecked(StackVal(uint64_t(ExitFrameType::Bare)));
    return exitFP;
  }

  void popExitFrame(StackVal* fp) {
    StackVal* prevFP = reinterpret_cast<StackVal*>(fp->asVoidPtr());
    MOZ_ASSERT(prevFP);
    restore(fp + 3);
    this->fp = prevFP;
    TRACE_PRINTF("popExitFrame: fp -> %p sp -> %p\n", fp, sp);
  }

  BaselineFrame* frameFromFP() {
    return reinterpret_cast<BaselineFrame*>(reinterpret_cast<uintptr_t>(fp) -
                                            BaselineFrame::Size());
  }

  StackVal& operator[](size_t index) { return sp[index]; }
};

struct ICRegs {
  CacheIRReader cacheIRReader;
  static const int kMaxICVals = 16;
  uint64_t icVals[kMaxICVals];
  uint64_t icResult;
  int extraArgs;
  bool spreadCall;

  ICRegs() : cacheIRReader(nullptr, nullptr) {}
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

  State(JSContext* cx)
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
  VMFrame(VMFrameManager& mgr, Stack& stack_, jsbytecode* pc)
      : cx(mgr.cx), stack(stack_) {
    mgr.frame->interpreterPC() = pc;
    exitFP = stack.pushExitFrame(mgr.frame);
    if (!exitFP) {
      return;
    }
    cx->activation()->asJit()->setJSExitFP(reinterpret_cast<uint8_t*>(exitFP));
    prevSavedStack = cx->portableBaselineStack().top;
    cx->portableBaselineStack().top = reinterpret_cast<void*>(stack.sp);
  }

  ~VMFrame() {
    stack.popExitFrame(exitFP);
    cx->portableBaselineStack().top = prevSavedStack;
  }

  JSContext* getCx() const { return cx; }
  operator JSContext*() const { return cx; }

  bool success() const { return exitFP != nullptr; }
};

static EnvironmentObject& getEnvironmentFromCoordinate(
    BaselineFrame* frame, EnvironmentCoordinate ec) {
  JSObject* env = &frame->environmentChain()->as<EnvironmentObject>();
  for (unsigned i = ec.hops(); i; i--) {
    env = &env->as<EnvironmentObject>().enclosingEnvironment();
  }
  return env->as<EnvironmentObject>();
}

enum class PBIResult {
  Ok,
  Error,
  Unwind,
  UnwindError,
  UnwindRet,
};

static PBIResult PortableBaselineInterpret(JSContext* cx_, State& state,
                                           Stack& parentStack,
                                           JSObject* envChain, Value* ret);

#define TRY(x)               \
  if (!(x)) {                \
    return PBIResult::Error; \
  }

#define PUSH_UNCHECKED(val) stack.pushUnchecked(val)

#define PUSH_EXIT_FRAME_OR_RET(value) \
  VMFrame cx(frameMgr, stack, pc);    \
  if (!cx.success()) {                \
    return value;                     \
  }

enum class ICInterpretOpResult {
  NextIC,
  Return,
  Error,
  Unwind,
  UnwindError,
  UnwindRet,
};

#define PUSH_IC_FRAME() PUSH_EXIT_FRAME_OR_RET(ICInterpretOpResult::Error);

static ICInterpretOpResult MOZ_ALWAYS_INLINE ICInterpretOps(
    BaselineFrame* frame, VMFrameManager& frameMgr, State& state,
    ICRegs& icregs, Stack& stack, ICCacheIRStub* cstub, jsbytecode* pc) {
#define CACHEOP_CASE(name)                                                   \
  cacheop_##name                                                             \
      : TRACE_PRINTF("cacheop (frame %p pc %p stub %p): " #name "\n", frame, \
                     pc, cstub);
#define CACHEOP_CASE_FALLTHROUGH(name) CACHEOP_CASE(name)

  static const void* const addresses[long(CacheOp::NumOpcodes)] = {
#define OP(name, ...) &&cacheop_##name,
      CACHE_IR_OPS(OP)
#undef OP
  };

#define DISPATCH_CACHEOP()                 \
  cacheop = icregs.cacheIRReader.readOp(); \
  goto* addresses[long(cacheop)];

#define BOUNDSCHECK(resultId) \
  if (resultId.id() >= ICRegs::kMaxICVals) return ICInterpretOpResult::NextIC;

#define PREDICT_NEXT(name)                              \
  if (icregs.cacheIRReader.peekOp() == CacheOp::name) { \
    icregs.cacheIRReader.readOp();                      \
    goto cacheop_##name;                                \
  }

  CacheOp cacheop;

  DISPATCH_CACHEOP();

  CACHEOP_CASE(ReturnFromIC) {
    TRACE_PRINTF("stub successful!\n");
    return ICInterpretOpResult::Return;
  }

  CACHEOP_CASE(GuardToInt32) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    TRACE_PRINTF("GuardToInt32 (%d): icVal %" PRIx64 "\n", inputId.id(),
                 icregs.icVals[inputId.id()]);
    if (!v.isInt32()) {
      return ICInterpretOpResult::NextIC;
    }
    // N.B.: we don't need to unbox because the low 32 bits are
    // already the int32 itself, and we are careful when using
    // `Int32Operand`s to only use those bits.

    PREDICT_NEXT(GuardToInt32);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToObject) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    TRACE_PRINTF("GuardToObject: icVal %" PRIx64 "\n",
                 icregs.icVals[inputId.id()]);
    if (!v.isObject()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[inputId.id()] = reinterpret_cast<uint64_t>(&v.toObject());
    PREDICT_NEXT(GuardShape);
    PREDICT_NEXT(GuardSpecificFunction);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToString) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isString()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[inputId.id()] = reinterpret_cast<uint64_t>(v.toString());
    PREDICT_NEXT(GuardToString);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToSymbol) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isSymbol()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[inputId.id()] = reinterpret_cast<uint64_t>(v.toSymbol());
    PREDICT_NEXT(GuardSpecificSymbol);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToBigInt) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isBigInt()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[inputId.id()] = reinterpret_cast<uint64_t>(v.toBigInt());
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToBoolean) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isBoolean()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[inputId.id()] = v.toBoolean() ? 1 : 0;
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardIsNullOrUndefined) {
    ObjOperandId inputId = icregs.cacheIRReader.objOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isNullOrUndefined()) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardIsNull) {
    ObjOperandId inputId = icregs.cacheIRReader.objOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isNull()) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardIsUndefined) {
    ObjOperandId inputId = icregs.cacheIRReader.objOperandId();
    Value v = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (!v.isUndefined()) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardNonDoubleType) {
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    ValueType type = icregs.cacheIRReader.valueType();
    Value val = Value::fromRawBits(icregs.icVals[inputId.id()]);
    switch (type) {
      case ValueType::String:
        if (!val.isString()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::Symbol:
        if (!val.isSymbol()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::BigInt:
        if (!val.isBigInt()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::Int32:
        if (!val.isInt32()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::Boolean:
        if (!val.isBoolean()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::Undefined:
        if (!val.isUndefined()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case ValueType::Null:
        if (!val.isNull()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      default:
        MOZ_CRASH("Unexpected type");
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardShape) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t shapeOffset = icregs.cacheIRReader.stubOffset();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    uintptr_t expectedShape =
        cstub->stubInfo()->getStubRawWord(cstub, shapeOffset);
    if (reinterpret_cast<uintptr_t>(nobj->shape()) != expectedShape) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardProto) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t protoOffset = icregs.cacheIRReader.stubOffset();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    uintptr_t expectedProto =
        cstub->stubInfo()->getStubRawWord(cstub, protoOffset);
    if (reinterpret_cast<uintptr_t>(nobj->staticPrototype()) != expectedProto) {
      return ICInterpretOpResult::NextIC;
    }
    PREDICT_NEXT(LoadProto);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadProto) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    ObjOperandId resultId = icregs.cacheIRReader.objOperandId();
    BOUNDSCHECK(resultId);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    icregs.icVals[resultId.id()] =
        reinterpret_cast<uintptr_t>(nobj->staticPrototype());
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardSpecificFunction) {
    ObjOperandId funId = icregs.cacheIRReader.objOperandId();
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uint32_t nargsAndFlagsOffset = icregs.cacheIRReader.stubOffset();
    (void)nargsAndFlagsOffset;  // Unused.
    uintptr_t expected =
        cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
    if (expected != icregs.icVals[funId.id()]) {
      return ICInterpretOpResult::NextIC;
    }
    PREDICT_NEXT(LoadArgumentFixedSlot);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardSpecificObject) {
    ObjOperandId funId = icregs.cacheIRReader.objOperandId();
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t expected =
        cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
    if (expected != icregs.icVals[funId.id()]) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardSpecificAtom) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t expected =
        cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
    if (expected != icregs.icVals[strId.id()]) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardSpecificSymbol) {
    SymbolOperandId symId = icregs.cacheIRReader.symbolOperandId();
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t expected =
        cstub->stubInfo()->getStubRawWord(cstub, expectedOffset);
    if (expected != icregs.icVals[symId.id()]) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardSpecificInt32) {
    Int32OperandId int32Id = icregs.cacheIRReader.int32OperandId();
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uint32_t expected =
        cstub->stubInfo()->getStubRawInt32(cstub, expectedOffset);
    if (expected != uint32_t(icregs.icVals[int32Id.id()])) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardClass) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    GuardClassKind kind = icregs.cacheIRReader.guardClassKind();
    JSObject* object = reinterpret_cast<JSObject*>(icregs.icVals[objId.id()]);
    switch (kind) {
      case GuardClassKind::Array:
        if (object->getClass() != &ArrayObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::PlainObject:
        if (object->getClass() != &PlainObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::ArrayBuffer:
        if (object->getClass() != &ArrayBufferObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::SharedArrayBuffer:
        if (object->getClass() != &SharedArrayBufferObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::DataView:
        if (object->getClass() != &DataViewObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::MappedArguments:
        if (object->getClass() != &MappedArgumentsObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::UnmappedArguments:
        if (object->getClass() != &UnmappedArgumentsObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::WindowProxy:
        if (object->getClass() !=
            frameMgr.cxForLocalUseOnly()->runtime()->maybeWindowProxyClass()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::JSFunction:
        if (!object->is<JSFunction>()) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::Set:
        if (object->getClass() != &SetObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::Map:
        if (object->getClass() != &MapObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
      case GuardClassKind::BoundFunction:
        if (object->getClass() != &BoundFunctionObject::class_) {
          return ICInterpretOpResult::NextIC;
        }
        break;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(StoreFixedSlot) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t offsetOffset = icregs.cacheIRReader.stubOffset();
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    uintptr_t offset = cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    GCPtr<Value>* slot = reinterpret_cast<GCPtr<Value>*>(
        reinterpret_cast<uintptr_t>(nobj) + offset);
    Value val = Value::fromRawBits(icregs.icVals[valId.id()]);
    slot->set(val);
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(StoreDynamicSlot) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t offsetOffset = icregs.cacheIRReader.stubOffset();
    uint32_t offset = cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    HeapSlot* slots = nobj->getSlotsUnchecked();
    Value val = Value::fromRawBits(icregs.icVals[valId.id()]);
    size_t dynSlot = offset / sizeof(Value);
    size_t slot = dynSlot + nobj->numFixedSlots();
    slots[dynSlot].set(nobj, HeapSlot::Slot, slot, val);
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadObject) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t objOffset = icregs.cacheIRReader.stubOffset();
    intptr_t obj = cstub->stubInfo()->getStubRawWord(cstub, objOffset);
    BOUNDSCHECK(objId);
    icregs.icVals[objId.id()] = obj;
    PREDICT_NEXT(GuardShape);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadProtoObject) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t objOffset = icregs.cacheIRReader.stubOffset();
    intptr_t obj = cstub->stubInfo()->getStubRawWord(cstub, objOffset);
    ObjOperandId receiverObjId = icregs.cacheIRReader.objOperandId();
    (void)receiverObjId;
    BOUNDSCHECK(objId);
    icregs.icVals[objId.id()] = obj;
    PREDICT_NEXT(GuardShape);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadOperandResult) {
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    icregs.icResult = icregs.icVals[valId.id()];
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadValueResult) {
    uint32_t valOffset = icregs.cacheIRReader.stubOffset();
    icregs.icResult = cstub->stubInfo()->getStubRawInt64(cstub, valOffset);
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadObjectResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    icregs.icResult =
        ObjectValue(*reinterpret_cast<JSObject*>(icregs.icVals[objId.id()]))
            .asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadStringResult) {
    StringOperandId stringId = icregs.cacheIRReader.stringOperandId();
    icregs.icResult =
        StringValue(reinterpret_cast<JSString*>(icregs.icVals[stringId.id()]))
            .asRawBits();
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadConstantStringResult) {
    uint32_t offset = icregs.cacheIRReader.stubOffset();
    JSString* str = reinterpret_cast<JSString*>(
        cstub->stubInfo()->getStubRawWord(cstub, offset));
    icregs.icResult = StringValue(str).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadInt32Constant) {
    uint32_t valueOffset = icregs.cacheIRReader.stubOffset();
    Int32OperandId resultId = icregs.cacheIRReader.int32OperandId();
    uint32_t value = cstub->stubInfo()->getStubRawInt32(cstub, valueOffset);
    BOUNDSCHECK(resultId);
    icregs.icVals[resultId.id()] = value;
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadSymbolResult) {
    SymbolOperandId symbolId = icregs.cacheIRReader.symbolOperandId();
    icregs.icResult =
        SymbolValue(reinterpret_cast<JS::Symbol*>(icregs.icVals[symbolId.id()]))
            .asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadInt32Result) {
    Int32OperandId intId = icregs.cacheIRReader.int32OperandId();
    icregs.icResult = Int32Value(icregs.icVals[intId.id()]).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadBigIntResult) {
    BigIntOperandId bigintId = icregs.cacheIRReader.bigIntOperandId();
    icregs.icResult =
        BigIntValue(reinterpret_cast<JS::BigInt*>(icregs.icVals[bigintId.id()]))
            .asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadDoubleResult) {
    NumberOperandId numId = icregs.cacheIRReader.numberOperandId();
    Value val = Value::fromRawBits(icregs.icVals[numId.id()]);
    if (val.isInt32()) {
      val = DoubleValue(val.toInt32());
    }
    icregs.icResult = val.asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadBooleanResult) {
    icregs.icResult = BooleanValue(icregs.cacheIRReader.readBool()).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadFixedSlotResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t offsetOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t offset = cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    Value* slot =
        reinterpret_cast<Value*>(reinterpret_cast<uintptr_t>(nobj) + offset);
    TRACE_PRINTF(
        "LoadFixedSlotResult: obj %p offsetOffset %d offset %d slotPtr %p "
        "slot %" PRIx64 "\n",
        nobj, int(offsetOffset), int(offset), slot, slot->asRawBits());
    icregs.icResult = slot->asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadDynamicSlotResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t offsetOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t offset = cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    HeapSlot* slots = nobj->getSlotsUnchecked();
    icregs.icResult = slots[offset / sizeof(Value)].get().asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardDynamicSlotIsSpecificObject) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    ObjOperandId expectedId = icregs.cacheIRReader.objOperandId();
    uint32_t offsetOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t offset = cstub->stubInfo()->getStubRawInt32(cstub, offsetOffset);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    HeapSlot* slots = nobj->getSlotsUnchecked();
    uintptr_t actual = slots[offset / sizeof(Value)].get().asRawBits();
    uintptr_t expected = icregs.icVals[expectedId.id()];
    if (actual != expected) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadArgumentDynamicSlot) {
    ValOperandId resultId = icregs.cacheIRReader.valOperandId();
    BOUNDSCHECK(resultId);
    Int32OperandId argcId = icregs.cacheIRReader.int32OperandId();
    uint8_t slotIndex = icregs.cacheIRReader.readByte();
    int32_t argc = int32_t(icregs.icVals[argcId.id()]);
    Value val = stack[slotIndex + argc].asValue();
    icregs.icVals[resultId.id()] = val.asRawBits();
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadArgumentFixedSlot) {
    ValOperandId resultId = icregs.cacheIRReader.valOperandId();
    BOUNDSCHECK(resultId);
    uint8_t slotIndex = icregs.cacheIRReader.readByte();
    Value val = stack[slotIndex].asValue();
    TRACE_PRINTF(" -> slot %d: val %" PRIx64 "\n", int(slotIndex),
                 val.asRawBits());
    icregs.icVals[resultId.id()] = val.asRawBits();
    DISPATCH_CACHEOP();
  }

#define INT32_OP(name, op, extra_check)                           \
  CACHEOP_CASE(Int32##name##Result) {                             \
    Int32OperandId lhsId = icregs.cacheIRReader.int32OperandId(); \
    Int32OperandId rhsId = icregs.cacheIRReader.int32OperandId(); \
    int64_t lhs = int64_t(int32_t(icregs.icVals[lhsId.id()]));    \
    int64_t rhs = int64_t(int32_t(icregs.icVals[rhsId.id()]));    \
    extra_check;                                                  \
    int64_t result = lhs op rhs;                                  \
    if (result < INT32_MIN || result > INT32_MAX) {               \
      return ICInterpretOpResult::NextIC;                         \
    }                                                             \
    icregs.icResult = Int32Value(int32_t(result)).asRawBits();    \
    PREDICT_NEXT(ReturnFromIC);                                   \
    DISPATCH_CACHEOP();                                           \
  }

  INT32_OP(Add, +, {});
  INT32_OP(Sub, -, {});
  INT32_OP(Mul, *, {
    if (rhs * lhs == 0 && ((rhs < 0) ^ (lhs < 0))) {
      return ICInterpretOpResult::NextIC;
    }
  });
  INT32_OP(Div, /, {
    if (rhs == 0 || (lhs == INT32_MIN && rhs == -1)) {
      return ICInterpretOpResult::NextIC;
    }
    if (lhs == 0 && rhs < 0) {
      return ICInterpretOpResult::NextIC;
    }
    if (lhs % rhs != 0) {
      return ICInterpretOpResult::NextIC;
    }
  });
  INT32_OP(Mod, %, {
    if (rhs == 0 || (lhs == INT32_MIN && rhs == -1)) {
      return ICInterpretOpResult::NextIC;
    }
    if (lhs % rhs == 0 && lhs < 0) {
      return ICInterpretOpResult::NextIC;
    }
  });
  INT32_OP(Pow, <<, {
    if (rhs >= 32 || rhs < 0) {
      return ICInterpretOpResult::NextIC;
    }
  });
  INT32_OP(BitAnd, &, {});
  INT32_OP(BitOr, |, {});

  CACHEOP_CASE(Int32IncResult) {
    Int32OperandId intId = icregs.cacheIRReader.int32OperandId();
    int64_t value = int64_t(int32_t(icregs.icVals[intId.id()]));
    value++;
    if (value > INT32_MAX) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icResult = Int32Value(int32_t(value)).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CompareInt32Result) {
    JSOp op = icregs.cacheIRReader.jsop();
    Int32OperandId lhsId = icregs.cacheIRReader.int32OperandId();
    Int32OperandId rhsId = icregs.cacheIRReader.int32OperandId();
    int64_t lhs = int64_t(int32_t(icregs.icVals[lhsId.id()]));
    int64_t rhs = int64_t(int32_t(icregs.icVals[rhsId.id()]));
    TRACE_PRINTF("lhs (%d) = %" PRIi64 " rhs (%d) = %" PRIi64 "\n", lhsId.id(),
                 lhs, rhsId.id(), rhs);
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
    icregs.icResult = BooleanValue(result).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(Int32MinMax) {
    bool isMax = icregs.cacheIRReader.readBool();
    Int32OperandId lhsId = icregs.cacheIRReader.int32OperandId();
    Int32OperandId rhsId = icregs.cacheIRReader.int32OperandId();
    Int32OperandId resultId = icregs.cacheIRReader.int32OperandId();
    BOUNDSCHECK(resultId);
    int32_t lhs = int32_t(icregs.icVals[lhsId.id()]);
    int32_t rhs = int32_t(icregs.icVals[rhsId.id()]);
    int32_t result = ((lhs > rhs) ^ isMax) ? rhs : lhs;
    icregs.icVals[resultId.id()] = result;
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(IsObjectResult) {
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    Value val = Value::fromRawBits(icregs.icVals[valId.id()]);
    icregs.icResult = BooleanValue(val.isObject()).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CompareStringResult) {
    JSOp op = icregs.cacheIRReader.jsop();
    StringOperandId lhsId = icregs.cacheIRReader.stringOperandId();
    StringOperandId rhsId = icregs.cacheIRReader.stringOperandId();
    state.str0 = reinterpret_cast<JSString*>(icregs.icVals[lhsId.id()]);
    state.str1 = reinterpret_cast<JSString*>(icregs.icVals[rhsId.id()]);
    {
      PUSH_IC_FRAME();
      bool result;
      switch (op) {
        case JSOp::Eq:
        case JSOp::StrictEq:
          if (!StringsEqual<EqualityKind::Equal>(cx, state.str0, state.str1,
                                                 &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        case JSOp::Ne:
        case JSOp::StrictNe:
          if (!StringsEqual<EqualityKind::NotEqual>(cx, state.str0, state.str1,
                                                    &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        case JSOp::Lt:
          if (!StringsCompare<ComparisonKind::LessThan>(cx, state.str0,
                                                        state.str1, &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        case JSOp::Ge:
          if (!StringsCompare<ComparisonKind::GreaterThanOrEqual>(
                  cx, state.str0, state.str1, &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        case JSOp::Le:
          if (!StringsCompare<ComparisonKind::GreaterThanOrEqual>(
                  cx, state.str1, state.str0, &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        case JSOp::Gt:
          if (!StringsCompare<ComparisonKind::LessThan>(cx, state.str1,
                                                        state.str0, &result)) {
            return ICInterpretOpResult::Error;
          }
          break;
        default:
          MOZ_CRASH("bad opcode");
      }
      icregs.icResult = BooleanValue(result).asRawBits();
    }
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CompareNullUndefinedResult) {
    JSOp op = icregs.cacheIRReader.jsop();
    bool isUndefined = icregs.cacheIRReader.readBool();
    ValOperandId inputId = icregs.cacheIRReader.valOperandId();
    Value val = Value::fromRawBits(icregs.icVals[inputId.id()]);
    if (val.isObject() && val.toObject().getClass()->isProxyObject()) {
      return ICInterpretOpResult::NextIC;
    }

    bool result;
    switch (op) {
      case JSOp::Eq:
        result =
            val.isUndefined() || val.isNull() ||
            (val.isObject() && val.toObject().getClass()->emulatesUndefined());
        break;
      case JSOp::Ne:
        result = !(
            val.isUndefined() || val.isNull() ||
            (val.isObject() && val.toObject().getClass()->emulatesUndefined()));
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
    icregs.icResult = BooleanValue(result).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadObjectTruthyResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    JSObject* obj = reinterpret_cast<JSObject*>(icregs.icVals[objId.id()]);
    const JSClass* cls = obj->getClass();
    if (cls->isProxyObject()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icResult = BooleanValue(!cls->emulatesUndefined()).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardToInt32Index) {
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    Int32OperandId resultId = icregs.cacheIRReader.int32OperandId();
    BOUNDSCHECK(resultId);
    Value val = Value::fromRawBits(icregs.icVals[valId.id()]);
    if (val.isInt32()) {
      icregs.icVals[resultId.id()] = val.toInt32();
      DISPATCH_CACHEOP();
    } else if (val.isDouble()) {
      double doubleVal = val.toDouble();
      if (doubleVal >= double(INT32_MIN) && doubleVal <= double(INT32_MAX)) {
        icregs.icVals[resultId.id()] = int32_t(doubleVal);
        DISPATCH_CACHEOP();
      }
    }
    return ICInterpretOpResult::NextIC;
  }

  CACHEOP_CASE(GuardGlobalGeneration) {
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    uint32_t generationAddrOffset = icregs.cacheIRReader.stubOffset();
    uint32_t expected =
        cstub->stubInfo()->getStubRawInt32(cstub, expectedOffset);
    uint32_t* generationAddr = reinterpret_cast<uint32_t*>(
        cstub->stubInfo()->getStubRawWord(cstub, generationAddrOffset));
    if (*generationAddr != expected) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardFunctionScript) {
    ObjOperandId funId = icregs.cacheIRReader.objOperandId();
    JSFunction* fun = reinterpret_cast<JSFunction*>(icregs.icVals[funId.id()]);
    uint32_t expectedOffset = icregs.cacheIRReader.stubOffset();
    BaseScript* expected = reinterpret_cast<BaseScript*>(
        cstub->stubInfo()->getStubRawWord(cstub, expectedOffset));
    uint32_t nargsFlagsAndOffset = icregs.cacheIRReader.stubOffset();
    (void)nargsFlagsAndOffset;

    if (!fun->hasBaseScript() || fun->baseScript() != expected) {
      return ICInterpretOpResult::NextIC;
    }

    PREDICT_NEXT(CallScriptedFunction);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CallScriptedFunction)
  CACHEOP_CASE_FALLTHROUGH(CallNativeFunction) {
    bool isNative = cacheop == CacheOp::CallNativeFunction;
    TRACE_PRINTF("CallScriptedFunction / CallNativeFunction (native: %d)\n",
                 isNative);
    ObjOperandId calleeId = icregs.cacheIRReader.objOperandId();
    JSFunction* callee =
        reinterpret_cast<JSFunction*>(icregs.icVals[calleeId.id()]);
    Int32OperandId argcId = icregs.cacheIRReader.int32OperandId();
    uint32_t argc = uint32_t(icregs.icVals[argcId.id()]);
    CallFlags flags = icregs.cacheIRReader.callFlags();
    uint32_t argcFixed = icregs.cacheIRReader.uint32Immediate();
    (void)argcFixed;
    bool ignoresRv = false;
    if (isNative) {
      ignoresRv = icregs.cacheIRReader.readBool();
    }

    if (!isNative) {
      if (!callee->hasBaseScript() || !callee->baseScript()->hasBytecode() ||
          !callee->baseScript()->hasJitScript()) {
        return ICInterpretOpResult::NextIC;
      }
    }

    // For now, fail any constructing or different-realm cases.
    if (flags.isConstructing() || !flags.isSameRealm()) {
      TRACE_PRINTF("failing: constructing or not same realm\n");
      return ICInterpretOpResult::NextIC;
    }
    // And support only "standard" arg formats.
    if (flags.getArgFormat() != CallFlags::Standard) {
      TRACE_PRINTF("failing: not standard arg format\n");
      return ICInterpretOpResult::NextIC;
    }

    // For now, fail any arg-underflow case.
    if (argc < callee->nargs()) {
      TRACE_PRINTF("failing: too few args\n");
      return ICInterpretOpResult::NextIC;
    }

    uint32_t extra = 1 + flags.isConstructing() + isNative;
    uint32_t totalArgs = argc + extra;
    StackVal* origArgs = stack.cur();

    {
      PUSH_IC_FRAME();
      // This will not be an Exit frame but a BaselinStub frame, so
      // replace the ExitFrameType with the ICStub pointer.
      stack[0] = StackVal(cstub);

      if (!stack.check(sizeof(StackVal) * (totalArgs + 6))) {
        return ICInterpretOpResult::Error;
      }

      // Push args.
      for (uint32_t i = 0; i < totalArgs; i++) {
        stack.pushUnchecked(origArgs[i]);
      }
      Value* args = reinterpret_cast<Value*>(stack.cur());

      TRACE_PRINTF("pushing callee: %p\n", callee);
      stack.pushUnchecked(
          StackVal(CalleeToToken(callee, /* isConstructing = */ false)));
      stack.pushUnchecked(StackVal(
          MakeFrameDescriptorForJitCall(FrameType::BaselineStub, argc)));
      stack.pushUnchecked(StackVal(nullptr));  // fake return address.

      if (isNative) {
        // We *also* need an exit frame (the native baseline
        // execution would invoke a trampoline here).
        StackVal* trampolinePrevFP = stack.fp;
        stack.pushUnchecked(StackVal(stack.fp));
        stack.fp = stack.cur();
        stack.pushUnchecked(StackVal(uint64_t(ExitFrameType::Bare)));
        stack.pushUnchecked(StackVal(nullptr));  // fake return address.
        cx.getCx()->activation()->asJit()->setJSExitFP(
            reinterpret_cast<uint8_t*>(stack.fp));
        cx.getCx()->portableBaselineStack().top =
            reinterpret_cast<void*>(stack.sp);

        JSNative native = ignoresRv
                              ? callee->jitInfo()->ignoresReturnValueMethod
                              : callee->native();
        bool success = native(cx, argc, args);

        stack.fp = trampolinePrevFP;
        stack.popn(3);

        if (!success) {
          return ICInterpretOpResult::Error;
        }
        icregs.icResult = args[0].asRawBits();
      } else {
        switch (PortableBaselineInterpret(
            cx, state, stack, /* envChain = */ nullptr,
            reinterpret_cast<Value*>(&icregs.icResult))) {
          case PBIResult::Ok:
            break;
          case PBIResult::Error:
            return ICInterpretOpResult::Error;
          case PBIResult::Unwind:
            return ICInterpretOpResult::Unwind;
          case PBIResult::UnwindError:
            return ICInterpretOpResult::UnwindError;
          case PBIResult::UnwindRet:
            return ICInterpretOpResult::UnwindRet;
        }
      }
    }

    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(AssertPropertyLookup) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    uint32_t idOffset = icregs.cacheIRReader.stubOffset();
    uint32_t slotOffset = icregs.cacheIRReader.stubOffset();
    // Debug-only assertion; we can ignore.
    (void)objId;
    (void)idOffset;
    (void)slotOffset;
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CallInt32ToString) {
    Int32OperandId inputId = icregs.cacheIRReader.int32OperandId();
    StringOperandId resultId = icregs.cacheIRReader.stringOperandId();
    BOUNDSCHECK(resultId);
    int32_t input = int32_t(icregs.icVals[inputId.id()]);
    JSLinearString* str =
        Int32ToStringPure(frameMgr.cxForLocalUseOnly(), input);
    if (str) {
      icregs.icVals[resultId.id()] = reinterpret_cast<uintptr_t>(str);
    } else {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(CallStringConcatResult) {
    StringOperandId lhsId = icregs.cacheIRReader.stringOperandId();
    StringOperandId rhsId = icregs.cacheIRReader.stringOperandId();
    // We don't push a frame and do a CanGC invocation here; we do a
    // pure (NoGC) invocation only, because it's cheaper.
    FakeRooted<JSString*> lhs(
        nullptr, reinterpret_cast<JSString*>(icregs.icVals[lhsId.id()]));
    FakeRooted<JSString*> rhs(
        nullptr, reinterpret_cast<JSString*>(icregs.icVals[rhsId.id()]));
    JSString* result =
        ConcatStrings<NoGC>(frameMgr.cxForLocalUseOnly(), lhs, rhs);
    if (result) {
      icregs.icResult = StringValue(result).asRawBits();
    } else {
      return ICInterpretOpResult::NextIC;
    }
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadInt32ArrayLength) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    Int32OperandId resultId = icregs.cacheIRReader.int32OperandId();
    BOUNDSCHECK(resultId);
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    ObjectElements* elems = nobj->getElementsHeader();
    uint32_t length = elems->getLength();
    if (length > uint32_t(INT32_MAX)) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icVals[resultId.id()] = length;
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadInt32ArrayLengthResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    ObjectElements* elems = nobj->getElementsHeader();
    uint32_t length = elems->getLength();
    if (length > uint32_t(INT32_MAX)) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icResult = Int32Value(length).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadStringLengthResult) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    JSString* str = reinterpret_cast<JSString*>(icregs.icVals[strId.id()]);
    size_t length = str->length();
    if (length > size_t(INT32_MAX)) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icResult = Int32Value(length).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadStringCharResult) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    Int32OperandId indexId = icregs.cacheIRReader.int32OperandId();
    bool handleOOB = icregs.cacheIRReader.readBool();

    JSString* str =
        reinterpret_cast<JSLinearString*>(icregs.icVals[strId.id()]);
    int32_t index = int32_t(icregs.icVals[indexId.id()]);
    JSString* result = nullptr;
    if (index < 0 || size_t(index) >= str->length()) {
      if (handleOOB) {
        // Return an empty string.
        result = frameMgr.cxForLocalUseOnly()->names().empty;
      } else {
        return ICInterpretOpResult::NextIC;
      }
    } else {
      char16_t c;
      // Guaranteed to be always work because this CacheIR op is
      // always preceded by LinearizeForCharAccess.
      MOZ_ALWAYS_TRUE(str->getChar(/* cx = */ nullptr, index, &c));
      StaticStrings& sstr = frameMgr.cxForLocalUseOnly()->staticStrings();
      if (sstr.hasUnit(c)) {
        result = sstr.getUnit(c);
      } else {
        PUSH_IC_FRAME();
        result = StringFromCharCode(cx, c);
        if (!result) {
          return ICInterpretOpResult::Error;
        }
      }
    }
    icregs.icResult = StringValue(result).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadStringCharCodeResult) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    Int32OperandId indexId = icregs.cacheIRReader.int32OperandId();
    bool handleOOB = icregs.cacheIRReader.readBool();

    JSString* str =
        reinterpret_cast<JSLinearString*>(icregs.icVals[strId.id()]);
    int32_t index = int32_t(icregs.icVals[indexId.id()]);
    Value result;
    if (index < 0 || size_t(index) >= str->length()) {
      if (handleOOB) {
        // Return NaN.
        result = JS::NaNValue();
      } else {
        return ICInterpretOpResult::NextIC;
      }
    } else {
      char16_t c;
      // Guaranteed to be always work because this CacheIR op is
      // always preceded by LinearizeForCharAccess.
      MOZ_ALWAYS_TRUE(str->getChar(/* cx = */ nullptr, index, &c));
      result = Int32Value(c);
    }
    icregs.icResult = result.asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LinearizeForCharAccess) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    Int32OperandId indexId = icregs.cacheIRReader.int32OperandId();
    StringOperandId resultId = icregs.cacheIRReader.stringOperandId();
    BOUNDSCHECK(resultId);
    JSString* str =
        reinterpret_cast<JSLinearString*>(icregs.icVals[strId.id()]);
    (void)indexId;

    if (!str->isRope()) {
      icregs.icVals[resultId.id()] = reinterpret_cast<uintptr_t>(str);
    } else {
      PUSH_IC_FRAME();
      JSLinearString* result = LinearizeForCharAccess(cx, str);
      if (!result) {
        return ICInterpretOpResult::Error;
      }
      icregs.icVals[resultId.id()] = reinterpret_cast<uintptr_t>(result);
    }
    PREDICT_NEXT(LoadStringCharResult);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadStringTruthyResult) {
    StringOperandId strId = icregs.cacheIRReader.stringOperandId();
    JSString* str =
        reinterpret_cast<JSLinearString*>(icregs.icVals[strId.id()]);
    icregs.icResult = BooleanValue(str->length() > 0).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadInt32TruthyResult) {
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    int32_t val = int32_t(icregs.icVals[valId.id()]);
    icregs.icResult = BooleanValue(val != 0).asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(LoadDenseElementResult) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    Int32OperandId indexId = icregs.cacheIRReader.int32OperandId();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    ObjectElements* elems = nobj->getElementsHeader();
    int32_t index = int32_t(icregs.icVals[indexId.id()]);
    if (index < 0 || uint32_t(index) >= elems->getInitializedLength()) {
      return ICInterpretOpResult::NextIC;
    }
    HeapSlot* slot = &elems->elements()[index];
    Value val = slot->get();
    if (val.isMagic()) {
      return ICInterpretOpResult::NextIC;
    }
    icregs.icResult = val.asRawBits();
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(StoreDenseElement) {
    ObjOperandId objId = icregs.cacheIRReader.objOperandId();
    Int32OperandId indexId = icregs.cacheIRReader.int32OperandId();
    ValOperandId valId = icregs.cacheIRReader.valOperandId();
    NativeObject* nobj =
        reinterpret_cast<NativeObject*>(icregs.icVals[objId.id()]);
    ObjectElements* elems = nobj->getElementsHeader();
    int32_t index = int32_t(icregs.icVals[indexId.id()]);
    if (index < 0 || uint32_t(index) >= elems->getInitializedLength()) {
      return ICInterpretOpResult::NextIC;
    }
    HeapSlot* slot = &elems->elements()[index];
    if (slot->get().isMagic()) {
      return ICInterpretOpResult::NextIC;
    }
    Value val = Value::fromRawBits(icregs.icVals[valId.id()]);
    slot->set(nobj, HeapSlot::Element, index, val);
    PREDICT_NEXT(ReturnFromIC);
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardNoAllocationMetadataBuilder) {
    uint32_t builderAddrOffset = icregs.cacheIRReader.stubOffset();
    uintptr_t builderAddr =
        cstub->stubInfo()->getStubRawWord(cstub, builderAddrOffset);
    if (*reinterpret_cast<uintptr_t*>(builderAddr) != 0) {
      return ICInterpretOpResult::NextIC;
    }
    DISPATCH_CACHEOP();
  }

  CACHEOP_CASE(GuardIsNumber)
  CACHEOP_CASE(GuardToNonGCThing)
  CACHEOP_CASE(GuardBooleanToInt32)
  CACHEOP_CASE(Int32ToIntPtr)
  CACHEOP_CASE(GuardNumberToIntPtrIndex)
  CACHEOP_CASE(GuardToInt32ModUint32)
  CACHEOP_CASE(GuardToUint8Clamped)
  CACHEOP_CASE(GuardMultipleShapes)
  CACHEOP_CASE(GuardNullProto)
  CACHEOP_CASE(GuardAnyClass)
  CACHEOP_CASE(HasClassResult)
  CACHEOP_CASE(CallRegExpMatcherResult)
  CACHEOP_CASE(CallRegExpSearcherResult)
  CACHEOP_CASE(RegExpBuiltinExecMatchResult)
  CACHEOP_CASE(RegExpBuiltinExecTestResult)
  CACHEOP_CASE(RegExpFlagResult)
  CACHEOP_CASE(CallSubstringKernelResult)
  CACHEOP_CASE(StringReplaceStringResult)
  CACHEOP_CASE(StringSplitStringResult)
  CACHEOP_CASE(RegExpPrototypeOptimizableResult)
  CACHEOP_CASE(RegExpInstanceOptimizableResult)
  CACHEOP_CASE(GetFirstDollarIndexResult)
  CACHEOP_CASE(GuardCompartment)
  CACHEOP_CASE(GuardIsExtensible)
  CACHEOP_CASE(GuardIsNativeObject)
  CACHEOP_CASE(GuardIsProxy)
  CACHEOP_CASE(GuardIsNotProxy)
  CACHEOP_CASE(GuardIsNotArrayBufferMaybeShared)
  CACHEOP_CASE(GuardIsTypedArray)
  CACHEOP_CASE(GuardHasProxyHandler)
  CACHEOP_CASE(GuardIsNotDOMProxy)
  CACHEOP_CASE(GuardObjectIdentity)
  CACHEOP_CASE(GuardNoDenseElements)
  CACHEOP_CASE(GuardStringToIndex)
  CACHEOP_CASE(GuardStringToInt32)
  CACHEOP_CASE(GuardStringToNumber)
  CACHEOP_CASE(BooleanToNumber)
  CACHEOP_CASE(GuardHasGetterSetter)
  CACHEOP_CASE(GuardInt32IsNonNegative)
  CACHEOP_CASE(GuardIndexIsValidUpdateOrAdd)
  CACHEOP_CASE(GuardIndexIsNotDenseElement)
  CACHEOP_CASE(GuardTagNotEqual)
  CACHEOP_CASE(GuardXrayExpandoShapeAndDefaultProto)
  CACHEOP_CASE(GuardXrayNoExpando)
  CACHEOP_CASE(GuardDynamicSlotIsNotObject)
  CACHEOP_CASE(GuardFixedSlotValue)
  CACHEOP_CASE(GuardDynamicSlotValue)
  CACHEOP_CASE(LoadFixedSlot)
  CACHEOP_CASE(LoadDynamicSlot)
  CACHEOP_CASE(GuardFunctionHasJitEntry)
  CACHEOP_CASE(GuardFunctionHasNoJitEntry)
  CACHEOP_CASE(GuardFunctionIsNonBuiltinCtor)
  CACHEOP_CASE(GuardFunctionIsConstructor)
  CACHEOP_CASE(GuardNotClassConstructor)
  CACHEOP_CASE(GuardArrayIsPacked)
  CACHEOP_CASE(GuardArgumentsObjectFlags)
  CACHEOP_CASE(LoadEnclosingEnvironment)
  CACHEOP_CASE(LoadWrapperTarget)
  CACHEOP_CASE(LoadValueTag)
  CACHEOP_CASE(TruncateDoubleToUInt32)
  CACHEOP_CASE(MegamorphicLoadSlotResult)
  CACHEOP_CASE(MegamorphicLoadSlotByValueResult)
  CACHEOP_CASE(MegamorphicStoreSlot)
  CACHEOP_CASE(MegamorphicSetElement)
  CACHEOP_CASE(MegamorphicHasPropResult)
  CACHEOP_CASE(ObjectToIteratorResult)
  CACHEOP_CASE(ValueToIteratorResult)
  CACHEOP_CASE(LoadDOMExpandoValue)
  CACHEOP_CASE(LoadDOMExpandoValueGuardGeneration)
  CACHEOP_CASE(LoadDOMExpandoValueIgnoreGeneration)
  CACHEOP_CASE(GuardDOMExpandoMissingOrGuardShape)
  CACHEOP_CASE(AddAndStoreFixedSlot)
  CACHEOP_CASE(AddAndStoreDynamicSlot)
  CACHEOP_CASE(AllocateAndStoreDynamicSlot)
  CACHEOP_CASE(AddSlotAndCallAddPropHook)
  CACHEOP_CASE(StoreDenseElementHole)
  CACHEOP_CASE(ArrayPush)
  CACHEOP_CASE(ArrayJoinResult)
  CACHEOP_CASE(PackedArrayPopResult)
  CACHEOP_CASE(PackedArrayShiftResult)
  CACHEOP_CASE(PackedArraySliceResult)
  CACHEOP_CASE(ArgumentsSliceResult)
  CACHEOP_CASE(IsArrayResult)
  CACHEOP_CASE(StoreFixedSlotUndefinedResult)
  CACHEOP_CASE(IsPackedArrayResult)
  CACHEOP_CASE(IsCallableResult)
  CACHEOP_CASE(IsConstructorResult)
  CACHEOP_CASE(IsCrossRealmArrayConstructorResult)
  CACHEOP_CASE(IsTypedArrayResult)
  CACHEOP_CASE(IsTypedArrayConstructorResult)
  CACHEOP_CASE(ArrayBufferViewByteOffsetInt32Result)
  CACHEOP_CASE(ArrayBufferViewByteOffsetDoubleResult)
  CACHEOP_CASE(TypedArrayByteLengthInt32Result)
  CACHEOP_CASE(TypedArrayByteLengthDoubleResult)
  CACHEOP_CASE(TypedArrayElementSizeResult)
  CACHEOP_CASE(GuardHasAttachedArrayBuffer)
  CACHEOP_CASE(NewArrayIteratorResult)
  CACHEOP_CASE(NewStringIteratorResult)
  CACHEOP_CASE(NewRegExpStringIteratorResult)
  CACHEOP_CASE(ObjectCreateResult)
  CACHEOP_CASE(NewArrayFromLengthResult)
  CACHEOP_CASE(NewTypedArrayFromLengthResult)
  CACHEOP_CASE(NewTypedArrayFromArrayBufferResult)
  CACHEOP_CASE(NewTypedArrayFromArrayResult)
  CACHEOP_CASE(NewStringObjectResult)
  CACHEOP_CASE(StringFromCharCodeResult)
  CACHEOP_CASE(StringFromCodePointResult)
  CACHEOP_CASE(StringIndexOfResult)
  CACHEOP_CASE(StringStartsWithResult)
  CACHEOP_CASE(StringEndsWithResult)
  CACHEOP_CASE(StringToLowerCaseResult)
  CACHEOP_CASE(StringToUpperCaseResult)
  CACHEOP_CASE(MathAbsInt32Result)
  CACHEOP_CASE(MathAbsNumberResult)
  CACHEOP_CASE(MathClz32Result)
  CACHEOP_CASE(MathSignInt32Result)
  CACHEOP_CASE(MathSignNumberResult)
  CACHEOP_CASE(MathSignNumberToInt32Result)
  CACHEOP_CASE(MathImulResult)
  CACHEOP_CASE(MathSqrtNumberResult)
  CACHEOP_CASE(MathFRoundNumberResult)
  CACHEOP_CASE(MathRandomResult)
  CACHEOP_CASE(MathHypot2NumberResult)
  CACHEOP_CASE(MathHypot3NumberResult)
  CACHEOP_CASE(MathHypot4NumberResult)
  CACHEOP_CASE(MathAtan2NumberResult)
  CACHEOP_CASE(MathFloorNumberResult)
  CACHEOP_CASE(MathCeilNumberResult)
  CACHEOP_CASE(MathTruncNumberResult)
  CACHEOP_CASE(MathFloorToInt32Result)
  CACHEOP_CASE(MathCeilToInt32Result)
  CACHEOP_CASE(MathTruncToInt32Result)
  CACHEOP_CASE(MathRoundToInt32Result)
  CACHEOP_CASE(NumberMinMax)
  CACHEOP_CASE(Int32MinMaxArrayResult)
  CACHEOP_CASE(NumberMinMaxArrayResult)
  CACHEOP_CASE(MathFunctionNumberResult)
  CACHEOP_CASE(NumberParseIntResult)
  CACHEOP_CASE(DoubleParseIntResult)
  CACHEOP_CASE(ObjectToStringResult)
  CACHEOP_CASE(ReflectGetPrototypeOfResult)
  CACHEOP_CASE(StoreTypedArrayElement)
  CACHEOP_CASE(AtomicsCompareExchangeResult)
  CACHEOP_CASE(AtomicsExchangeResult)
  CACHEOP_CASE(AtomicsAddResult)
  CACHEOP_CASE(AtomicsSubResult)
  CACHEOP_CASE(AtomicsAndResult)
  CACHEOP_CASE(AtomicsOrResult)
  CACHEOP_CASE(AtomicsXorResult)
  CACHEOP_CASE(AtomicsLoadResult)
  CACHEOP_CASE(AtomicsStoreResult)
  CACHEOP_CASE(AtomicsIsLockFreeResult)
  CACHEOP_CASE(CallNativeSetter)
  CACHEOP_CASE(CallScriptedSetter)
  CACHEOP_CASE(CallInlinedSetter)
  CACHEOP_CASE(CallDOMSetter)
  CACHEOP_CASE(CallSetArrayLength)
  CACHEOP_CASE(ProxySet)
  CACHEOP_CASE(ProxySetByValue)
  CACHEOP_CASE(CallAddOrUpdateSparseElementHelper)
  CACHEOP_CASE(CallNumberToString)
  CACHEOP_CASE(Int32ToStringWithBaseResult)
  CACHEOP_CASE(BooleanToString)
  CACHEOP_CASE(CallBoundScriptedFunction)
  CACHEOP_CASE(CallWasmFunction)
  CACHEOP_CASE(GuardWasmArg)
  CACHEOP_CASE(CallDOMFunction)
  CACHEOP_CASE(CallClassHook)
  CACHEOP_CASE(CallInlinedFunction)
  CACHEOP_CASE(MetaScriptedThisShape)
  CACHEOP_CASE(BindFunctionResult)
  CACHEOP_CASE(SpecializedBindFunctionResult)
  CACHEOP_CASE(LoadFixedSlotTypedResult)
  CACHEOP_CASE(LoadDenseElementHoleResult)
  CACHEOP_CASE(CallGetSparseElementResult)
  CACHEOP_CASE(LoadDenseElementExistsResult)
  CACHEOP_CASE(LoadTypedArrayElementExistsResult)
  CACHEOP_CASE(LoadDenseElementHoleExistsResult)
  CACHEOP_CASE(LoadTypedArrayElementResult)
  CACHEOP_CASE(LoadDataViewValueResult)
  CACHEOP_CASE(StoreDataViewValueResult)
  CACHEOP_CASE(LoadArgumentsObjectArgResult)
  CACHEOP_CASE(LoadArgumentsObjectArgHoleResult)
  CACHEOP_CASE(LoadArgumentsObjectArgExistsResult)
  CACHEOP_CASE(LoadArgumentsObjectLengthResult)
  CACHEOP_CASE(LoadArgumentsObjectLength)
  CACHEOP_CASE(LoadFunctionLengthResult)
  CACHEOP_CASE(LoadFunctionNameResult)
  CACHEOP_CASE(LoadBoundFunctionNumArgs)
  CACHEOP_CASE(LoadBoundFunctionTarget)
  CACHEOP_CASE(GuardBoundFunctionIsConstructor)
  CACHEOP_CASE(LoadArrayBufferByteLengthInt32Result)
  CACHEOP_CASE(LoadArrayBufferByteLengthDoubleResult)
  CACHEOP_CASE(LoadArrayBufferViewLengthInt32Result)
  CACHEOP_CASE(LoadArrayBufferViewLengthDoubleResult)
  CACHEOP_CASE(FrameIsConstructingResult)
  CACHEOP_CASE(CallScriptedGetterResult)
  CACHEOP_CASE(CallInlinedGetterResult)
  CACHEOP_CASE(CallNativeGetterResult)
  CACHEOP_CASE(CallDOMGetterResult)
  CACHEOP_CASE(ProxyGetResult)
  CACHEOP_CASE(ProxyGetByValueResult)
  CACHEOP_CASE(ProxyHasPropResult)
  CACHEOP_CASE(CallObjectHasSparseElementResult)
  CACHEOP_CASE(CallNativeGetElementResult)
  CACHEOP_CASE(CallNativeGetElementSuperResult)
  CACHEOP_CASE(GetNextMapSetEntryForIteratorResult)
  CACHEOP_CASE(LoadUndefinedResult)
  CACHEOP_CASE(LoadDoubleConstant)
  CACHEOP_CASE(LoadBooleanConstant)
  CACHEOP_CASE(LoadUndefined)
  CACHEOP_CASE(LoadConstantString)
  CACHEOP_CASE(LoadInstanceOfObjectResult)
  CACHEOP_CASE(LoadTypeOfObjectResult)
  CACHEOP_CASE(DoubleAddResult)
  CACHEOP_CASE(DoubleSubResult)
  CACHEOP_CASE(DoubleMulResult)
  CACHEOP_CASE(DoubleDivResult)
  CACHEOP_CASE(DoubleModResult)
  CACHEOP_CASE(DoublePowResult)
  CACHEOP_CASE(BigIntAddResult)
  CACHEOP_CASE(BigIntSubResult)
  CACHEOP_CASE(BigIntMulResult)
  CACHEOP_CASE(BigIntDivResult)
  CACHEOP_CASE(BigIntModResult)
  CACHEOP_CASE(BigIntPowResult)
  CACHEOP_CASE(Int32BitXorResult)
  CACHEOP_CASE(Int32LeftShiftResult)
  CACHEOP_CASE(Int32RightShiftResult)
  CACHEOP_CASE(Int32URightShiftResult)
  CACHEOP_CASE(Int32NotResult)
  CACHEOP_CASE(BigIntBitOrResult)
  CACHEOP_CASE(BigIntBitXorResult)
  CACHEOP_CASE(BigIntBitAndResult)
  CACHEOP_CASE(BigIntLeftShiftResult)
  CACHEOP_CASE(BigIntRightShiftResult)
  CACHEOP_CASE(BigIntNotResult)
  CACHEOP_CASE(Int32NegationResult)
  CACHEOP_CASE(DoubleNegationResult)
  CACHEOP_CASE(BigIntNegationResult)
  CACHEOP_CASE(Int32DecResult)
  CACHEOP_CASE(DoubleIncResult)
  CACHEOP_CASE(DoubleDecResult)
  CACHEOP_CASE(BigIntIncResult)
  CACHEOP_CASE(BigIntDecResult)
  CACHEOP_CASE(LoadDoubleTruthyResult)
  CACHEOP_CASE(LoadBigIntTruthyResult)
  CACHEOP_CASE(LoadValueTruthyResult)
  CACHEOP_CASE(NewPlainObjectResult)
  CACHEOP_CASE(NewArrayObjectResult)
  CACHEOP_CASE(CallStringObjectConcatResult)
  CACHEOP_CASE(CallIsSuspendedGeneratorResult)
  CACHEOP_CASE(CompareObjectResult)
  CACHEOP_CASE(CompareSymbolResult)
  CACHEOP_CASE(CompareDoubleResult)
  CACHEOP_CASE(CompareBigIntResult)
  CACHEOP_CASE(CompareBigIntInt32Result)
  CACHEOP_CASE(CompareBigIntNumberResult)
  CACHEOP_CASE(CompareBigIntStringResult)
  CACHEOP_CASE(CompareDoubleSameValueResult)
  CACHEOP_CASE(SameValueResult)
  CACHEOP_CASE(IndirectTruncateInt32Result)
  CACHEOP_CASE(BigIntAsIntNResult)
  CACHEOP_CASE(BigIntAsUintNResult)
  CACHEOP_CASE(SetHasResult)
  CACHEOP_CASE(SetHasNonGCThingResult)
  CACHEOP_CASE(SetHasStringResult)
  CACHEOP_CASE(SetHasSymbolResult)
  CACHEOP_CASE(SetHasBigIntResult)
  CACHEOP_CASE(SetHasObjectResult)
  CACHEOP_CASE(SetSizeResult)
  CACHEOP_CASE(MapHasResult)
  CACHEOP_CASE(MapHasNonGCThingResult)
  CACHEOP_CASE(MapHasStringResult)
  CACHEOP_CASE(MapHasSymbolResult)
  CACHEOP_CASE(MapHasBigIntResult)
  CACHEOP_CASE(MapHasObjectResult)
  CACHEOP_CASE(MapGetResult)
  CACHEOP_CASE(MapGetNonGCThingResult)
  CACHEOP_CASE(MapGetStringResult)
  CACHEOP_CASE(MapGetSymbolResult)
  CACHEOP_CASE(MapGetBigIntResult)
  CACHEOP_CASE(MapGetObjectResult)
  CACHEOP_CASE(MapSizeResult)
  CACHEOP_CASE(ArrayFromArgumentsObjectResult)
  CACHEOP_CASE(CloseIterScriptedResult)
  CACHEOP_CASE(CallPrintString)
  CACHEOP_CASE(Breakpoint)
  CACHEOP_CASE(WrapResult)
  CACHEOP_CASE(Bailout)
  CACHEOP_CASE(AssertRecoveredOnBailoutResult)
  CACHEOP_CASE(GuardIsNotUninitializedLexical) {
    // printf("unknown CacheOp: %s\n", CacheIROpNames[int(op)]);
    return ICInterpretOpResult::NextIC;
  }

#undef PREDICT_NEXT
}

#define NEXT_IC() frame->interpreterICEntry()++;

#define INVOKE_IC(kind)                                          \
  switch (IC##kind(frame, frameMgr, state, icregs, stack, pc)) { \
    case PBIResult::Ok:                                          \
      break;                                                     \
    case PBIResult::Error:                                       \
      goto error;                                                \
    case PBIResult::Unwind:                                      \
      goto unwind;                                               \
    case PBIResult::UnwindError:                                 \
      goto unwind_error;                                         \
    case PBIResult::UnwindRet:                                   \
      goto unwind_ret;                                           \
  }                                                              \
  NEXT_IC();

#define SAVE_INPUTS(arity)            \
  do {                                \
    switch (arity) {                  \
      case 0:                         \
        break;                        \
      case 1:                         \
        inputs[0] = icregs.icVals[0]; \
        break;                        \
      case 2:                         \
        inputs[0] = icregs.icVals[0]; \
        inputs[1] = icregs.icVals[1]; \
        break;                        \
      case 3:                         \
        inputs[0] = icregs.icVals[0]; \
        inputs[1] = icregs.icVals[1]; \
        inputs[2] = icregs.icVals[2]; \
        break;                        \
    }                                 \
  } while (0)

#define RESTORE_INPUTS(arity)         \
  do {                                \
    switch (arity) {                  \
      case 0:                         \
        break;                        \
      case 1:                         \
        icregs.icVals[0] = inputs[0]; \
        break;                        \
      case 2:                         \
        icregs.icVals[0] = inputs[0]; \
        icregs.icVals[1] = inputs[1]; \
        break;                        \
      case 3:                         \
        icregs.icVals[0] = inputs[0]; \
        icregs.icVals[1] = inputs[1]; \
        icregs.icVals[2] = inputs[2]; \
        break;                        \
    }                                 \
  } while (0)

#define DEFINE_IC(kind, arity, fallback_body)                                 \
  static PBIResult MOZ_ALWAYS_INLINE IC##kind(                                \
      BaselineFrame* frame, VMFrameManager& frameMgr, State& state,           \
      ICRegs& icregs, Stack& stack, jsbytecode* pc) {                         \
    ICStub* stub = frame->interpreterICEntry()->firstStub();                  \
    uint64_t inputs[3];                                                       \
    SAVE_INPUTS(arity);                                                       \
    while (true) {                                                            \
    next_stub:                                                                \
      if (stub->isFallback()) {                                               \
        ICFallbackStub* fallback = stub->toFallbackStub();                    \
        fallback_body;                                                        \
        icregs.icResult = state.res.asRawBits();                              \
        return PBIResult::Ok;                                                 \
      error:                                                                  \
        return PBIResult::Error;                                              \
      } else {                                                                \
        ICCacheIRStub* cstub = stub->toCacheIRStub();                         \
        cstub->incrementEnteredCount();                                       \
        new (&icregs.cacheIRReader) CacheIRReader(cstub->stubInfo()->code()); \
        switch (ICInterpretOps(frame, frameMgr, state, icregs, stack, cstub,  \
                               pc)) {                                         \
          case ICInterpretOpResult::NextIC:                                   \
            stub = stub->maybeNext();                                         \
            RESTORE_INPUTS(arity);                                            \
            goto next_stub;                                                   \
          case ICInterpretOpResult::Return:                                   \
            return PBIResult::Ok;                                             \
          case ICInterpretOpResult::Error:                                    \
            return PBIResult::Error;                                          \
          case ICInterpretOpResult::Unwind:                                   \
            return PBIResult::Unwind;                                         \
          case ICInterpretOpResult::UnwindError:                              \
            return PBIResult::UnwindError;                                    \
          case ICInterpretOpResult::UnwindRet:                                \
            return PBIResult::UnwindRet;                                      \
        }                                                                     \
      }                                                                       \
    }                                                                         \
  }

#define IC_LOAD_VAL(state_elem, index) \
  state.state_elem = Value::fromRawBits(icregs.icVals[(index)]);
#define IC_LOAD_OBJ(state_elem, index) \
  state.state_elem = reinterpret_cast<JSObject*>(icregs.icVals[(index)]);

#define PUSH_FALLBACK_IC_FRAME() PUSH_EXIT_FRAME_OR_RET(PBIResult::Error);

DEFINE_IC(Typeof, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoTypeOfFallback(cx, frame, fallback, state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetName, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetNameFallback(cx, frame, fallback, state.obj0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(Call, 1, {
  uint32_t argc = uint32_t(icregs.icVals[0]);
  uint32_t totalArgs =
      argc + icregs.extraArgs;  // this, callee, (cosntructing?), func args
  Value* args = reinterpret_cast<Value*>(&stack[0]);
  TRACE_PRINTF("Call fallback: argc %d totalArgs %d args %p\n", argc, totalArgs,
               args);
  // Reverse values on the stack.
  for (uint32_t i = 0; i < totalArgs / 2; i++) {
    std::swap(args[i], args[totalArgs - 1 - i]);
  }
  {
    PUSH_FALLBACK_IC_FRAME();
    if (icregs.spreadCall) {
      if (!DoSpreadCallFallback(cx, frame, fallback, args, &state.res)) {
        for (uint32_t i = 0; i < totalArgs / 2; i++) {
          std::swap(args[i], args[totalArgs - 1 - i]);
        }
        goto error;
      }
    } else {
      if (!DoCallFallback(cx, frame, fallback, argc, args, &state.res)) {
        for (uint32_t i = 0; i < totalArgs / 2; i++) {
          std::swap(args[i], args[totalArgs - 1 - i]);
        }
        goto error;
      }
    }
  }
});

DEFINE_IC(UnaryArith, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoUnaryArithFallback(cx, frame, fallback, state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(BinaryArith, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoBinaryArithFallback(cx, frame, fallback, state.value0, state.value1,
                             &state.res)) {
    goto error;
  }
});

DEFINE_IC(ToBool, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoToBoolFallback(cx, frame, fallback, state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(Compare, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCompareFallback(cx, frame, fallback, state.value0, state.value1,
                         &state.res)) {
    goto error;
  }
});

DEFINE_IC(InstanceOf, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoInstanceOfFallback(cx, frame, fallback, state.value0, state.value1,
                            &state.res)) {
    goto error;
  }
});

DEFINE_IC(In, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoInFallback(cx, frame, fallback, state.value0, state.value1,
                    &state.res)) {
    goto error;
  }
});

DEFINE_IC(BindName, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoBindNameFallback(cx, frame, fallback, state.obj0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(SetProp, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoSetPropFallback(cx, frame, fallback, nullptr, state.value0,
                         state.value1)) {
    goto error;
  }
});

DEFINE_IC(NewObject, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoNewObjectFallback(cx, frame, fallback, &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetProp, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetPropFallback(cx, frame, fallback, &state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetPropSuper, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetPropSuperFallback(cx, frame, fallback, state.value0, &state.value1,
                              &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetElem, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetElemFallback(cx, frame, fallback, state.value0, state.value1,
                         &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetElemSuper, 3, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  IC_LOAD_VAL(value2, 2);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetElemSuperFallback(cx, frame, fallback, state.value0, state.value1,
                              state.value2, &state.res)) {
    goto error;
  }
});

DEFINE_IC(NewArray, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoNewArrayFallback(cx, frame, fallback, &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetIntrinsic, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetIntrinsicFallback(cx, frame, fallback, &state.res)) {
    goto error;
  }
});

DEFINE_IC(SetElem, 3, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  IC_LOAD_VAL(value2, 2);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoSetElemFallback(cx, frame, fallback, nullptr, state.value0,
                         state.value1, state.value2)) {
    goto error;
  }
});

DEFINE_IC(HasOwn, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoHasOwnFallback(cx, frame, fallback, state.value0, state.value1,
                        &state.res)) {
    goto error;
  }
});

DEFINE_IC(CheckPrivateField, 2, {
  IC_LOAD_VAL(value0, 0);
  IC_LOAD_VAL(value1, 1);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCheckPrivateFieldFallback(cx, frame, fallback, state.value0,
                                   state.value1, &state.res)) {
    goto error;
  }
});

DEFINE_IC(GetIterator, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoGetIteratorFallback(cx, frame, fallback, state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(ToPropertyKey, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoToPropertyKeyFallback(cx, frame, fallback, state.value0, &state.res)) {
    goto error;
  }
});

DEFINE_IC(OptimizeSpreadCall, 1, {
  IC_LOAD_VAL(value0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoOptimizeSpreadCallFallback(cx, frame, fallback, state.value0,
                                    &state.res)) {
    goto error;
  }
});

DEFINE_IC(Rest, 0, {
  PUSH_FALLBACK_IC_FRAME();
  if (!DoRestFallback(cx, frame, fallback, &state.res)) {
    goto error;
  }
});

DEFINE_IC(CloseIter, 1, {
  IC_LOAD_OBJ(obj0, 0);
  PUSH_FALLBACK_IC_FRAME();
  if (!DoCloseIterFallback(cx, frame, fallback, state.obj0)) {
    goto error;
  }
});

#define PUSH_EXIT_FRAME() PUSH_EXIT_FRAME_OR_RET(PBIResult::Error)

#define LABEL(op) (&&label_##op)
#define CASE(op) label_##op:
#ifndef TRACE_INTERP
#  define DISPATCH()                                                         \
    timingEndCyc = timing::rdtsc();                                          \
    js::timing::OpcodeTimers[timingOp].accum(timingEndCyc - timingStartCyc); \
    timingStartCyc = timingEndCyc;                                           \
    timingOp = int(*pc);                                                     \
    goto* addresses[timingOp]
#else
#  define DISPATCH() goto dispatch
#endif

#define ADVANCE(delta) pc += (delta);
#define ADVANCE_AND_DISPATCH(delta) \
  ADVANCE(delta);                   \
  DISPATCH();

#define END_OP(op) ADVANCE_AND_DISPATCH(JSOpLength_##op);

#define IC_SET_ARG_FROM_STACK(index, stack_index) \
  icregs.icVals[(index)] = stack[(stack_index)].asUInt64();
#define IC_POP_ARG(index) icregs.icVals[(index)] = stack.pop().asUInt64();
#define IC_SET_VAL_ARG(index, expr) icregs.icVals[(index)] = (expr).asRawBits();
#define IC_SET_OBJ_ARG(index, expr) \
  icregs.icVals[(index)] = reinterpret_cast<uint64_t>(expr);
#define IC_PUSH_RESULT() PUSH_UNCHECKED(StackVal(icregs.icResult));

#define PREDICT_NEXT(op) \
  if (JSOp(*pc) == JSOp::op) goto label_##op;

static PBIResult PortableBaselineInterpret(JSContext* cx_, State& state,
                                           Stack& parentStack,
                                           JSObject* envChain, Value* ret) {
#define OPCODE_LABEL(op, ...) LABEL(op),
#define TRAILING_LABEL(v) LABEL(default),

  static const void* const addresses[EnableInterruptsPseudoOpcode + 1] = {
      FOR_EACH_OPCODE(OPCODE_LABEL)
          FOR_EACH_TRAILING_UNUSED_OPCODE(TRAILING_LABEL)};

#undef OPCODE_LABEL
#undef TRAILING_LABEL

  AutoCheckRecursionLimit recursion(cx_);
  if (!recursion.check(cx_)) {
    return PBIResult::Error;
  }

  Stack stack(parentStack);

  TRY(stack.pushFrame(cx_, envChain));

  ICRegs icregs;
  BaselineFrame* frame = stack.frameFromFP();
  RootedScript script(cx_, frame->script());
  jsbytecode* pc = frame->interpreterPC();

  // Check max stack depth once, so we don't need to check it
  // otherwise below for ordinary stack-manipulation opcodes (just for
  // exit frames).
  TRY(stack.check(script->nslots()));

  uint32_t nfixed = script->nfixed();
  for (uint32_t i = 0; i < nfixed; i++) {
    stack.pushUnchecked(StackVal(UndefinedValue()));
  }
  ret->setUndefined();

  VMFrameManager frameMgr(cx_, frame);

  uint64_t timingStartCyc = timing::rdtsc();
  uint64_t timingEndCyc = timingStartCyc;
  int timingOp = 256;

  if (CalleeTokenIsFunction(frame->calleeToken())) {
    JSFunction* func = CalleeTokenToFunction(frame->calleeToken());
    frame->setEnvironmentChain(func->environment());
    if (func->needsFunctionEnvironmentObjects()) {
      PUSH_EXIT_FRAME();
      TRY(js::InitFunctionEnvironmentObjects(cx, frame));
      TRACE_PRINTF("callee is func %p; created environment object: %p\n", func,
                   frame->environmentChain());
    }
  }

  TRACE_PRINTF("Entering: sp = %p fp = %p frame = %p, script = %p, pc = %p\n",
               stack.sp, stack.fp, frame, script.get(), pc);
  TRACE_PRINTF("nslots = %d nfixed = %d\n", int(script->nslots()),
               int(script->nfixed()));

#ifdef TRACE_INTERP
dispatch:
#endif

  while (true) {
#ifdef TRACE_INTERP
    {
      JSOp op = JSOp(*pc);
      printf("stack[0] = %" PRIx64 " stack[1] = %" PRIx64 " stack[2] = %" PRIx64
             "\n",
             stack[0].asUInt64(), stack[1].asUInt64(), stack[2].asUInt64());
      printf("script = %p pc = %p: %s (ic %d) pending = %d\n", script.get(), pc,
             CodeName(op),
             (int)(frame->interpreterICEntry() -
                   script->jitScript()->icScript()->icEntries()),
             frameMgr.cxForLocalUseOnly()->isExceptionPending());
      printf("sp = %p fp = %p\n", stack.sp, stack.fp);
      printf("TOS tag: %d\n", int(stack[0].asValue().asRawBits() >> 47));
      fflush(stdout);
    }
#endif

    goto* addresses[*pc];

    CASE(Nop) { END_OP(Nop); }
    CASE(Undefined) {
      PUSH_UNCHECKED(StackVal(UndefinedValue()));
      END_OP(Undefined);
    }
    CASE(Null) {
      PUSH_UNCHECKED(StackVal(NullValue()));
      END_OP(Null);
    }
    CASE(False) {
      PUSH_UNCHECKED(StackVal(BooleanValue(false)));
      END_OP(False);
    }
    CASE(True) {
      PUSH_UNCHECKED(StackVal(BooleanValue(true)));
      END_OP(True);
    }
    CASE(Int32) {
      PUSH_UNCHECKED(StackVal(Int32Value(GET_INT32(pc))));
      END_OP(Int32);
    }
    CASE(Zero) {
      PUSH_UNCHECKED(StackVal(Int32Value(0)));
      END_OP(Zero);
    }
    CASE(One) {
      PUSH_UNCHECKED(StackVal(Int32Value(1)));
      END_OP(One);
    }
    CASE(Int8) {
      PUSH_UNCHECKED(StackVal(Int32Value(GET_INT8(pc))));
      END_OP(Int8);
    }
    CASE(Uint16) {
      PUSH_UNCHECKED(StackVal(Int32Value(GET_UINT16(pc))));
      END_OP(Uint16);
    }
    CASE(Uint24) {
      PUSH_UNCHECKED(StackVal(Int32Value(GET_UINT24(pc))));
      END_OP(Uint24);
    }
    CASE(Double) {
      PUSH_UNCHECKED(StackVal(GET_INLINE_VALUE(pc)));
      END_OP(Double);
    }
    CASE(BigInt) {
      PUSH_UNCHECKED(StackVal(JS::BigIntValue(script->getBigInt(pc))));
      END_OP(BigInt);
    }
    CASE(String) {
      PUSH_UNCHECKED(StackVal(StringValue(script->getString(pc))));
      END_OP(String);
    }
    CASE(Symbol) {
      PUSH_UNCHECKED(StackVal(
          SymbolValue(frameMgr.cxForLocalUseOnly()->wellKnownSymbols().get(
              GET_UINT8(pc)))));
      END_OP(Symbol);
    }
    CASE(Void) {
      stack[0] = StackVal(JS::UndefinedValue());
      END_OP(Void);
    }

    CASE(Typeof)
    CASE(TypeofExpr) {
      static_assert(JSOpLength_Typeof == JSOpLength_TypeofExpr);
      IC_POP_ARG(0);
      INVOKE_IC(Typeof);
      IC_PUSH_RESULT();
      END_OP(Typeof);
    }

    CASE(Pos) {
      if (stack[0].asValue().isNumber()) {
        // Nothing!
        NEXT_IC();
        END_OP(Pos);
      } else {
        goto generic_unary;
      }
    }
    CASE(Neg) {
      if (stack[0].asValue().isInt32()) {
        int32_t i = stack[0].asValue().toInt32();
        if (i != 0 && i != INT32_MIN) {
          stack[0] = StackVal(Int32Value(-i));
          NEXT_IC();
          END_OP(Neg);
        }
      }
      goto generic_unary;
    }

    CASE(Inc) {
      if (stack[0].asValue().isInt32()) {
        int32_t i = stack[0].asValue().toInt32();
        if (i != INT32_MAX) {
          stack[0] = StackVal(Int32Value(i + 1));
          NEXT_IC();
          END_OP(Inc);
        }
      }
      goto generic_unary;
    }
    CASE(Dec) {
      if (stack[0].asValue().isInt32()) {
        int32_t i = stack[0].asValue().toInt32();
        if (i != INT32_MIN) {
          stack[0] = StackVal(Int32Value(i - 1));
          NEXT_IC();
          END_OP(Dec);
        }
      }
      goto generic_unary;
    }

    CASE(BitNot) {
      if (stack[0].asValue().isInt32()) {
        int32_t i = stack[0].asValue().toInt32();
        stack[0] = StackVal(Int32Value(~i));
        NEXT_IC();
        END_OP(Inc);
      }
      goto generic_unary;
    }

  generic_unary:
    CASE(ToNumeric) {
      static_assert(JSOpLength_Pos == JSOpLength_Neg);
      static_assert(JSOpLength_Pos == JSOpLength_BitNot);
      static_assert(JSOpLength_Pos == JSOpLength_Inc);
      static_assert(JSOpLength_Pos == JSOpLength_Dec);
      static_assert(JSOpLength_Pos == JSOpLength_ToNumeric);
      IC_POP_ARG(0);
      INVOKE_IC(UnaryArith);
      IC_PUSH_RESULT();
      END_OP(Pos);
    }

    CASE(Not) {
      if (stack[0].asValue().isBoolean()) {
        stack[0] = StackVal(BooleanValue(!stack[0].asValue().toBoolean()));
        NEXT_IC();
      } else {
        IC_POP_ARG(0);
        INVOKE_IC(ToBool);
        PUSH_UNCHECKED(StackVal(
            BooleanValue(!Value::fromRawBits(icregs.icResult).toBoolean())));
      }
      END_OP(Not);
    }

    CASE(And) {
      bool result;
      if (stack[0].asValue().isBoolean()) {
        result = stack[0].asValue().toBoolean();
      } else {
        IC_SET_ARG_FROM_STACK(0, 0);
        INVOKE_IC(ToBool);
        result = Value::fromRawBits(icregs.icResult).toBoolean();
      }
      uint32_t jumpOffset = GET_JUMP_OFFSET(pc);
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
      if (stack[0].asValue().isBoolean()) {
        result = stack[0].asValue().toBoolean();
      } else {
        IC_SET_ARG_FROM_STACK(0, 0);
        INVOKE_IC(ToBool);
        result = Value::fromRawBits(icregs.icResult).toBoolean();
      }
      uint32_t jumpOffset = GET_JUMP_OFFSET(pc);
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
      if (stack[0].asValue().isBoolean()) {
        result = stack.pop().asValue().toBoolean();
      } else {
        IC_POP_ARG(0);
        INVOKE_IC(ToBool);
        result = Value::fromRawBits(icregs.icResult).toBoolean();
      }
      uint32_t jumpOffset = GET_JUMP_OFFSET(pc);
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
      if (stack[0].asValue().isBoolean()) {
        result = stack.pop().asValue().toBoolean();
      } else {
        IC_POP_ARG(0);
        INVOKE_IC(ToBool);
        result = Value::fromRawBits(icregs.icResult).toBoolean();
      }
      uint32_t jumpOffset = GET_JUMP_OFFSET(pc);
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
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        int64_t lhs = stack[1].asValue().toInt32();
        int64_t rhs = stack[0].asValue().toInt32();
        if (lhs + rhs <= int64_t(INT32_MAX)) {
          stack.pop();
          stack[0] = StackVal(Int32Value(int32_t(lhs + rhs)));
          NEXT_IC();
          END_OP(Add);
        }
      }
      goto generic_binary;
    }

    CASE(Sub) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        int64_t lhs = stack[1].asValue().toInt32();
        int64_t rhs = stack[0].asValue().toInt32();
        if (lhs - rhs >= int64_t(INT32_MIN)) {
          stack.pop();
          stack[0] = StackVal(Int32Value(int32_t(lhs - rhs)));
          NEXT_IC();
          END_OP(Sub);
        }
      }
      goto generic_binary;
    }

  generic_binary:
    CASE(BitOr)
    CASE(BitXor)
    CASE(BitAnd)
    CASE(Lsh)
    CASE(Rsh)
    CASE(Ursh)
    CASE(Mul)
    CASE(Div)
    CASE(Mod)
    CASE(Pow) {
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
      INVOKE_IC(BinaryArith);
      IC_PUSH_RESULT();
      END_OP(Div);
    }

    CASE(Eq) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[0].asValue().toInt32() == stack[1].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Eq);
      }
      goto generic_cmp;
    }

    CASE(Ne) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[0].asValue().toInt32() != stack[1].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Ne);
      }
      goto generic_cmp;
    }

    CASE(Lt) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[1].asValue().toInt32() < stack[0].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Lt);
      }
      goto generic_cmp;
    }
    CASE(Le) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[1].asValue().toInt32() <= stack[0].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Le);
      }
      goto generic_cmp;
    }
    CASE(Gt) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[1].asValue().toInt32() > stack[0].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Gt);
      }
      goto generic_cmp;
    }
    CASE(Ge) {
      if (stack[0].asValue().isInt32() && stack[1].asValue().isInt32()) {
        bool result =
            stack[1].asValue().toInt32() >= stack[0].asValue().toInt32();
        stack.pop();
        stack[0] = StackVal(BooleanValue(result));
        NEXT_IC();
        END_OP(Ge);
      }
      goto generic_cmp;
    }

  generic_cmp:
    CASE(StrictEq)
    CASE(StrictNe) {
      static_assert(JSOpLength_Eq == JSOpLength_Ne);
      static_assert(JSOpLength_Eq == JSOpLength_StrictEq);
      static_assert(JSOpLength_Eq == JSOpLength_StrictNe);
      static_assert(JSOpLength_Eq == JSOpLength_Lt);
      static_assert(JSOpLength_Eq == JSOpLength_Gt);
      static_assert(JSOpLength_Eq == JSOpLength_Le);
      static_assert(JSOpLength_Eq == JSOpLength_Ge);
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(Compare);
      IC_PUSH_RESULT();
      END_OP(Eq);
    }

    CASE(Instanceof) {
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(InstanceOf);
      IC_PUSH_RESULT();
      END_OP(Instanceof);
    }

    CASE(In) {
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(In);
      IC_PUSH_RESULT();
      END_OP(In);
    }

    CASE(ToPropertyKey) {
      IC_POP_ARG(0);
      INVOKE_IC(ToPropertyKey);
      IC_PUSH_RESULT();
      END_OP(ToPropertyKey);
    }

    CASE(ToString) {
      if (stack[0].asValue().isString()) {
        END_OP(ToString);
      }
      FakeRooted s(nullptr, stack[0].asValue());
      if (JSString* result = ToStringSlow<NoGC>(nullptr, s)) {
        stack[0] = StackVal(StringValue(result));
      } else {
        state.value0 = stack.pop().asValue();
        {
          PUSH_EXIT_FRAME();
          result = ToString<CanGC>(cx, state.value0);
          if (!result) {
            goto error;
          }
        }
        PUSH_UNCHECKED(StackVal(StringValue(result)));
      }
      END_OP(ToString);
    }

    CASE(IsNullOrUndefined) {
      bool result =
          stack[0].asValue().isNull() || stack[0].asValue().isUndefined();
      PUSH_UNCHECKED(StackVal(BooleanValue(result)));
      END_OP(IsNullOrUndefined);
    }

    CASE(GlobalThis) {
      PUSH_UNCHECKED(StackVal(ObjectValue(*frameMgr.cxForLocalUseOnly()
                                               ->global()
                                               ->lexicalEnvironment()
                                               .thisObject())));
      END_OP(GlobalThis);
    }

    CASE(NonSyntacticGlobalThis) {
      {
        PUSH_EXIT_FRAME();
        state.obj0 = frame->environmentChain();
        js::GetNonSyntacticGlobalThis(cx, state.obj0, &state.value0);
      }
      PUSH_UNCHECKED(StackVal(state.value0));
      END_OP(NonSyntacticGlobalThis);
    }

    CASE(NewTarget) {
      PUSH_UNCHECKED(StackVal(frame->newTarget()));
      END_OP(NewTarget);
    }

    CASE(DynamicImport) {
      state.value0 = stack.pop().asValue();  // options
      state.value1 = stack.pop().asValue();  // specifier
      JSObject* promise;
      {
        PUSH_EXIT_FRAME();
        promise =
            StartDynamicModuleImport(cx, script, state.value1, state.value0);
        if (!promise) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*promise)));
      END_OP(DynamicImport);
    }

    CASE(ImportMeta) {
      JSObject* metaObject;
      {
        PUSH_EXIT_FRAME();
        metaObject = ImportMetaOperation(cx, script);
        if (!metaObject) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*metaObject)));
      END_OP(ImportMeta);
    }

    CASE(NewInit) {
      INVOKE_IC(NewObject);
      IC_PUSH_RESULT();
      END_OP(NewInit);
    }
    CASE(NewObject) {
      INVOKE_IC(NewObject);
      IC_PUSH_RESULT();
      END_OP(NewObject);
    }
    CASE(Object) {
      PUSH_UNCHECKED(StackVal(ObjectValue(*script->getObject(pc))));
      END_OP(Object);
    }
    CASE(ObjWithProto) {
      state.value0 = stack[0].asValue();
      JSObject* obj;
      {
        PUSH_EXIT_FRAME();
        obj = ObjectWithProtoOperation(cx, state.value0);
      }
      stack[0] = StackVal(ObjectValue(*obj));
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
      StackVal val = stack[0];
      IC_POP_ARG(2);
      IC_POP_ARG(1);
      IC_SET_ARG_FROM_STACK(0, 0);
      if (JSOp(*pc) == JSOp::SetElem || JSOp(*pc) == JSOp::StrictSetElem) {
        stack[0] = val;
      }
      INVOKE_IC(SetElem);
      if (JSOp(*pc) == JSOp::InitElemInc) {
        PUSH_UNCHECKED(StackVal(
            Int32Value(Value::fromRawBits(icregs.icVals[1]).toInt32() + 1)));
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
      state.obj1 = &stack.pop().asValue().toObject();  // val
      state.obj0 = &stack[0].asValue().toObject();     // obj; leave on stack
      state.name0 = script->getName(pc);
      {
        PUSH_EXIT_FRAME();
        if (!InitPropGetterSetterOperation(cx, pc, state.obj0, state.name0,
                                           state.obj1)) {
          goto error;
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
      state.obj1 = &stack.pop().asValue().toObject();  // val
      state.value0 = stack.pop().asValue();            // idval
      state.obj0 = &stack[0].asValue().toObject();     // obj; leave on stack
      {
        PUSH_EXIT_FRAME();
        if (!InitElemGetterSetterOperation(cx, pc, state.obj0, state.value0,
                                           state.obj1)) {
          goto error;
        }
      }
      END_OP(InitElemGetter);
    }

    CASE(GetProp)
    CASE(GetBoundName) {
      static_assert(JSOpLength_GetProp == JSOpLength_GetBoundName);
      IC_POP_ARG(0);
      INVOKE_IC(GetProp);
      IC_PUSH_RESULT();
      END_OP(GetProp);
    }
    CASE(GetPropSuper) {
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(GetPropSuper);
      IC_PUSH_RESULT();
      END_OP(GetPropSuper);
    }

    CASE(GetElem) {
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(GetElem);
      IC_PUSH_RESULT();
      END_OP(GetElem);
    }

    CASE(GetElemSuper) {
      IC_POP_ARG(2);
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(GetElemSuper);
      IC_PUSH_RESULT();
      END_OP(GetElemSuper);
    }

    CASE(DelProp) {
      state.value0 = stack.pop().asValue();
      state.name0 = script->getName(pc);
      bool res = false;
      {
        PUSH_EXIT_FRAME();
        if (!DelPropOperation<true>(cx, state.value0, state.name0, &res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(BooleanValue(res)));
      END_OP(DelProp);
    }
    CASE(StrictDelProp) {
      state.value0 = stack.pop().asValue();
      state.name0 = script->getName(pc);
      bool res = false;
      {
        PUSH_EXIT_FRAME();
        if (!DelPropOperation<true>(cx, state.value0, state.name0, &res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(BooleanValue(res)));
      END_OP(StrictDelProp);
    }
    CASE(DelElem) {
      state.value1 = stack.pop().asValue();
      state.value0 = stack.pop().asValue();
      bool res = false;
      {
        PUSH_EXIT_FRAME();
        if (!DelElemOperation<true>(cx, state.value0, state.value1, &res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(BooleanValue(res)));
      END_OP(DelElem);
    }
    CASE(StrictDelElem) {
      state.value1 = stack.pop().asValue();
      state.value0 = stack.pop().asValue();
      bool res = false;
      {
        PUSH_EXIT_FRAME();
        if (!DelElemOperation<true>(cx, state.value0, state.value1, &res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(BooleanValue(res)));
      END_OP(StrictDelElem);
    }

    CASE(HasOwn) {
      IC_POP_ARG(1);
      IC_POP_ARG(0);
      INVOKE_IC(HasOwn);
      IC_PUSH_RESULT();
      END_OP(HasOwn);
    }

    CASE(CheckPrivateField) {
      IC_SET_ARG_FROM_STACK(1, 1);
      IC_SET_ARG_FROM_STACK(0, 0);
      INVOKE_IC(CheckPrivateField);
      IC_PUSH_RESULT();
      END_OP(CheckPrivateField);
    }

    CASE(NewPrivateName) {
      state.atom0 = script->getAtom(pc);
      JS::Symbol* symbol;
      {
        PUSH_EXIT_FRAME();
        symbol = NewPrivateName(cx, state.atom0);
        if (!symbol) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(SymbolValue(symbol)));
      END_OP(NewPrivateName);
    }

    CASE(SuperBase) {
      JSFunction& superEnvFunc =
          stack.pop().asValue().toObject().as<JSFunction>();
      MOZ_ASSERT(superEnvFunc.allowSuperProperty());
      MOZ_ASSERT(superEnvFunc.baseScript()->needsHomeObject());
      const Value& homeObjVal = superEnvFunc.getExtendedSlot(
          FunctionExtended::METHOD_HOMEOBJECT_SLOT);

      JSObject* homeObj = &homeObjVal.toObject();
      JSObject* superBase = HomeObjectSuperBase(homeObj);

      PUSH_UNCHECKED(StackVal(ObjectOrNullValue(superBase)));
      END_OP(SuperBase);
    }

    CASE(SetPropSuper)
    CASE(StrictSetPropSuper) {
      // stack signature: receiver, lval, rval => rval
      static_assert(JSOpLength_SetPropSuper == JSOpLength_StrictSetPropSuper);
      bool strict = JSOp(*pc) == JSOp::StrictSetPropSuper;
      state.value2 = stack.pop().asValue();  // rval
      state.value1 = stack.pop().asValue();  // lval
      state.value0 = stack.pop().asValue();  // receiver
      state.name0 = script->getName(pc);
      {
        PUSH_EXIT_FRAME();
        // SetPropertySuper(cx, lval, receiver, name, rval, strict)
        // (N.B.: lval and receiver are transposed!)
        if (!SetPropertySuper(cx, state.value1, state.value0, state.name0,
                              state.value2, strict)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.value2));
      END_OP(SetPropSuper);
    }

    CASE(SetElemSuper)
    CASE(StrictSetElemSuper) {
      // stack signature: receiver, key, lval, rval => rval
      static_assert(JSOpLength_SetElemSuper == JSOpLength_StrictSetElemSuper);
      bool strict = JSOp(*pc) == JSOp::StrictSetElemSuper;
      state.value3 = stack.pop().asValue();  // rval
      state.value2 = stack.pop().asValue();  // lval
      state.value1 = stack.pop().asValue();  // index
      state.value0 = stack.pop().asValue();  // receiver
      {
        PUSH_EXIT_FRAME();
        // SetElementSuper(cx, lval, receiver, index, rval, strict)
        // (N.B.: lval, receiver and index are rotated!)
        if (!SetElementSuper(cx, state.value2, state.value0, state.value1,
                             state.value3, strict)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.value2));  // value
      END_OP(SetElemSuper);
    }

    CASE(Iter) {
      IC_POP_ARG(0);
      INVOKE_IC(GetIterator);
      IC_PUSH_RESULT();
      END_OP(Iter);
    }

    CASE(MoreIter) {
      // iter => iter, name
      Value v = IteratorMore(&stack[0].asValue().toObject());
      PUSH_UNCHECKED(StackVal(v));
      END_OP(MoreIter);
    }

    CASE(IsNoIter) {
      // iter => iter, bool
      bool result = stack[0].asValue().isMagic(JS_NO_ITER_VALUE);
      PUSH_UNCHECKED(StackVal(BooleanValue(result)));
      END_OP(IsNoIter);
    }

    CASE(EndIter) {
      // iter, interval =>
      stack.pop();
      CloseIterator(&stack.pop().asValue().toObject());
      END_OP(EndIter);
    }

    CASE(CloseIter) {
      IC_SET_OBJ_ARG(0, &stack.pop().asValue().toObject());
      INVOKE_IC(CloseIter);
      END_OP(CloseIter);
    }

    CASE(CheckIsObj) {
      if (!stack[0].asValue().isObject()) {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(
            js::ThrowCheckIsObject(cx, js::CheckIsObjectKind(GET_UINT8(pc))));
        goto error;
      }
      END_OP(CheckIsObj);
    }

    CASE(CheckObjCoercible) {
      state.value0 = stack[0].asValue();
      if (state.value0.isNullOrUndefined()) {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(ThrowObjectCoercible(cx, state.value0));
        goto error;
      }
      END_OP(CheckObjCoercible);
    }

    CASE(ToAsyncIter) {
      // iter, next => asynciter
      state.value0 = stack.pop().asValue();            // next
      state.obj0 = &stack.pop().asValue().toObject();  // iter

      JSObject* result;
      {
        PUSH_EXIT_FRAME();
        result = CreateAsyncFromSyncIterator(cx, state.obj0, state.value0);
        if (!result) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*result)));
      END_OP(ToAsyncIter);
    }

    CASE(MutateProto) {
      // obj, protoVal => obj
      state.value0 = stack.pop().asValue();
      state.obj0 = &stack[0].asValue().toObject();
      {
        PUSH_EXIT_FRAME();
        if (!MutatePrototype(cx, state.obj0.as<PlainObject>(), state.value0)) {
          goto error;
        }
      }
      END_OP(MutateProto);
    }

    CASE(NewArray) {
      INVOKE_IC(NewArray);
      IC_PUSH_RESULT();
      END_OP(NewArray);
    }

    CASE(InitElemArray) {
      // array, val => array
      state.value0 = stack.pop().asValue();
      state.obj0 = &stack[0].asValue().toObject();
      {
        PUSH_EXIT_FRAME();
        InitElemArrayOperation(cx, pc, state.obj0.as<ArrayObject>(),
                               state.value0);
      }
      END_OP(InitElemArray);
    }

    CASE(Hole) {
      PUSH_UNCHECKED(StackVal(MagicValue(JS_ELEMENTS_HOLE)));
      END_OP(Hole);
    }

    CASE(RegExp) {
      JSObject* obj;
      {
        PUSH_EXIT_FRAME();
        state.obj0 = script->getRegExp(pc);
        obj = CloneRegExpObject(cx, state.obj0.as<RegExpObject>());
        if (!obj) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*obj)));
      END_OP(RegExp);
    }

    CASE(Lambda) {
      state.fun0 = script->getFunction(pc);
      state.obj0 = frame->environmentChain();
      JSObject* res;
      {
        PUSH_EXIT_FRAME();
        res = js::Lambda(cx, state.fun0, state.obj0);
        if (!res) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*res)));
      END_OP(Lambda);
    }

    CASE(SetFunName) {
      // fun, name => fun
      state.value0 = stack.pop().asValue();  // name
      state.fun0 = &stack[0].asValue().toObject().as<JSFunction>();
      FunctionPrefixKind prefixKind = FunctionPrefixKind(GET_UINT8(pc));
      {
        PUSH_EXIT_FRAME();
        if (!SetFunctionName(cx, state.fun0, state.value0, prefixKind)) {
          goto error;
        }
      }
      END_OP(SetFunName);
    }

    CASE(InitHomeObject) {
      // fun, homeObject => fun
      state.obj0 = &stack.pop().asValue().toObject();  // homeObject
      state.fun0 = &stack[0].asValue().toObject().as<JSFunction>();
      MOZ_ASSERT(state.fun0->allowSuperProperty());
      MOZ_ASSERT(state.obj0->is<PlainObject>() || state.obj0->is<JSFunction>());
      state.fun0->setExtendedSlot(FunctionExtended::METHOD_HOMEOBJECT_SLOT,
                                  ObjectValue(*state.obj0));
      END_OP(InitHomeObject);
    }

    CASE(CheckClassHeritage) {
      state.value0 = stack[0].asValue();
      {
        PUSH_EXIT_FRAME();
        if (!CheckClassHeritageOperation(cx, state.value0)) {
          goto error;
        }
      }
      END_OP(CheckClassHeritage);
    }

    CASE(FunWithProto) {
      // proto => obj
      state.obj0 = &stack.pop().asValue().toObject();  // proto
      state.obj1 = frame->environmentChain();
      state.fun0 = script->getFunction(pc);
      JSObject* obj;
      {
        PUSH_EXIT_FRAME();
        obj = FunWithProtoOperation(cx, state.fun0, state.obj1, state.obj0);
        if (!obj) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*obj)));
      END_OP(FunWithProto);
    }

    CASE(BuiltinObject) {
      auto kind = BuiltinObjectKind(GET_UINT8(pc));
      JSObject* builtin;
      {
        PUSH_EXIT_FRAME();
        builtin = BuiltinObjectOperation(cx, kind);
        if (!builtin) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*builtin)));
      END_OP(BuiltinObject);
    }

    CASE(Call)
    CASE(CallIgnoresRv)
    CASE(CallContent)
    CASE(CallIter)
    CASE(CallContentIter)
    CASE(Eval)
    CASE(StrictEval) {
      static_assert(JSOpLength_Call == JSOpLength_CallIgnoresRv);
      static_assert(JSOpLength_Call == JSOpLength_CallContent);
      static_assert(JSOpLength_Call == JSOpLength_CallIter);
      static_assert(JSOpLength_Call == JSOpLength_CallContentIter);
      static_assert(JSOpLength_Call == JSOpLength_Eval);
      static_assert(JSOpLength_Call == JSOpLength_StrictEval);
      uint32_t argc = GET_ARGC(pc);
      icregs.icVals[0] = argc;
      icregs.extraArgs = 2;
      icregs.spreadCall = false;
      INVOKE_IC(Call);
      stack.popn(argc + 2);
      PUSH_UNCHECKED(StackVal(Value::fromRawBits(icregs.icResult)));
      END_OP(Call);
    }

    CASE(SuperCall)
    CASE(New)
    CASE(NewContent) {
      static_assert(JSOpLength_SuperCall == JSOpLength_New);
      static_assert(JSOpLength_SuperCall == JSOpLength_NewContent);
      uint32_t argc = GET_ARGC(pc);
      icregs.icVals[0] = argc;
      icregs.extraArgs = 3;
      icregs.spreadCall = false;
      INVOKE_IC(Call);
      stack.popn(argc + 3);
      PUSH_UNCHECKED(StackVal(Value::fromRawBits(icregs.icResult)));
      END_OP(SuperCall);
    }

    CASE(SpreadCall)
    CASE(SpreadEval)
    CASE(StrictSpreadEval) {
      static_assert(JSOpLength_SpreadCall == JSOpLength_SpreadEval);
      static_assert(JSOpLength_SpreadCall == JSOpLength_StrictSpreadEval);
      icregs.icVals[0] = 1;
      icregs.extraArgs = 2;
      icregs.spreadCall = true;
      INVOKE_IC(Call);
      stack.popn(3);
      PUSH_UNCHECKED(StackVal(Value::fromRawBits(icregs.icResult)));
      END_OP(SpreadCall);
    }

    CASE(SpreadSuperCall)
    CASE(SpreadNew) {
      static_assert(JSOpLength_SpreadSuperCall == JSOpLength_SpreadNew);
      icregs.icVals[0] = 1;
      icregs.extraArgs = 3;
      icregs.spreadCall = true;
      INVOKE_IC(Call);
      stack.popn(4);
      PUSH_UNCHECKED(StackVal(Value::fromRawBits(icregs.icResult)));
      END_OP(SpreadSuperCall);
    }

    CASE(OptimizeSpreadCall) {
      IC_POP_ARG(0);
      INVOKE_IC(OptimizeSpreadCall);
      IC_PUSH_RESULT();
      END_OP(OptimizeSpreadCall);
    }

    CASE(ImplicitThis) {
      state.obj0 = frame->environmentChain();
      state.name0 = script->getName(pc);
      {
        PUSH_EXIT_FRAME();
        if (!ImplicitThisOperation(cx, state.obj0, state.name0, &state.res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.res));
      END_OP(ImplicitThis);
    }

    CASE(CallSiteObj) {
      JSObject* cso = script->getObject(pc);
      MOZ_ASSERT(!cso->as<ArrayObject>().isExtensible());
      MOZ_ASSERT(cso->as<ArrayObject>().containsPure(
          frameMgr.cxForLocalUseOnly()->names().raw));
      PUSH_UNCHECKED(StackVal(ObjectValue(*cso)));
      END_OP(CallSiteObj);
    }

    CASE(IsConstructing) {
      PUSH_UNCHECKED(StackVal(MagicValue(JS_IS_CONSTRUCTING)));
      END_OP(IsConstructing);
    }

    CASE(SuperFun) {
      JSObject* superEnvFunc = &stack.pop().asValue().toObject();
      JSObject* superFun = SuperFunOperation(superEnvFunc);
      PUSH_UNCHECKED(StackVal(ObjectOrNullValue(superFun)));
      END_OP(SuperFun);
    }

    CASE(CheckThis)
    CASE(CheckThisReinit) {
      static_assert(JSOpLength_CheckThis == JSOpLength_CheckThisReinit);
      if (!stack[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(ThrowInitializedThis(cx));
        goto error;
      }
      END_OP(CheckThis);
    }

    CASE(Generator) {
      JSObject* generator;
      {
        PUSH_EXIT_FRAME();
        generator = CreateGeneratorFromFrame(cx, frame);
        if (!generator) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*generator)));
      END_OP(Generator);
    }

    CASE(InitialYield) {
      // gen => rval, gen, resumeKind
      state.obj0 = &stack[0].asValue().toObject();
      uint32_t frameSize = stack.frameSize(frame);
      {
        PUSH_EXIT_FRAME();
        if (!NormalSuspend(cx, state.obj0, frame, frameSize, pc)) {
          goto error;
        }
      }
      *ret = stack[0].asValue();
      stack.popFrame(frameMgr.cxForLocalUseOnly());
      stack.pop();  // fake return address
      return PBIResult::Ok;
    }

    CASE(Await)
    CASE(Yield) {
      // rval1, gen => rval2, gen, resumeKind
      state.obj0 = &stack[0].asValue().toObject();
      uint32_t frameSize = stack.frameSize(frame);
      {
        PUSH_EXIT_FRAME();
        if (!NormalSuspend(cx, state.obj0, frame, frameSize, pc)) {
          goto error;
        }
      }
      *ret = stack[0].asValue();
      stack.popFrame(frameMgr.cxForLocalUseOnly());
      stack.pop();  // fake return address
      return PBIResult::Ok;
    }

    CASE(FinalYieldRval) {
      // gen =>
      state.obj0 = &stack.pop().asValue().toObject();
      {
        PUSH_EXIT_FRAME();
        if (!FinalSuspend(cx, state.obj0, pc)) {
          goto error;
        }
      }
      stack.popFrame(frameMgr.cxForLocalUseOnly());
      stack.pop();  // fake return address
      return PBIResult::Ok;
    }

    CASE(IsGenClosing) {
      PUSH_UNCHECKED(StackVal(MagicValue(JS_GENERATOR_CLOSING)));
      END_OP(IsGenClosing);
    }

    CASE(AsyncAwait) {
      // value, gen => promise
      state.obj0 = &stack.pop().asValue().toObject();  // gen
      state.value0 = stack.pop().asValue();            // value
      JSObject* promise;
      {
        PUSH_EXIT_FRAME();
        promise = AsyncFunctionAwait(
            cx, state.obj0.as<AsyncFunctionGeneratorObject>(), state.value0);
        if (!promise) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*promise)));
      END_OP(AsyncAwait);
    }

    CASE(AsyncResolve) {
      // valueOrReason, gen => promise
      state.obj0 = &stack.pop().asValue().toObject();  // gen
      state.value0 = stack.pop().asValue();            // valueOrReason
      auto resolveKind = AsyncFunctionResolveKind(GET_UINT8(pc));
      JSObject* promise;
      {
        PUSH_EXIT_FRAME();
        promise = AsyncFunctionResolve(
            cx, state.obj0.as<AsyncFunctionGeneratorObject>(), state.value0,
            resolveKind);
        if (!promise) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*promise)));
      END_OP(AsyncResolve);
    }

    CASE(CanSkipAwait) {
      // value => value, can_skip
      state.value0 = stack[0].asValue();
      bool result = false;
      {
        PUSH_EXIT_FRAME();
        if (!CanSkipAwait(cx, state.value0, &result)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(BooleanValue(result)));
      END_OP(CanSkipAwait);
    }

    CASE(MaybeExtractAwaitValue) {
      // value, can_skip => value_or_resolved, can_skip
      state.value1 = stack.pop().asValue();  // can_skip
      state.value0 = stack.pop().asValue();  // value
      if (state.value1.toBoolean()) {
        PUSH_EXIT_FRAME();
        if (!ExtractAwaitValue(cx, state.value0, &state.value0)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.value0));
      PUSH_UNCHECKED(StackVal(state.value1));
      END_OP(MaybeExtractAwaitValue);
    }

    CASE(ResumeKind) {
      GeneratorResumeKind resumeKind = ResumeKindFromPC(pc);
      PUSH_UNCHECKED(StackVal(Int32Value(int32_t(resumeKind))));
      END_OP(ResumeKind);
    }

    CASE(CheckResumeKind) {
      // rval, gen, resumeKind => rval
      GeneratorResumeKind resumeKind =
          IntToResumeKind(stack.pop().asValue().toInt32());
      state.obj0 = &stack.pop().asValue().toObject();  // gen
      state.value0 = stack[0].asValue();               // rval
      if (resumeKind != GeneratorResumeKind::Next) {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(GeneratorThrowOrReturn(
            cx, frame, state.obj0.as<AbstractGeneratorObject>(), state.value0,
            resumeKind));
        goto error;
      }
      END_OP(CheckResumeKind);
    }

    CASE(Resume) {
      MOZ_CRASH("implement resume");
      END_OP(Resume);
    }

    CASE(JumpTarget) {
      int32_t icIndex = GET_INT32(pc);
      frame->interpreterICEntry() = frame->icScript()->icEntries() + icIndex;
      END_OP(JumpTarget);
    }
    CASE(LoopHead) {
      int32_t icIndex = GET_INT32(pc);
      frame->interpreterICEntry() = frame->icScript()->icEntries() + icIndex;
      END_OP(LoopHead);
    }
    CASE(AfterYield) {
      int32_t icIndex = GET_INT32(pc);
      frame->interpreterICEntry() = frame->icScript()->icEntries() + icIndex;
      END_OP(AfterYield);
    }

    CASE(Goto) {
      ADVANCE(GET_JUMP_OFFSET(pc));
      PREDICT_NEXT(JumpTarget);
      PREDICT_NEXT(LoopHead);
      DISPATCH();
    }

    CASE(Coalesce) {
      if (!stack[0].asValue().isNullOrUndefined()) {
        ADVANCE(GET_JUMP_OFFSET(pc));
        DISPATCH();
      } else {
        END_OP(Coalesce);
      }
    }

    CASE(Case) {
      bool cond = stack.pop().asValue().toBoolean();
      if (cond) {
        stack.pop();
        ADVANCE(GET_JUMP_OFFSET(pc));
        DISPATCH();
      } else {
        END_OP(Case);
      }
    }

    CASE(Default) {
      stack.pop();
      ADVANCE(GET_JUMP_OFFSET(pc));
      DISPATCH();
    }

    CASE(TableSwitch) {
      int32_t len = GET_JUMP_OFFSET(pc);
      int32_t low = GET_JUMP_OFFSET(pc + 1 * JUMP_OFFSET_LEN);
      int32_t high = GET_JUMP_OFFSET(pc + 2 * JUMP_OFFSET_LEN);
      Value v = stack.pop().asValue();
      int32_t i = 0;
      if (v.isInt32()) {
        i = v.toInt32();
      } else if (!v.isDouble() ||
                 !mozilla::NumberEqualsInt32(v.toDouble(), &i)) {
        ADVANCE(len);
        DISPATCH();
      }

      i = uint32_t(i) - uint32_t(low);
      if ((uint32_t(i) < uint32_t(high - low + 1))) {
        len = script->tableSwitchCaseOffset(pc, uint32_t(i)) -
              script->pcToOffset(pc);
      }
      ADVANCE(len);
      DISPATCH();
    }

    CASE(Return) {
      *ret = stack.pop().asValue();
      stack.popFrame(frameMgr.cxForLocalUseOnly());
      stack.pop();  // fake return address
      return PBIResult::Ok;
    }

    CASE(GetRval) {
      PUSH_UNCHECKED(StackVal(*ret));
      END_OP(GetRval);
    }

    CASE(SetRval) {
      *ret = stack.pop().asValue();
      END_OP(SetRval);
    }

    CASE(RetRval) {
      stack.popFrame(frameMgr.cxForLocalUseOnly());
      stack.pop();  // fake return address
      return PBIResult::Ok;
    }

    CASE(CheckReturn) {
      Value thisval = stack.pop().asValue();
      if (ret->isObject()) {
        PUSH_UNCHECKED(StackVal(*ret));
      } else if (!ret->isUndefined()) {
        PUSH_EXIT_FRAME();
        state.value0 = *ret;
        ReportValueError(cx, JSMSG_BAD_DERIVED_RETURN, JSDVG_IGNORE_STACK,
                         state.value0, nullptr);
        goto error;
      } else if (thisval.isMagic(JS_UNINITIALIZED_LEXICAL)) {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(ThrowUninitializedThis(cx));
        goto error;
      } else {
        PUSH_UNCHECKED(StackVal(thisval));
      }
      END_OP(CheckReturn);
    }

    CASE(Throw) {
      state.value0 = stack.pop().asValue();
      {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(ThrowOperation(cx, state.value0));
        goto error;
      }
      END_OP(Throw);
    }

    CASE(ThrowMsg) {
      {
        PUSH_EXIT_FRAME();
        MOZ_ALWAYS_FALSE(ThrowMsgOperation(cx, GET_UINT8(pc)));
        goto error;
      }
      END_OP(ThrowMsg);
    }

    CASE(ThrowSetConst) {
      {
        PUSH_EXIT_FRAME();
        ReportRuntimeLexicalError(cx, JSMSG_BAD_CONST_ASSIGN, script, pc);
        goto error;
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
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.res));
      END_OP(Exception);
    }

    CASE(Finally) { END_OP(Finally); }

    CASE(Uninitialized) {
      PUSH_UNCHECKED(StackVal(MagicValue(JS_UNINITIALIZED_LEXICAL)));
      END_OP(Uninitialized);
    }
    CASE(InitLexical) {
      uint32_t i = GET_LOCALNO(pc);
      frame->unaliasedLocal(i) = stack[0].asValue();
      END_OP(InitLexical);
    }

    CASE(InitAliasedLexical) {
      EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
      EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
      obj.setAliasedBinding(ec, stack[0].asValue());
      END_OP(InitAliasedLexical);
    }
    CASE(CheckLexical) {
      if (stack[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
        PUSH_EXIT_FRAME();
        ReportRuntimeLexicalError(cx, JSMSG_UNINITIALIZED_LEXICAL, script, pc);
        goto error;
      }
      END_OP(CheckLexical);
    }
    CASE(CheckAliasedLexical) {
      if (stack[0].asValue().isMagic(JS_UNINITIALIZED_LEXICAL)) {
        PUSH_EXIT_FRAME();
        ReportRuntimeLexicalError(cx, JSMSG_UNINITIALIZED_LEXICAL, script, pc);
        goto error;
      }
      END_OP(CheckAliasedLexical);
    }

    CASE(BindGName) {
      IC_SET_OBJ_ARG(
          0, &frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
      state.name0.set(script->getName(pc));
      INVOKE_IC(BindName);
      IC_PUSH_RESULT();
      END_OP(BindGName);
    }
    CASE(BindName) {
      IC_SET_OBJ_ARG(0, frame->environmentChain());
      INVOKE_IC(BindName);
      IC_PUSH_RESULT();
      END_OP(BindName);
    }
    CASE(GetGName) {
      IC_SET_OBJ_ARG(
          0, &frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
      INVOKE_IC(GetName);
      IC_PUSH_RESULT();
      END_OP(GetGName);
    }
    CASE(GetName) {
      IC_SET_OBJ_ARG(0, frame->environmentChain());
      INVOKE_IC(GetName);
      IC_PUSH_RESULT();
      END_OP(GetName);
    }

    CASE(GetArg) {
      unsigned i = GET_ARGNO(pc);
      if (script->argsObjAliasesFormals()) {
        PUSH_UNCHECKED(StackVal(frame->argsObj().arg(i)));
      } else {
        PUSH_UNCHECKED(StackVal(frame->unaliasedFormal(i)));
      }
      END_OP(GetArg);
    }

    CASE(GetFrameArg) {
      uint32_t i = GET_ARGNO(pc);
      PUSH_UNCHECKED(StackVal(frame->unaliasedFormal(i, DONT_CHECK_ALIASING)));
      END_OP(GetFrameArg);
    }

    CASE(GetLocal) {
      uint32_t i = GET_LOCALNO(pc);
      TRACE_PRINTF(" -> local: %d\n", int(i));
      PUSH_UNCHECKED(StackVal(frame->unaliasedLocal(i)));
      END_OP(GetLocal);
    }

    CASE(ArgumentsLength) {
      PUSH_UNCHECKED(StackVal(Int32Value(frame->numActualArgs())));
      END_OP(ArgumentsLength);
    }

    CASE(GetActualArg) {
      MOZ_ASSERT(!script->needsArgsObj());
      uint32_t index = stack[0].asValue().toInt32();
      stack[0] = StackVal(frame->unaliasedActual(index));
      END_OP(GetActualArg);
    }

    CASE(GetAliasedVar)
    CASE(GetAliasedDebugVar) {
      static_assert(JSOpLength_GetAliasedVar == JSOpLength_GetAliasedDebugVar);
      EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
      EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
      PUSH_UNCHECKED(StackVal(obj.aliasedBinding(ec)));
      END_OP(GetAliasedVar);
    }

    CASE(GetImport) {
      state.obj0 = frame->environmentChain();
      state.value0 = stack[0].asValue();
      {
        PUSH_EXIT_FRAME();
        if (!GetImportOperation(cx, state.obj0, script, pc, &state.value0)) {
          goto error;
        }
      }
      stack[0] = StackVal(state.value0);
      END_OP(GetImport);
    }

    CASE(GetIntrinsic) {
      INVOKE_IC(GetIntrinsic);
      IC_PUSH_RESULT();
      END_OP(GetIntrinsic);
    }

    CASE(Callee) {
      PUSH_UNCHECKED(StackVal(frame->calleev()));
      END_OP(Callee);
    }

    CASE(EnvCallee) {
      uint8_t numHops = GET_UINT8(pc);
      JSObject* env = &frame->environmentChain()->as<EnvironmentObject>();
      for (unsigned i = 0; i < numHops; i++) {
        env = &env->as<EnvironmentObject>().enclosingEnvironment();
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(env->as<CallObject>().callee())));
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
      PUSH_UNCHECKED(StackVal(icregs.icVals[1]));
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
      INVOKE_IC(SetProp);
      END_OP(InitProp);
    }
    CASE(InitGLexical) {
      IC_SET_ARG_FROM_STACK(1, 0);
      IC_SET_OBJ_ARG(
          0, &frameMgr.cxForLocalUseOnly()->global()->lexicalEnvironment());
      INVOKE_IC(SetProp);
      END_OP(InitGLexical);
    }

    CASE(SetArg) {
      unsigned i = GET_ARGNO(pc);
      if (script->argsObjAliasesFormals()) {
        frame->argsObj().setArg(i, stack[0].asValue());
      } else {
        frame->unaliasedFormal(i) = stack[0].asValue();
      }
      END_OP(SetArg);
    }

    CASE(SetLocal) {
      uint32_t i = GET_LOCALNO(pc);
      TRACE_PRINTF(" -> local: %d\n", int(i));
      frame->unaliasedLocal(i) = stack[0].asValue();
      END_OP(SetLocal);
    }

    CASE(SetAliasedVar) {
      EnvironmentCoordinate ec = EnvironmentCoordinate(pc);
      EnvironmentObject& obj = getEnvironmentFromCoordinate(frame, ec);
      MOZ_ASSERT(!IsUninitializedLexical(obj.aliasedBinding(ec)));
      obj.setAliasedBinding(ec, stack[0].asValue());
      END_OP(SetAliasedVar);
    }

    CASE(SetIntrinsic) {
      state.value0 = stack[0].asValue();
      {
        PUSH_EXIT_FRAME();
        if (!SetIntrinsicOperation(cx, script, pc, state.value0)) {
          goto error;
        }
      }
      END_OP(SetIntrinsic);
    }

    CASE(PushLexicalEnv) {
      state.scope0 = script->getScope(pc);
      {
        PUSH_EXIT_FRAME();
        if (!frame->pushLexicalEnvironment(cx,
                                           state.scope0.as<LexicalScope>())) {
          goto error;
        }
      }
      END_OP(PushLexicalEnv);
    }
    CASE(PopLexicalEnv) {
      frame->popOffEnvironmentChain<LexicalEnvironmentObject>();
      END_OP(PopLexicalEnv);
    }
    CASE(DebugLeaveLexicalEnv) { END_OP(DebugLeaveLexicalEnv); }

    CASE(RecreateLexicalEnv) {
      {
        PUSH_EXIT_FRAME();
        if (!frame->recreateLexicalEnvironment(cx)) {
          goto error;
        }
      }
      END_OP(RecreateLexicalEnv);
    }

    CASE(FreshenLexicalEnv) {
      {
        PUSH_EXIT_FRAME();
        if (!frame->freshenLexicalEnvironment(cx)) {
          goto error;
        }
      }
      END_OP(FreshenLexicalEnv);
    }
    CASE(PushClassBodyEnv) {
      state.scope0 = script->getScope(pc);
      {
        PUSH_EXIT_FRAME();
        if (!frame->pushClassBodyEnvironment(
                cx, state.scope0.as<ClassBodyScope>())) {
          goto error;
        }
      }
      END_OP(PushClassBodyEnv);
    }
    CASE(PushVarEnv) {
      state.scope0 = script->getScope(pc);
      {
        PUSH_EXIT_FRAME();
        if (!frame->pushVarEnvironment(cx, state.scope0)) {
          goto error;
        }
      }
      END_OP(PushVarEnv);
    }
    CASE(EnterWith) {
      state.scope0 = script->getScope(pc);
      state.value0 = stack.pop().asValue();
      {
        PUSH_EXIT_FRAME();
        if (!EnterWithOperation(cx, frame, state.value0,
                                state.scope0.as<WithScope>())) {
          goto error;
        }
      }
      END_OP(EnterWith);
    }
    CASE(LeaveWith) {
      frame->popOffEnvironmentChain<WithEnvironmentObject>();
      END_OP(LeaveWith);
    }
    CASE(BindVar) {
      state.obj0 = frame->environmentChain();
      JSObject* varObj;
      {
        PUSH_EXIT_FRAME();
        varObj = BindVarOperation(cx, state.obj0);
      }
      PUSH_UNCHECKED(StackVal(ObjectValue(*varObj)));
      END_OP(BindVar);
    }

    CASE(GlobalOrEvalDeclInstantiation) {
      GCThingIndex lastFun = GET_GCTHING_INDEX(pc);
      state.script0 = script;
      state.obj0 = frame->environmentChain();
      {
        PUSH_EXIT_FRAME();
        if (!GlobalOrEvalDeclInstantiation(cx, state.obj0, state.script0,
                                           lastFun)) {
          goto error;
        }
      }
      END_OP(GlobalOrEvalDeclInstantiation);
    }

    CASE(DelName) {
      state.name0 = script->getName(pc);
      state.obj0 = frame->environmentChain();
      {
        PUSH_EXIT_FRAME();
        if (!DeleteNameOperation(cx, state.name0, state.obj0, &state.res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.res));
      END_OP(DelName);
    }

    CASE(Arguments) {
      {
        PUSH_EXIT_FRAME();
        if (!NewArgumentsObject(cx, frame, &state.res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.res));
      END_OP(Arguments);
    }

    CASE(Rest) {
      INVOKE_IC(Rest);
      IC_PUSH_RESULT();
      END_OP(Rest);
    }

    CASE(FunctionThis) {
      {
        PUSH_EXIT_FRAME();
        if (!js::GetFunctionThis(cx, frame, &state.res)) {
          goto error;
        }
      }
      PUSH_UNCHECKED(StackVal(state.res));
      END_OP(FunctionThis);
    }

    CASE(Pop) {
      stack.pop();
      END_OP(Pop);
    }
    CASE(PopN) {
      uint32_t n = GET_UINT16(pc);
      stack.popn(n);
      END_OP(PopN);
    }
    CASE(Dup) {
      StackVal value = stack[0];
      PUSH_UNCHECKED(value);
      END_OP(Dup);
    }
    CASE(Dup2) {
      StackVal value1 = stack[0];
      StackVal value2 = stack[1];
      PUSH_UNCHECKED(value2);
      PUSH_UNCHECKED(value1);
      END_OP(Dup2);
    }
    CASE(DupAt) {
      unsigned i = GET_UINT24(pc);
      StackVal value = stack[i];
      PUSH_UNCHECKED(value);
      END_OP(DupAt);
    }
    CASE(Swap) {
      std::swap(stack[0], stack[1]);
      END_OP(Swap);
    }
    CASE(Pick) {
      unsigned i = GET_UINT8(pc);
      StackVal tmp = stack[i];
      memmove(&stack[1], &stack[0], sizeof(StackVal) * i);
      stack[0] = tmp;
      END_OP(Pick);
    }
    CASE(Unpick) {
      unsigned i = GET_UINT8(pc);
      StackVal tmp = stack[0];
      memmove(&stack[0], &stack[1], sizeof(StackVal) * i);
      stack[i] = tmp;
      END_OP(Unpick);
    }
    CASE(DebugCheckSelfHosted) { END_OP(DebugCheckSelfHosted); }
    CASE(Lineno) { END_OP(Lineno); }
    CASE(NopDestructuring) { END_OP(NopDestructuring); }
    CASE(ForceInterpreter) { END_OP(ForceInterpreter); }
    CASE(Debugger) { END_OP(Debugger); }

  label_default:
    MOZ_CRASH("Bad opcode");
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
        *ret = MagicValue(JS_ION_ERROR);
        stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        stack.sp = reinterpret_cast<StackVal*>(rfe.stackPointer);
        goto unwind_error;
      case ExceptionResumeKind::Catch:
        pc = frame->interpreterPC();
        stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        stack.sp = reinterpret_cast<StackVal*>(rfe.stackPointer);
        TRACE_PRINTF(" -> catch to pc %p\n", pc);
        goto unwind;
      case ExceptionResumeKind::Finally:
        pc = frame->interpreterPC();
        stack.fp = reinterpret_cast<StackVal*>(rfe.framePointer);
        stack.sp = reinterpret_cast<StackVal*>(rfe.stackPointer);
        TRACE_PRINTF(" -> finally to pc %p\n", pc);
        PUSH_UNCHECKED(StackVal(rfe.exception));
        PUSH_UNCHECKED(StackVal(BooleanValue(true)));
        goto unwind;
      case ExceptionResumeKind::ForcedReturnBaseline:
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

unwind:
  if (reinterpret_cast<uintptr_t>(stack.fp) >
      reinterpret_cast<uintptr_t>(frame) + BaselineFrame::Size()) {
    return PBIResult::Unwind;
  }
  DISPATCH();
unwind_error:
  if (reinterpret_cast<uintptr_t>(stack.fp) >
      reinterpret_cast<uintptr_t>(frame) + BaselineFrame::Size()) {
    return PBIResult::UnwindError;
  }
  return PBIResult::Error;
unwind_ret:
  if (reinterpret_cast<uintptr_t>(stack.fp) >
      reinterpret_cast<uintptr_t>(frame) + BaselineFrame::Size()) {
    return PBIResult::UnwindRet;
  }
  return PBIResult::Ok;
}

bool js::PortableBaselineTrampoline(JSContext* cx, size_t argc, Value* argv,
                                    size_t numActualArgs,
                                    CalleeToken calleeToken, JSObject* envChain,
                                    Value* result) {
  State state(cx);
  Stack stack(cx->portableBaselineStack());

  // Expected stack frame:
  // - argN
  // - ...
  // - arg1
  // - this
  // - calleeToken
  // - descriptor
  // - "return address" (nullptr for top frame)

  if (CalleeTokenIsConstructing(calleeToken)) {
    argc++;
  }

  if (!stack.check(sizeof(StackVal) * (argc + 3))) {
    return false;
  }

  for (size_t i = 0; i < argc; i++) {
    stack.pushUnchecked(StackVal(argv[argc - 1 - i]));
  }
  stack.pushUnchecked(StackVal(calleeToken));
  stack.pushUnchecked(StackVal(
      MakeFrameDescriptorForJitCall(FrameType::CppToJSJit, numActualArgs)));
  stack.pushUnchecked(StackVal(nullptr));  // Fake return address.

  switch (PortableBaselineInterpret(cx, state, stack, envChain, result)) {
    case PBIResult::Ok:
    case PBIResult::UnwindRet:
      break;
    case PBIResult::Error:
    case PBIResult::UnwindError:
      return false;
    case PBIResult::Unwind:
      MOZ_CRASH("Should not unwind out of top / entry frame");
  }

  // Pop the descriptor, calleeToken, and args. (Return address is
  // popped in callee.)
  stack.popn(2 + argc);

  return true;
}

MethodStatus js::CanEnterPortableBaselineInterpreter(JSContext* cx,
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
  if (state.script()->nslots() > BaselineMaxScriptSlots) {
    return MethodStatus::Method_CantCompile;
  }
  if (state.isInvoke()) {
    InvokeState& invoke = *state.asInvoke();
    if (TooManyActualArguments(invoke.args().length())) {
      return MethodStatus::Method_CantCompile;
    }
  }
  if (state.script()->getWarmUpCount() <=
      JitOptions.portableBaselineInterpreterWarmUpThreshold) {
    return MethodStatus::Method_Skipped;
  }
  if (!cx->realm()->ensureJitRealmExists(cx)) {
    return MethodStatus::Method_Error;
  }

  AutoKeepJitScripts keepJitScript(cx);
  if (!state.script()->ensureHasJitScript(cx, keepJitScript)) {
    return MethodStatus::Method_Error;
  }
  state.script()->updateJitCodeRaw(cx->runtime());
  return MethodStatus::Method_Compiled;
}
