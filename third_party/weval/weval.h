#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* partial-evaluation async requests and queues                              */
/* ------------------------------------------------------------------------- */

typedef void (*weval_func_t)();

typedef struct weval_req_t weval_req_t;
typedef struct weval_req_arg_t weval_req_arg_t;

struct weval_req_t {
  weval_req_t* next;
  weval_req_t* prev;
  weval_func_t func;
  weval_req_arg_t* args;
  uint32_t nargs;
  weval_func_t* specialized;
};

typedef enum {
  weval_req_arg_i32 = 0,
  weval_req_arg_i64 = 1,
  weval_req_arg_f32 = 2,
  weval_req_arg_f64 = 3,
} weval_req_arg_type;

struct weval_req_arg_t {
  uint32_t specialize;
  uint32_t ty;
  union {
    uint32_t i32;
    uint64_t i64;
    float f32;
    double f64;
  } u;
};

extern weval_req_t* weval_req_pending_head;

#define WEVAL_DEFINE_REQ_LIST()                                    \
  weval_req_t* weval_req_pending_head;                             \
  __attribute__((export_name("weval.pending.head"))) weval_req_t** \
  __weval_pending_head() {                                         \
    return &weval_req_pending_head;                                \
  }

static inline void weval_request(weval_req_t* req) {
  req->next = weval_req_pending_head;
  req->prev = NULL;
  if (weval_req_pending_head) {
    weval_req_pending_head->prev = req;
  }
  weval_req_pending_head = req;
}

static inline void weval_free(weval_req_t* req) {
  if (req->prev) {
    req->prev->next = req->next;
  } else if (weval_req_pending_head == req) {
    weval_req_pending_head = req->next;
  }
  if (req->next) {
    req->next->prev = req->prev;
  }
  if (req->args) {
    free(req->args);
  }
  free(req);
}

/* ------------------------------------------------------------------------- */
/* intrinsics                                                                */
/* ------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

#define WEVAL_WASM_IMPORT(name) \
  __attribute__((__import_module__("weval"), __import_name__(name)))

const void* weval_assume_const_memory(const void* p)
    WEVAL_WASM_IMPORT("assume.const.memory");
const void* weval_assume_const_memory_transitive(const void* p)
    WEVAL_WASM_IMPORT("assume.const.memory.transitive");
void weval_push_context(uint32_t pc) WEVAL_WASM_IMPORT("push.context");
void weval_pop_context() WEVAL_WASM_IMPORT("pop.context");
void weval_update_context(uint32_t pc) WEVAL_WASM_IMPORT("update.context");
void* weval_make_symbolic_ptr(void* p) WEVAL_WASM_IMPORT("make.symbolic.ptr");
void weval_flush_to_mem() WEVAL_WASM_IMPORT("flush.to.mem");
void weval_trace_line(uint32_t line_number) WEVAL_WASM_IMPORT("trace.line");
void weval_abort_specialization(uint32_t line_number, uint32_t fatal)
    WEVAL_WASM_IMPORT("abort.specialization");
void weval_assert_const32(uint32_t value, uint32_t line_no)
    WEVAL_WASM_IMPORT("assert.const32");
void weval_assert_const_memory(void* p, uint32_t line_no)
    WEVAL_WASM_IMPORT("assert.const.memory");
uint32_t weval_specialize_value(uint32_t value, uint32_t lo, uint32_t hi)
    WEVAL_WASM_IMPORT("specialize.value");
void weval_print(const char* message, uint32_t line, uint32_t val)
    WEVAL_WASM_IMPORT("print");

void weval_context_bucket(uint32_t bucket) WEVAL_WASM_IMPORT("context.bucket");

#undef WEVAL_WASM_IMPORT

#ifdef __cplusplus
}  // extern "C"
#endif

#ifdef __cplusplus
namespace weval {
template <typename T>
const T* assume_const_memory(const T* t) {
  return (const T*)weval_assume_const_memory((const void*)t);
}
template <typename T>
T* assume_const_memory(T* t) {
  return (T*)weval_assume_const_memory((void*)t);
}
template <typename T>
const T* assume_const_memory_transitive(const T* t) {
  return (const T*)weval_assume_const_memory_transitive((const void*)t);
}
template <typename T>
T* assume_const_memory_transitive(T* t) {
  return (T*)weval_assume_const_memory_transitive((void*)t);
}

static inline void push_context(uint32_t pc) { weval_push_context(pc); }

static inline void pop_context() { weval_pop_context(); }

static inline void update_context(uint32_t pc) { weval_update_context(pc); }
template <typename T>
static T* make_symbolic_ptr(T* t) {
  return (T*)weval_make_symbolic_ptr((void*)t);
}
template <typename T>
void flush_to_mem() {
  weval_flush_to_mem();
}

}  // namespace weval
#endif  // __cplusplus

/* ------------------------------------------------------------------------- */
/* C++ type-safe wrapper for partial evaluation of functions                 */
/* ------------------------------------------------------------------------- */

#ifdef __cplusplus
namespace weval {

template <typename T>
struct ArgSpec {};

template <typename T>
struct RuntimeArg : ArgSpec<T> {};

template <typename T>
RuntimeArg<T> Runtime() {
  return RuntimeArg<T>{};
}

template <typename T>
struct Specialize : ArgSpec<T> {
  T value;
  explicit Specialize(T value_) : value(value_) {}
};

namespace impl {
template <typename Ret, typename... Args>
using FuncPtr = Ret (*)(Args...);

template <typename T>
struct StoreArg;

template <>
struct StoreArg<uint32_t> {
  void operator()(weval_req_arg_t* arg, uint32_t value) {
    arg->specialize = 1;
    arg->ty = weval_req_arg_i32;
    arg->u.i32 = value;
  }
};
template <>
struct StoreArg<bool> {
  void operator()(weval_req_arg_t* arg, bool value) {
    arg->specialize = 1;
    arg->ty = weval_req_arg_i32;
    arg->u.i32 = value ? 1 : 0;
  }
};
template <>
struct StoreArg<uint64_t> {
  void operator()(weval_req_arg_t* arg, uint64_t value) {
    arg->specialize = 1;
    arg->ty = weval_req_arg_i64;
    arg->u.i64 = value;
  }
};
template <>
struct StoreArg<float> {
  void operator()(weval_req_arg_t* arg, float value) {
    arg->specialize = 1;
    arg->ty = weval_req_arg_f32;
    arg->u.f32 = value;
  }
};
template <>
struct StoreArg<double> {
  void operator()(weval_req_arg_t* arg, double value) {
    arg->specialize = 1;
    arg->ty = weval_req_arg_f64;
    arg->u.f64 = value;
  }
};
template <typename T>
struct StoreArg<T*> {
  void operator()(weval_req_arg_t* arg, T* value) {
    static_assert(sizeof(T*) == 4, "Only 32-bit Wasm supported");
    arg->specialize = 1;
    arg->ty = weval_req_arg_i32;
    arg->u.i32 = reinterpret_cast<uint32_t>(value);
  }
};
template <typename T>
struct StoreArg<T&> {
  void operator()(weval_req_arg_t* arg, T& value) { StoreArg<T*>(arg, &value); }
};
template <typename T>
struct StoreArg<const T*> {
  void operator()(weval_req_arg_t* arg, const T* value) {
    static_assert(sizeof(const T*) == 4, "Only 32-bit Wasm supported");
    arg->specialize = 1;
    arg->ty = weval_req_arg_i32;
    arg->u.i32 = reinterpret_cast<uint32_t>(value);
  }
};

template <typename... Args>
struct StoreArgs {};

template <>
struct StoreArgs<> {
  void operator()(weval_req_arg_t* args) {}
};

template <typename T, typename... Rest>
struct StoreArgs<Specialize<T>, Rest...> {
  void operator()(weval_req_arg_t* args, Specialize<T> arg0, Rest... rest) {
    StoreArg<T>()(args, arg0.value);
    StoreArgs<Rest...>()(args + 1, rest...);
  }
};

template <typename T, typename... Rest>
struct StoreArgs<RuntimeArg<T>, Rest...> {
  void operator()(weval_req_arg_t* args, RuntimeArg<T> arg0, Rest... rest) {
    args[0].specialize = 0;
    StoreArgs<Rest...>()(args + 1, rest...);
  }
};

}  // namespace impl

template <typename Ret, typename... Args, typename... WrappedArgs>
weval_req_t* weval(impl::FuncPtr<Ret, Args...>* dest,
                   impl::FuncPtr<Ret, Args...> generic, WrappedArgs... args) {
  weval_req_t* req = (weval_req_t*)malloc(sizeof(weval_req_t));
  if (!req) {
    return nullptr;
  }
  uint32_t nargs = sizeof...(Args);
  weval_req_arg_t* arg_storage =
      (weval_req_arg_t*)malloc(sizeof(weval_req_arg_t) * nargs);
  if (!arg_storage) {
    return nullptr;
  }
  impl::StoreArgs<WrappedArgs...>()(arg_storage, args...);

  req->func = (weval_func_t)generic;
  req->args = arg_storage;
  req->nargs = nargs;
  req->specialized = (weval_func_t*)dest;

  weval_request(req);

  return req;
}

inline void free(weval_req_t* req) { weval_free(req); }

}  // namespace weval

#endif  // __cplusplus
