/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_wasm32_codegen_Wasm_emitter_h
#define jit_wasm32_codegen_Wasm_emitter_h

#include <cstdint>
#include <vector>

namespace js::jit::wasm32 {

enum class ValueType : uint8_t {
  I32 = 0x7f,      // SLEB128(-0x01)
  I64 = 0x7e,      // SLEB128(-0x02)
  F32 = 0x7d,      // SLEB128(-0x03)
  F64 = 0x7c,      // SLEB128(-0x04)
  FuncRef = 0x70,  // SLEB128(-0x10)
};

enum class SectionId : uint8_t {
  Custom = 0,
  Type = 1,
  Import = 2,
  Function = 3,
  Table = 4,
  Memory = 5,
  Global = 6,
  Export = 7,
  Start = 8,
  Elem = 9,
  Code = 10,
  Data = 11,
  DataCount = 12,
};

enum class ImportKind : uint8_t {
  Func = 0x00,
  Table = 0x01,
  Memory = 0x02,
  Global = 0x03,
};

enum class Mutability : uint8_t { Const = 0x00, Var = 0x01 };

enum class RefType : uint8_t { Funcref = 0x70, Externref = 0x6f };

enum class WasmOp : uint8_t;

class WasmEmitter {
 public:
  void emitI32Const(int32_t val);
  void emitF32Const(float val);
  void emitF64Const(double val);
  void emitI32Store(uint32_t offset, uint32_t alignment);
  void emitI32Store8(uint32_t offset, uint32_t alignment);
  void emitI32Store16(uint32_t offset, uint32_t alignment);
  void emitI32Load(uint32_t offset, uint32_t alignment);
  void emitI32Load8U(uint32_t offset, uint32_t alignment);
  void emitI32Load8S(uint32_t offset, uint32_t alignment);
  void emitI32Load16U(uint32_t offset, uint32_t alignment);
  void emitI32Load16S(uint32_t offset, uint32_t alignment);
  void emitI64Load(uint32_t offset, uint32_t alignment);
  void emitI64Store(uint32_t offset, uint32_t alignment);
  void emitF32Load(uint32_t offset, uint32_t alignment);
  void emitF32Store(uint32_t offset, uint32_t alignment);
  void emitF64Load(uint32_t offset, uint32_t alignment);
  void emitF64Store(uint32_t offset, uint32_t alignment);
  void emitI64Eq();
  void emitI64Eqz();
  void emitI64Neq();
  void emitI32Add();
  void emitI32Sub();
  void emitI32Mul();
  void emitI32DivU();
  void emitI32DivS();
  void emitI32RemU();
  void emitI32RemS();
  void emitI32And();
  void emitI32Or();
  void emitI32Xor();
  void emitI32Shl();
  void emitI32ShrU();
  void emitI32ShrS();
  void emitI32Eq();
  void emitI32Neq();
  void emitI32Eqz();
  void emitI32Ltu();
  void emitI32Lts();
  void emitI32LeU();
  void emitI32LeS();
  void emitI32GeU();
  void emitI32GeS();
  void emitI32GtU();
  void emitI32GtS();
  void emitI32WrapI64();
  void emitI32TruncF64S();
  void emitI64ExtendI32S();
  void emitI64ExtendI32U();
  void emitI64Add();
  void emitI64Sub();
  void emitI64Mul();
  void emitI64Or();
  void emitI64And();
  void emitI64Xor();
  void emitI64ShrU();
  void emitI64ShrS();
  void emitI64Shl();
  void emitI64GeS();
  void emitI64GeU();
  void emitI64LtS();
  void emitI64LtU();
  void emitI64LeS();
  void emitI64LeU();
  void emitI64GtS();
  void emitI64GtU();
  void emitI64Const(int64_t val);
  void emitIf();
  void emitEmptyBlockType();
  void emitF32Add();
  void emitF32Sub();
  void emitF32Mul();
  void emitF32Div();
  void emitF64Add();
  void emitF64Sub();
  void emitF64Mul();
  void emitF64Div();
  void emitI32ReinterpretF32();
  void emitI64ReinterpretF64();
  void emitF64ReinterpretI64();
  void emitF64ConvertI32S();
  void emitF64ConvertI64S();
  void emitF64CopySign();

  void emitF32Nearest();
  void emitF64Nearest();

  void emitF32Neg();
  void emitF64Neg();

  void emitF32Sqrt();
  void emitF64Sqrt();

  void emitF32Trunc();
  void emitF64Trunc();
  void emitF32Floor();
  void emitF64Floor();

  void emitF32Lt();
  void emitF32Le();
  void emitF32Gt();
  void emitF32Ge();
  void emitF32Eq();
  void emitF32Neq();
  void emitF32Min();
  void emitF32Max();

  void emitF64Lt();
  void emitF64Le();
  void emitF64Gt();
  void emitF64Ge();
  void emitF64Eq();
  void emitF64Neq();
  void emitF64Min();
  void emitF64Max();

  void emitSelect();
  void emitUnreachable();
  void emitCall(uint32_t functionIndex);
  void emitCallIndirect(uint32_t typeIndex, uint32_t tableIndex = 0);
  void emitReturnCallIndirect(uint32_t typeIndex, uint32_t tableIndex = 0);

  void emitGlobalGet(uint32_t globalIndex);
  void emitGlobalSet(uint32_t globalIndex);

  void emitLocalGet(uint32_t localIndex);
  void emitLocalSet(uint32_t localIndex);

  void emitTableSet();
  void emitTableSize();
  void emitTableGrow();

  void emitRefFunc(uint32_t functionIndex);
  void emitRefNullFunc();

  void emitDrop();

  void emitReturn();
  void emitBr(uint32_t blockIndex);
  void emitBrIf(uint32_t blockIndex);
  void emitBrTable(const std::vector<uint32_t>& indices);
  void emitNop();
  void emitEnd();

  void emitMagic();
  void emitVersion();

  std::vector<uint8_t> finalize();

  template <class T>
  void emit(T val) {
    buffer_.push_back(static_cast<uint8_t>(val));
  }
  void emit(std::vector<uint8_t> bytes);
  void emit(uint32_t bytes);

  void emitVarI32(int32_t val);
  void emitVarU32(uint32_t val);
  std::size_t emitPatchableVarU32();
  void patchVarU32(std::size_t offset, uint32_t val);
  void emitVarI64(int64_t val);
  std::size_t currentOffset() const;
  void emitMemArg(uint32_t align, uint32_t offset);

  void emitFloat32(float val);
  void emitFloat64(double val);

 private:
  std::vector<uint8_t> buffer_;
};

enum class WasmOp : uint8_t {
  // Control flow operators
  Unreachable = 0x00,
  Nop = 0x01,
  Block = 0x02,
  Loop = 0x03,
  If = 0x04,
  Else = 0x05,
  Try = 0x06,
  Catch = 0x07,
  Throw = 0x08,
  Rethrow = 0x09,
  End = 0x0b,
  Br = 0x0c,
  BrIf = 0x0d,
  BrTable = 0x0e,
  Return = 0x0f,

  // Call operators
  Call = 0x10,
  CallIndirect = 0x11,

  // Tail calls
  ReturnCall = 0x12,
  ReturnCallIndirect = 0x13,

  // Additional exception operators
  Delegate = 0x18,
  CatchAll = 0x19,

  // Parametric operators
  Drop = 0x1a,
  SelectNumeric = 0x1b,
  SelectTyped = 0x1c,

  // Variable access
  LocalGet = 0x20,
  LocalSet = 0x21,
  LocalTee = 0x22,
  GlobalGet = 0x23,
  GlobalSet = 0x24,
  TableGet = 0x25,
  TableSet = 0x26,

  // Memory-related operators
  I32Load = 0x28,
  I64Load = 0x29,
  F32Load = 0x2a,
  F64Load = 0x2b,
  I32Load8S = 0x2c,
  I32Load8U = 0x2d,
  I32Load16S = 0x2e,
  I32Load16U = 0x2f,
  I64Load8S = 0x30,
  I64Load8U = 0x31,
  I64Load16S = 0x32,
  I64Load16U = 0x33,
  I64Load32S = 0x34,
  I64Load32U = 0x35,
  I32Store = 0x36,
  I64Store = 0x37,
  F32Store = 0x38,
  F64Store = 0x39,
  I32Store8 = 0x3a,
  I32Store16 = 0x3b,
  I64Store8 = 0x3c,
  I64Store16 = 0x3d,
  I64Store32 = 0x3e,
  MemorySize = 0x3f,
  MemoryGrow = 0x40,

  // Constants
  I32Const = 0x41,
  I64Const = 0x42,
  F32Const = 0x43,
  F64Const = 0x44,

  // Comparison operators
  I32Eqz = 0x45,
  I32Eq = 0x46,
  I32Ne = 0x47,
  I32LtS = 0x48,
  I32LtU = 0x49,
  I32GtS = 0x4a,
  I32GtU = 0x4b,
  I32LeS = 0x4c,
  I32LeU = 0x4d,
  I32GeS = 0x4e,
  I32GeU = 0x4f,
  I64Eqz = 0x50,
  I64Eq = 0x51,
  I64Ne = 0x52,
  I64LtS = 0x53,
  I64LtU = 0x54,
  I64GtS = 0x55,
  I64GtU = 0x56,
  I64LeS = 0x57,
  I64LeU = 0x58,
  I64GeS = 0x59,
  I64GeU = 0x5a,
  F32Eq = 0x5b,
  F32Ne = 0x5c,
  F32Lt = 0x5d,
  F32Gt = 0x5e,
  F32Le = 0x5f,
  F32Ge = 0x60,
  F64Eq = 0x61,
  F64Ne = 0x62,
  F64Lt = 0x63,
  F64Gt = 0x64,
  F64Le = 0x65,
  F64Ge = 0x66,

  // Numeric operators
  I32Clz = 0x67,
  I32Ctz = 0x68,
  I32Popcnt = 0x69,
  I32Add = 0x6a,
  I32Sub = 0x6b,
  I32Mul = 0x6c,
  I32DivS = 0x6d,
  I32DivU = 0x6e,
  I32RemS = 0x6f,
  I32RemU = 0x70,
  I32And = 0x71,
  I32Or = 0x72,
  I32Xor = 0x73,
  I32Shl = 0x74,
  I32ShrS = 0x75,
  I32ShrU = 0x76,
  I32Rotl = 0x77,
  I32Rotr = 0x78,
  I64Clz = 0x79,
  I64Ctz = 0x7a,
  I64Popcnt = 0x7b,
  I64Add = 0x7c,
  I64Sub = 0x7d,
  I64Mul = 0x7e,
  I64DivS = 0x7f,
  I64DivU = 0x80,
  I64RemS = 0x81,
  I64RemU = 0x82,
  I64And = 0x83,
  I64Or = 0x84,
  I64Xor = 0x85,
  I64Shl = 0x86,
  I64ShrS = 0x87,
  I64ShrU = 0x88,
  I64Rotl = 0x89,
  I64Rotr = 0x8a,
  F32Abs = 0x8b,
  F32Neg = 0x8c,
  F32Ceil = 0x8d,
  F32Floor = 0x8e,
  F32Trunc = 0x8f,
  F32Nearest = 0x90,
  F32Sqrt = 0x91,
  F32Add = 0x92,
  F32Sub = 0x93,
  F32Mul = 0x94,
  F32Div = 0x95,
  F32Min = 0x96,
  F32Max = 0x97,
  F32CopySign = 0x98,
  F64Abs = 0x99,
  F64Neg = 0x9a,
  F64Ceil = 0x9b,
  F64Floor = 0x9c,
  F64Trunc = 0x9d,
  F64Nearest = 0x9e,
  F64Sqrt = 0x9f,
  F64Add = 0xa0,
  F64Sub = 0xa1,
  F64Mul = 0xa2,
  F64Div = 0xa3,
  F64Min = 0xa4,
  F64Max = 0xa5,
  F64CopySign = 0xa6,

  // Conversions
  I32WrapI64 = 0xa7,
  I32TruncF32S = 0xa8,
  I32TruncF32U = 0xa9,
  I32TruncF64S = 0xaa,
  I32TruncF64U = 0xab,
  I64ExtendI32S = 0xac,
  I64ExtendI32U = 0xad,
  I64TruncF32S = 0xae,
  I64TruncF32U = 0xaf,
  I64TruncF64S = 0xb0,
  I64TruncF64U = 0xb1,
  F32ConvertI32S = 0xb2,
  F32ConvertI32U = 0xb3,
  F32ConvertI64S = 0xb4,
  F32ConvertI64U = 0xb5,
  F32DemoteF64 = 0xb6,
  F64ConvertI32S = 0xb7,
  F64ConvertI32U = 0xb8,
  F64ConvertI64S = 0xb9,
  F64ConvertI64U = 0xba,
  F64PromoteF32 = 0xbb,

  // Reinterpretations
  I32ReinterpretF32 = 0xbc,
  I64ReinterpretF64 = 0xbd,
  F32ReinterpretI32 = 0xbe,
  F64ReinterpretI64 = 0xbf,

  // Sign extension
  I32Extend8S = 0xc0,
  I32Extend16S = 0xc1,
  I64Extend8S = 0xc2,
  I64Extend16S = 0xc3,
  I64Extend32S = 0xc4,

  // Reference types
  RefNull = 0xd0,
  RefIsNull = 0xd1,
  RefFunc = 0xd2,

  // Function references
  RefAsNonNull = 0xd3,
  BrOnNull = 0xd4,

  // GC (experimental)
  RefEq = 0xd5,

  FirstPrefix = 0xfa,
  GcPrefix = 0xfb,
  MiscPrefix = 0xfc,
  SimdPrefix = 0xfd,
  ThreadPrefix = 0xfe,
  MozPrefix = 0xff,
};

// Opcodes in the "miscellaneous" opcode space.
enum class WasmMiscOp {
  // Saturating float-to-int conversions
  I32TruncSatF32S = 0x00,
  I32TruncSatF32U = 0x01,
  I32TruncSatF64S = 0x02,
  I32TruncSatF64U = 0x03,
  I64TruncSatF32S = 0x04,
  I64TruncSatF32U = 0x05,
  I64TruncSatF64S = 0x06,
  I64TruncSatF64U = 0x07,

  // Bulk memory operations, per proposal as of February 2019.
  MemoryInit = 0x08,
  DataDrop = 0x09,
  MemoryCopy = 0x0a,
  MemoryFill = 0x0b,
  TableInit = 0x0c,
  ElemDrop = 0x0d,
  TableCopy = 0x0e,

  // Reftypes, per proposal as of February 2019.
  TableGrow = 0x0f,
  TableSize = 0x10,
  TableFill = 0x11,

  Limit
};

enum class WasmTypeCode : uint8_t {
  I32 = 0x7f,   // SLEB128(-0x01)
  I64 = 0x7e,   // SLEB128(-0x02)
  F32 = 0x7d,   // SLEB128(-0x03)
  F64 = 0x7c,   // SLEB128(-0x04)
  V128 = 0x7b,  // SLEB128(-0x05)

  I8 = 0x7a,   // SLEB128(-0x06)
  I16 = 0x79,  // SLEB128(-0x07)

  // A function pointer with any signature
  FuncRef = 0x70,  // SLEB128(-0x10)

  // A reference to any host value.
  ExternRef = 0x6f,  // SLEB128(-0x11)

  // A reference to a struct/array value.
  EqRef = 0x6d,  // SLEB128(-0x12)

  // Type constructor for nullable reference types.
  NullableRef = 0x6c,  // SLEB128(-0x14)

  // Type constructor for non-nullable reference types.
  Ref = 0x6b,  // SLEB128(-0x15)

  // Type constructors for rtt types.
  RttWithDepth = 0x69,  // SLEB128(-0x17)
  Rtt = 0x68,           // SLEB128(-0x18)

  // Type constructor for function types
  Func = 0x60,  // SLEB128(-0x20)

  // Type constructor for structure types - gc proposal
  Struct = 0x5f,  // SLEB128(-0x21)

  // Type constructor for array types - gc proposal
  Array = 0x5e,  // SLEB128(-0x22)

  // The 'empty' case of blocktype.
  BlockVoid = 0x40,  // SLEB128(-0x40)

  Limit = 0x80
};

constexpr const uint32_t WasmJitCodeDelimiterPattern = 0xccccccccu;
constexpr const uint32_t WasmJitCodeIndexPlaceHolderPattern = 0xffffffffu;

}  // namespace js::jit::wasm32

#endif  // jit_wasm32_codegen_Wasm_emitter_h
