/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/codegen/WasmEmitter.h"

#include "mozilla/Casting.h"

namespace js::jit::wasm32 {

void WasmEmitter::emitI32Const(int32_t val) {
  emit(WasmOp::I32Const);
  emitVarI32(val);
}

void WasmEmitter::emitF32Const(float val) {
  emit(WasmOp::F32Const);
  emitFloat32(val);
}

void WasmEmitter::emitF64Const(double val) {
  emit(WasmOp::F64Const);
  emitFloat64(val);
}

void WasmEmitter::emitI32Store(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Store);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Store8(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Store8);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Store16(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Store16);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Load(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Load);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Load8U(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Load8U);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Load8S(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Load8S);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Load16U(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Load16U);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI32Load16S(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I32Load16S);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI64Load(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I64Load);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI64Store(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::I64Store);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitF32Load(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::F32Load);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitF32Store(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::F32Store);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitF64Load(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::F64Load);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitF64Store(uint32_t offset, uint32_t alignment) {
  emit(WasmOp::F64Store);
  emitMemArg(alignment, offset);
}

void WasmEmitter::emitI64Eq() { emit(WasmOp::I64Eq); }

void WasmEmitter::emitI64Eqz() { emit(WasmOp::I64Eqz); }

void WasmEmitter::emitI64Neq() { emit(WasmOp::I64Ne); }

void WasmEmitter::emitI32Add() { emit(WasmOp::I32Add); }

void WasmEmitter::emitI32Sub() { emit(WasmOp::I32Sub); }

void WasmEmitter::emitI32Mul() { emit(WasmOp::I32Mul); }

void WasmEmitter::emitI32DivU() { emit(WasmOp::I32DivU); }

void WasmEmitter::emitI32DivS() { emit(WasmOp::I32DivS); }

void WasmEmitter::emitI32RemU() { emit(WasmOp::I32RemU); }

void WasmEmitter::emitI32RemS() { emit(WasmOp::I32RemS); }

void WasmEmitter::emitI32And() { emit(WasmOp::I32And); }

void WasmEmitter::emitI32Or() { emit(WasmOp::I32Or); }

void WasmEmitter::emitI32Xor() { emit(WasmOp::I32Xor); }

void WasmEmitter::emitI32Shl() { emit(WasmOp::I32Shl); }

void WasmEmitter::emitI32ShrU() { emit(WasmOp::I32ShrU); }

void WasmEmitter::emitI32ShrS() { emit(WasmOp::I32ShrS); }

void WasmEmitter::emitI32Eq() { emit(WasmOp::I32Eq); }

void WasmEmitter::emitI32Neq() { emit(WasmOp::I32Ne); }

void WasmEmitter::emitI32Eqz() { emit(WasmOp::I32Eqz); }

void WasmEmitter::emitI32Ltu() { emit(WasmOp::I32LtU); }

void WasmEmitter::emitI32Lts() { emit(WasmOp::I32LtS); }

void WasmEmitter::emitI32LeU() { emit(WasmOp::I32LeU); }

void WasmEmitter::emitI32LeS() { emit(WasmOp::I32LeS); }

void WasmEmitter::emitI32GeU() { emit(WasmOp::I32GeU); }

void WasmEmitter::emitI32GeS() { emit(WasmOp::I32GeS); }

void WasmEmitter::emitI32GtU() { emit(WasmOp::I32GtU); }

void WasmEmitter::emitI32GtS() { emit(WasmOp::I32GtS); }

void WasmEmitter::emitI32WrapI64() { emit(WasmOp::I32WrapI64); }

void WasmEmitter::emitI32TruncF64S() { emit(WasmOp::I32TruncF64S); }

void WasmEmitter::emitI64ExtendI32S() { emit(WasmOp::I64ExtendI32S); }

void WasmEmitter::emitI64ExtendI32U() { emit(WasmOp::I64ExtendI32U); }

void WasmEmitter::emitI64Add() { emit(WasmOp::I64Add); }

void WasmEmitter::emitI64Sub() { emit(WasmOp::I64Sub); }

void WasmEmitter::emitI64Mul() { emit(WasmOp::I64Mul); }

void WasmEmitter::emitI64Or() { emit(WasmOp::I64Or); }

void WasmEmitter::emitI64And() { emit(WasmOp::I64And); }

void WasmEmitter::emitI64Xor() { emit(WasmOp::I64Xor); }

void WasmEmitter::emitI64ShrU() { emit(WasmOp::I64ShrU); }
void WasmEmitter::emitI64ShrS() { emit(WasmOp::I64ShrS); }

void WasmEmitter::emitI64Shl() { emit(WasmOp::I64Shl); }

void WasmEmitter::emitI64GtS() { emit(WasmOp::I64GtS); }
void WasmEmitter::emitI64GtU() { emit(WasmOp::I64GtU); }
void WasmEmitter::emitI64GeS() { emit(WasmOp::I64GeS); }
void WasmEmitter::emitI64GeU() { emit(WasmOp::I64GeU); }

void WasmEmitter::emitI64LtS() { emit(WasmOp::I64LtS); }
void WasmEmitter::emitI64LtU() { emit(WasmOp::I64LtU); }
void WasmEmitter::emitI64LeS() { emit(WasmOp::I64LeS); }
void WasmEmitter::emitI64LeU() { emit(WasmOp::I64LeU); }

void WasmEmitter::emitI64Const(int64_t val) {
  emit(WasmOp::I64Const);
  emitVarI64(val);
}

void WasmEmitter::emitIf() { emit(WasmOp::If); }

void WasmEmitter::emitEmptyBlockType() { emit(WasmTypeCode::BlockVoid); }

void WasmEmitter::emitF32Add() { emit(WasmOp::F32Add); }
void WasmEmitter::emitF32Sub() { emit(WasmOp::F32Sub); }
void WasmEmitter::emitF32Mul() { emit(WasmOp::F32Mul); }
void WasmEmitter::emitF32Div() { emit(WasmOp::F32Div); }

void WasmEmitter::emitF64Add() { emit(WasmOp::F64Add); }
void WasmEmitter::emitF64Sub() { emit(WasmOp::F64Sub); }
void WasmEmitter::emitF64Mul() { emit(WasmOp::F64Mul); }
void WasmEmitter::emitF64Div() { emit(WasmOp::F64Div); }

void WasmEmitter::emitI32ReinterpretF32() { emit(WasmOp::I32ReinterpretF32); }
void WasmEmitter::emitI64ReinterpretF64() { emit(WasmOp::I64ReinterpretF64); }
void WasmEmitter::emitF64ReinterpretI64() { emit(WasmOp::F64ReinterpretI64); }
void WasmEmitter::emitF64ConvertI32S() { emit(WasmOp::F64ConvertI32S); }
void WasmEmitter::emitF64ConvertI64S() { emit(WasmOp::F64ConvertI64S); }
void WasmEmitter::emitF64CopySign() { emit(WasmOp::F64CopySign); }

void WasmEmitter::emitF32Nearest() { emit(WasmOp::F32Nearest); }
void WasmEmitter::emitF64Nearest() { emit(WasmOp::F64Nearest); }

void WasmEmitter::emitF32Neg() { emit(WasmOp::F32Neg); }
void WasmEmitter::emitF64Neg() { emit(WasmOp::F64Neg); }

void WasmEmitter::emitF32Sqrt() { emit(WasmOp::F32Sqrt); }
void WasmEmitter::emitF64Sqrt() { emit(WasmOp::F64Sqrt); }

void WasmEmitter::emitF32Trunc() { emit(WasmOp::F32Trunc); }
void WasmEmitter::emitF64Trunc() { emit(WasmOp::F64Trunc); }

void WasmEmitter::emitF32Floor() { emit(WasmOp::F32Floor); }
void WasmEmitter::emitF64Floor() { emit(WasmOp::F64Floor); }

void WasmEmitter::emitF32Lt() { emit(WasmOp::F32Lt); }
void WasmEmitter::emitF32Le() { emit(WasmOp::F32Le); }
void WasmEmitter::emitF32Gt() { emit(WasmOp::F32Gt); }
void WasmEmitter::emitF32Ge() { emit(WasmOp::F32Ge); }
void WasmEmitter::emitF32Eq() { emit(WasmOp::F32Eq); }
void WasmEmitter::emitF32Neq() { emit(WasmOp::F32Ne); }

void WasmEmitter::emitF32Min() { emit(WasmOp::F32Min); }
void WasmEmitter::emitF32Max() { emit(WasmOp::F32Max); }

void WasmEmitter::emitF64Lt() { emit(WasmOp::F64Lt); }
void WasmEmitter::emitF64Le() { emit(WasmOp::F64Le); }
void WasmEmitter::emitF64Gt() { emit(WasmOp::F64Gt); }
void WasmEmitter::emitF64Ge() { emit(WasmOp::F64Ge); }
void WasmEmitter::emitF64Eq() { emit(WasmOp::F64Eq); }
void WasmEmitter::emitF64Neq() { emit(WasmOp::F64Ne); }

void WasmEmitter::emitF64Min() { emit(WasmOp::F64Min); }
void WasmEmitter::emitF64Max() { emit(WasmOp::F64Max); }

void WasmEmitter::emitGlobalGet(uint32_t globalIndex) {
  emit(WasmOp::GlobalGet);
  emitVarU32(globalIndex);
}

void WasmEmitter::emitGlobalSet(uint32_t globalIndex) {
  emit(WasmOp::GlobalSet);
  emitVarU32(globalIndex);
}

void WasmEmitter::emitLocalGet(uint32_t localIndex) {
  emit(WasmOp::LocalGet);
  emitVarU32(localIndex);
}

void WasmEmitter::emitLocalSet(uint32_t localIndex) {
  emit(WasmOp::LocalSet);
  emitVarU32(localIndex);
}

void WasmEmitter::emitTableSet() {
  emit(WasmOp::TableSet);
  emit(0x00);
}

void WasmEmitter::emitTableSize() {
  emit(WasmOp::MiscPrefix);
  emit(WasmMiscOp::TableSize);
  emit(0x00);
}

void WasmEmitter::emitTableGrow() {
  emit(WasmOp::MiscPrefix);
  emit(WasmMiscOp::TableGrow);
  emit(0x00);
}

void WasmEmitter::emitRefFunc(uint32_t functionIndex) {
  emit(WasmOp::RefFunc);
  emitVarU32(functionIndex);
}

void WasmEmitter::emitRefNullFunc() {
  emit(WasmOp::RefNull);
  emit(WasmTypeCode::FuncRef);
}

void WasmEmitter::emitDrop() { emit(WasmOp::Drop); }

void WasmEmitter::emitSelect() { emit(WasmOp::SelectNumeric); }

void WasmEmitter::emitUnreachable() { emit(WasmOp::Unreachable); }

void WasmEmitter::emitCall(uint32_t functionIndex) {
  emit(WasmOp::Call);
  emitVarU32(functionIndex);
}

void WasmEmitter::emitCallIndirect(uint32_t typeIndex, uint32_t tableIndex) {
  emit(WasmOp::CallIndirect);
  emitVarU32(typeIndex);
  emitVarU32(tableIndex);
}

void WasmEmitter::emitReturnCallIndirect(uint32_t typeIndex,
                                         uint32_t tableIndex) {
  emit(WasmOp::ReturnCallIndirect);
  emitVarU32(typeIndex);
  emitVarU32(tableIndex);
}

void WasmEmitter::emitReturn() { emit(WasmOp::Return); }

void WasmEmitter::emitBr(uint32_t blockIndex) {
  emit(WasmOp::Br);
  emitVarU32(blockIndex);
}

void WasmEmitter::emitBrIf(uint32_t blockIndex) {
  emit(WasmOp::BrIf);
  emitVarU32(blockIndex);
}

void WasmEmitter::emitBrTable(const std::vector<uint32_t>& indices) {
  emit(WasmOp::BrTable);
  emitVarU32(indices.size() - 1);
  for (const auto& idx : indices) {
    emitVarU32(idx);
  }
}

void WasmEmitter::emitNop() { emit(WasmOp::Nop); }

void WasmEmitter::emitEnd() { emit(WasmOp::End); }

void WasmEmitter::emitMagic() { emit({0x00, 0x61, 0x73, 0x6D}); }

void WasmEmitter::emitVersion() { emit({0x01, 0x00, 0x00, 0x00}); }

std::vector<uint8_t> WasmEmitter::finalize() { return std::move(buffer_); }

void WasmEmitter::emitVarI32(int32_t val) {
  bool done;
  do {
    uint8_t byte = val & 0x7f;
    val >>= 7;
    done = ((val == 0) && !(byte & 0x40)) || ((val == -1) && (byte & 0x40));
    if (!done) {
      byte |= 0x80;
    }
    emit(byte);
  } while (!done);
}

void WasmEmitter::emitVarU32(uint32_t val) {
  do {
    uint8_t byte = val & 0x7f;
    val >>= 7;
    if (val != 0) {
      byte |= 0x80;
    }
    emit(byte);
  } while (val != 0);
}

std::size_t WasmEmitter::emitPatchableVarU32() {
  const std::size_t offset = currentOffset();
  emitVarU32(UINT32_MAX);
  return offset;
}

void WasmEmitter::patchVarU32(std::size_t offset, uint32_t val) {
  for (size_t i = 0; i < 5; ++i, val >>= 7) {
    uint8_t byte = val & 0x7f;
    if (i < 4) {
      byte |= 0x80;
    }
    buffer_[offset + i] = byte;
  }
}

void WasmEmitter::emitVarI64(int64_t val) {
  bool done;
  do {
    uint8_t byte = val & 0x7f;
    val >>= 7;
    done = ((val == 0) && !(byte & 0x40)) || ((val == -1) && (byte & 0x40));
    if (!done) {
      byte |= 0x80;
    }
    emit(byte);
  } while (!done);
}

std::size_t WasmEmitter::currentOffset() const { return buffer_.size(); }

void WasmEmitter::emitMemArg(uint32_t align, uint32_t offset) {
  emitVarU32(align);
  emitVarU32(offset);
}

void WasmEmitter::emitFloat32(float val) {
  static_assert(sizeof(float) == 4, "wasm float requirement");
  auto bitsRepresentationOfFloat = mozilla::BitwiseCast<uint32_t>(val);
  for (std::size_t i = 0; i < sizeof(float); ++i) {
    emit(static_cast<uint8_t>(bitsRepresentationOfFloat & 0xff));
    bitsRepresentationOfFloat >>= 8;
  }
}

void WasmEmitter::emitFloat64(double val) {
  static_assert(sizeof(double) == 8, "wasm double requirement");
  auto bitsRepresentationOfDouble = mozilla::BitwiseCast<uint64_t>(val);
  for (std::size_t i = 0; i < sizeof(double); ++i) {
    emit(static_cast<uint8_t>(bitsRepresentationOfDouble & 0xff));
    bitsRepresentationOfDouble >>= 8;
  }
}

void WasmEmitter::emit(std::vector<uint8_t> bytes) {
  buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
}

void WasmEmitter::emit(uint32_t bytes) {
  buffer_.push_back(static_cast<uint8_t>(bytes >> 24));
  buffer_.push_back(static_cast<uint8_t>(bytes >> 16));
  buffer_.push_back(static_cast<uint8_t>(bytes >> 8));
  buffer_.push_back(static_cast<uint8_t>(bytes >> 0));
}

}  // namespace js::jit::wasm32
