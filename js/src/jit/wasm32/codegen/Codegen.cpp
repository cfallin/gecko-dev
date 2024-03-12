/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/wasm32/codegen/Codegen.h"

#include <algorithm>
#include <set>
#include <stack>
#include <utility>
#include <vector>

namespace js::jit::wasm32 {

namespace {

struct LayoutTreeNode {
  BasicBlock* block;
  LayoutTreeNode* parent;
  std::vector<std::unique_ptr<LayoutTreeNode>> childrenInOrder;
};

class Graph {
 public:
  explicit Graph(uint32_t numberOfVertex) : adjacentList(numberOfVertex) {}

  void addEdge(uint32_t from, uint32_t to) {
    MOZ_RELEASE_ASSERT(from < adjacentList.size());
    MOZ_RELEASE_ASSERT(to < adjacentList.size());

    adjacentList[from].push_back(to);
  }

  void removeEdge(uint32_t from, uint32_t to) {
    MOZ_RELEASE_ASSERT(from < adjacentList.size());
    MOZ_RELEASE_ASSERT(to < adjacentList.size());

    auto& adjacentListForFrom = adjacentList[from];
    auto it =
        std::find(adjacentListForFrom.begin(), adjacentListForFrom.end(), to);
    MOZ_RELEASE_ASSERT(it != adjacentListForFrom.end());
    std::iter_swap(it, adjacentListForFrom.end() - 1);
    adjacentListForFrom.pop_back();
  }

  void transpose() {
    std::vector<std::vector<uint32_t>> newAdjacentList(adjacentList.size());
    for (uint32_t from = 0; from < numVertex(); ++from) {
      for (const auto to : adjacentList[from]) {
        newAdjacentList[to].push_back(from);
      }
    }
    adjacentList = std::move(newAdjacentList);
  }

  uint32_t numVertex() const { return adjacentList.size(); }

  const std::vector<uint32_t>& adjacents(uint32_t from) const {
    MOZ_RELEASE_ASSERT(from < adjacentList.size());
    return adjacentList[from];
  }

 private:
  std::vector<std::vector<uint32_t>> adjacentList;
};

struct LoopInfo {
  std::set<uint32_t> loopBody;
  uint32_t loopTail;

  bool contains(uint32_t blockId) const {
    return loopBody.find(blockId) != loopBody.end();
  }
};

struct LoopTailAndPredecessor {
  uint32_t tail;
  uint32_t loopHeadPredecessor;
};

#ifdef DEBUG
void printLayoutTree(const LayoutTreeNode* root) {
  if (root->block) {
    std::cout << root->block->id;
    return;
  }

  std::cout << "(";
  for (const auto& children : root->childrenInOrder) {
    printLayoutTree(children.get());
    std::cout << ", ";
  }
  std::cout << ")";
}
#endif

void postOrder(BasicBlock* root, std::set<BasicBlock*>* visited,
               std::vector<BasicBlock*>* order) {
  visited->insert(root);

  for (const auto& succ : root->successors) {
    if (visited->find(succ) == visited->end()) {
      postOrder(succ, visited, order);
    }
  }
  order->push_back(root);
}

std::vector<BasicBlock*> reversePostOrder(BasicBlock* root) {
  std::set<BasicBlock*> visited;
  std::vector<BasicBlock*> order;
  postOrder(root, &visited, &order);
  std::reverse(order.begin(), order.end());
  return order;
}

uint32_t getOffset(LayoutTreeNode* current, const uint32_t targetBlockId) {
  const auto* enclosingVector = &current->parent->childrenInOrder;
  auto currentBlockPosition =
      std::find_if(enclosingVector->cbegin(), enclosingVector->cend(),
                   [current](const auto& x) {
                     return x->block && x->block->id == current->block->id;
                   });
  auto targetPosition =
      std::find_if(enclosingVector->cbegin(), enclosingVector->cend(),
                   [targetBlockId](const auto& x) {
                     return x->block && x->block->id == targetBlockId;
                   });

  // Case 1: current level jump.
  if (targetPosition != enclosingVector->cend()) {
    // Case 1.1: jump to loop head.
    if (targetPosition == enclosingVector->cbegin()) {
      return std::distance(currentBlockPosition, enclosingVector->cend()) - 1;
    }

    // Case 1.2: jump to non loop head but still at the current level.
    return std::distance(currentBlockPosition, targetPosition) - 1;
  }

  // Case 2: current level jump to inner loop head.
  targetPosition = std::find_if(
      enclosingVector->cbegin(), enclosingVector->cend(),
      [targetBlockId](const auto& x) {
        return x->block == nullptr && x->childrenInOrder.front()->block &&
               x->childrenInOrder.front()->block->id == targetBlockId;
      });
  if (targetPosition != enclosingVector->cend()) {
    return std::distance(currentBlockPosition, targetPosition) - 1;
  }

  // Case 3: distant jump to an other level.
  uint32_t offset =
      std::distance(currentBlockPosition, enclosingVector->cend());
  while (current) {
    enclosingVector = &current->parent->parent->childrenInOrder;
    MOZ_RELEASE_ASSERT(!enclosingVector->empty());

    auto parentPosition = std::find_if(
        enclosingVector->cbegin(), enclosingVector->cend(),
        [current](const auto& x) { return x.get() == current->parent; });
    auto targetPosition =
        std::find_if(parentPosition, enclosingVector->cend(),
                     [targetBlockId](const auto& x) {
                       return x->block && x->block->id == targetBlockId;
                     });
    if (targetPosition != enclosingVector->cend()) {
      return offset + std::distance(parentPosition, targetPosition) - 1;
    }

    current = current->parent;
    offset += std::distance(parentPosition, enclosingVector->cend());
  }

  MOZ_CRASH();
  return UINT32_MAX;
}

void updateBranchInstruction(LayoutTreeNode* current) {
  auto* basicBlock = current->block;
  if (basicBlock->instructions.back().is<BR_INSTR>()) {
    auto& instruction = basicBlock->instructions.back().as<BR_INSTR>();
    instruction.blockIndex = getOffset(current, instruction.blockIndex);

    if (instruction.isConditional()) {
      *instruction.fallthroughBlockIndex =
          getOffset(current, *instruction.fallthroughBlockIndex);
    }
    return;
  }

  if (basicBlock->instructions.back().is<BR_TABLE>()) {
    auto& instruction = basicBlock->instructions.back().as<BR_TABLE>();
    for (uint32_t i = 0; i < instruction.blockIndices.size(); ++i) {
      instruction.blockIndices[i] =
          getOffset(current, instruction.blockIndices[i]);
    }
  }
}

void updateBranchTargets(LayoutTreeNode* root) {
  std::stack<LayoutTreeNode*> stack;

  // Visit all nodes and update br* instructions according to the layout.
  stack.push(root);
  while (!stack.empty()) {
    auto* current = stack.top();
    stack.pop();

    if (current->block) {
      updateBranchInstruction(current);
    }
    for (auto& child : current->childrenInOrder) {
      stack.push(child.get());
    }
  }
}

std::set<uint32_t> reachableNodes(const Graph& graph, uint32_t from) {
  std::vector<bool> visited(graph.numVertex(), false);
  std::stack<uint32_t> stack;
  std::set<uint32_t> reachedNodes;

  stack.push(from);

  while (!stack.empty()) {
    auto current = stack.top();
    stack.pop();

    if (!visited[current]) {
      reachedNodes.insert(current);
      visited[current] = true;
    }

    for (const auto to : graph.adjacents(current)) {
      if (!visited[to]) {
        stack.push(to);
      }
    }
  }

  return reachedNodes;
}

std::map<uint32_t, LoopTailAndPredecessor> buildLoopInfoHelper(
    const Graph& graph, uint32_t from) {
  enum Action { Enter, Exit };
  enum VertexColor { White, Gray, Black };

  std::map<uint32_t, LoopTailAndPredecessor> loopInfo;
  std::vector<VertexColor> color(graph.numVertex(), VertexColor::White);
  std::vector<uint32_t> previous(graph.numVertex(), UINT32_MAX);

  std::stack<std::pair<uint32_t, Action>> stack;
  stack.push(std::make_pair(from, Action::Enter));

  // This is an iterative DFS with a simple modifications to track cycles.
  while (!stack.empty()) {
    auto current = stack.top();
    stack.pop();

    if (current.second == Action::Exit) {
      color[current.first] = VertexColor::Black;
      continue;
    }

    color[current.first] = VertexColor::Gray;
    stack.push(std::make_pair(current.first, Action::Exit));

    for (const auto child : graph.adjacents(current.first)) {
      if (color[child] == VertexColor::White) {
        stack.push(std::make_pair(child, Action::Enter));
        previous[child] = current.first;
      } else if (color[child] == VertexColor::Gray) {
        loopInfo[child].tail = current.first;
      }
    }
  }

  for (auto& iter : loopInfo) {
    iter.second.loopHeadPredecessor = previous[iter.first];
  }

  return loopInfo;
}

std::map<uint32_t, LoopInfo> buildLoopInfo(
    const BasicBlock* root,
    const std::map<uint32_t, std::unique_ptr<BasicBlock>>& blocks) {
  MOZ_RELEASE_ASSERT(!blocks.empty());

  // Build a graph from CFG.
  uint32_t maxBlockId =
      blocks.rbegin()->first + 1;  // map in c++ is sorted by keys.
  Graph graph(maxBlockId);
  for (const auto& iter : blocks) {
    for (const auto& successor : iter.second->successors) {
      graph.addEdge(iter.second->id, successor->id);
    }
  }

  // Find loop head, its predecessor and loop tail for every loop in the CFG.
  auto loopHeadToTailAndPredecessor = buildLoopInfoHelper(graph, root->id);

  // Build a transposed graph to find loop bodies later in the next step
  // using the Kosaraju's algorithm:
  // https://en.wikipedia.org/wiki/Kosaraju%27s_algorithm.
  graph.transpose();

  // Find loop bodies.
  std::map<uint32_t, LoopInfo> loopInfo;
  for (const auto& [head, tailAndPred] : loopHeadToTailAndPredecessor) {
    std::set<uint32_t> loopBody;

    // Cut the loop above the its head to prevent adding enclosing loop body.
    graph.removeEdge(head, tailAndPred.loopHeadPredecessor);

    loopInfo[head].loopBody = reachableNodes(graph, head);
    loopInfo[head].loopTail = tailAndPred.tail;

    // Restore the loop link.
    graph.addEdge(head, tailAndPred.tail);
  }

  return loopInfo;
}

std::vector<BasicBlock*> reversePostOrderWithLoopCompaction(
    BasicBlock* root, const std::map<uint32_t, LoopInfo>& loopMapInfo) {
  // Here we sort RPO such that loop bodies be continuously.
  // This is needed for creating proper wasm loop blocks for the loops in CFG.

  auto order = reversePostOrder(root);
  std::vector<BasicBlock*> remainings;
  std::vector<BasicBlock*> inLoop;
  for (uint32_t i = 0; i < order.size(); ++i) {
    auto it = loopMapInfo.find(order[i]->id);
    if (it == loopMapInfo.end()) {
      continue;
    }

    const LoopInfo& currentLoop = it->second;
    if (order[i]->id == currentLoop.loopTail) {
      continue;
    }

    remainings.clear();
    inLoop.clear();
    uint32_t j = i + 1;
    for (; j < order.size() && order[j]->id != currentLoop.loopTail; ++j) {
      if (!currentLoop.contains(order[j]->id)) {
        remainings.push_back(order[j]);
      } else {
        inLoop.push_back(order[j]);
      }
    }

    if (j < order.size() && order[j]->id == currentLoop.loopTail) {
      inLoop.push_back(order[j]);
    }

    auto endLoopIter =
        std::copy(inLoop.begin(), inLoop.end(), order.begin() + i + 1);
    std::copy(remainings.begin(), remainings.end(), endLoopIter);
  }

  return order;
}

std::unique_ptr<LayoutTreeNode> constructLayoutTree(
    BasicBlock* cfgRoot,
    const std::map<uint32_t, std::unique_ptr<BasicBlock>>& blocks) {
  const auto loopMapInfo = buildLoopInfo(cfgRoot, blocks);
  auto isLoopHead = [&loopMapInfo](const BasicBlock* block) {
    return loopMapInfo.find(block->id) != loopMapInfo.end();
  };

  std::stack<std::pair<LayoutTreeNode*, uint32_t>> stackLoopNodes;
  auto root =
      std::make_unique<LayoutTreeNode>(LayoutTreeNode{nullptr, nullptr, {}});
  stackLoopNodes.push({root.get(), UINT32_MAX});
  for (auto* block : reversePostOrderWithLoopCompaction(cfgRoot, loopMapInfo)) {
    auto* enclosingLoop = stackLoopNodes.top().first;
    if (isLoopHead(block)) {
      auto loopNode = std::make_unique<LayoutTreeNode>(
          LayoutTreeNode{nullptr, enclosingLoop, {}});
      auto* loopNodePtr = loopNode.get();
      enclosingLoop->childrenInOrder.push_back(std::move(loopNode));

      auto node = std::make_unique<LayoutTreeNode>(
          LayoutTreeNode{block, loopNodePtr, {}});
      loopNodePtr->childrenInOrder.push_back(std::move(node));

      if (loopMapInfo.at(block->id).loopTail != block->id) {
        stackLoopNodes.push({loopNodePtr, loopMapInfo.at(block->id).loopTail});
      }
      continue;
    }

    auto node = std::make_unique<LayoutTreeNode>(
        LayoutTreeNode{block, enclosingLoop, {}});
    enclosingLoop->childrenInOrder.push_back(std::move(node));

    if (stackLoopNodes.top().second == block->id) {
      stackLoopNodes.pop();
    }
  }

  return root;
}

}  // namespace

void Wasm32CodeGenerator::emitLocalsInfo(
    std::vector<WasmLocalType> localTypes) {
  emitter.emitVarU32(localTypes.size());

  if (localTypes.empty()) {
    return;
  }

  for (const auto localType : localTypes) {
    emitter.emitVarU32(1);
    auto typeCode = WasmTypeCode::Limit;
    switch (localType) {
      case WasmLocalType::I32: {
        typeCode = WasmTypeCode::I32;
        break;
      }
      case WasmLocalType::I64: {
        typeCode = WasmTypeCode::I64;
        break;
      }
      case WasmLocalType::F32: {
        typeCode = WasmTypeCode::F32;
        break;
      }
      case WasmLocalType::F64: {
        typeCode = WasmTypeCode::F64;
        break;
      }
    }
    emitter.emit(typeCode);
  }
}

void Wasm32CodeGenerator::emitLayoutTree(LayoutTreeNode* root,
                                         bool emitFirstAsLoop) {
  MOZ_RELEASE_ASSERT(!root->childrenInOrder.empty());
  if (emitFirstAsLoop) {
    emitter.emit(WasmOp::Loop);
  } else {
    emitter.emit(WasmOp::Block);
  }
  emitter.emit(WasmTypeCode::BlockVoid);

  for (std::size_t i = 1; i < root->childrenInOrder.size(); ++i) {
    emitter.emit(WasmOp::Block);
    emitter.emit(WasmTypeCode::BlockVoid);
  }

  for (const auto& node : root->childrenInOrder) {
    if (node->block) {
      for (const auto& instruction : node->block->instructions) {
        encodeInstruction(instruction);
      }
    } else {
      emitLayoutTree(node.get(), true);
    }

    emitter.emit(WasmOp::End);
  }
}

std::vector<uint8_t> Wasm32CodeGenerator::generateCode(
    FunctionDescription function) {
  if (function.root->empty()) {
    return {};
  }

  // Reserve 4 bytes place for storing index of this code in the
  // WebAssembly.Table. Reserving a dedicated space for index preventing us from
  // rewriting the code itself.
  emitter.emit(WasmJitCodeIndexPlaceHolderPattern);

  emitLocalsInfo(std::move(function.locals));

  auto layoutTree = constructLayoutTree(function.root, function.blocks);
  updateBranchTargets(layoutTree.get());
  emitLayoutTree(layoutTree.get(), false);

  emitter.emitUnreachable();  // To satisfy wasm type checker, we return early
                              // anyway.
  emitter.emit(WasmOp::End);  // End for the whole function.

  // Emit the special pattern as a delimeter to separate multiple codes in one
  // buffer.
  emitter.emit(WasmJitCodeDelimiterPattern);

  return emitter.finalize();
}

void Wasm32CodeGenerator::encodeInstruction(
    const WasmInstruction& instruction) {
  instruction.match(
      [this](const I32_CONST& instr) { emitter.emitI32Const(instr.value); },
      [this](const I32_STORE& instr) {
        emitter.emitI32Store(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_STORE_8& instr) {
        emitter.emitI32Store8(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_STORE_16& instr) {
        emitter.emitI32Store16(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_LOAD& instr) {
        emitter.emitI32Load(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_LOAD_8U& instr) {
        emitter.emitI32Load8U(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_LOAD_8S& instr) {
        emitter.emitI32Load8S(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_LOAD_16U& instr) {
        emitter.emitI32Load16U(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_LOAD_16S& instr) {
        emitter.emitI32Load16S(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I32_ADD& instr) { emitter.emitI32Add(); },
      [this](const I32_SUB& instr) { emitter.emitI32Sub(); },
      [this](const I32_MUL& instr) { emitter.emitI32Mul(); },
      [this](const I32_DIV_U& instr) { emitter.emitI32DivU(); },
      [this](const I32_DIV_S& instr) { emitter.emitI32DivS(); },
      [this](const I32_REM_U& instr) { emitter.emitI32RemU(); },
      [this](const I32_REM_S& instr) { emitter.emitI32RemS(); },
      [this](const I32_AND& instr) { emitter.emitI32And(); },
      [this](const I32_OR& instr) { emitter.emitI32Or(); },
      [this](const I32_XOR& instr) { emitter.emitI32Xor(); },
      [this](const I32_SHL& instr) { emitter.emitI32Shl(); },
      [this](const I32_SHR_U& instr) { emitter.emitI32ShrU(); },
      [this](const I32_SHR_S& instr) { emitter.emitI32ShrS(); },
      [this](const I32_EQ& instr) { emitter.emitI32Eq(); },
      [this](const I32_NEQ& instr) { emitter.emitI32Neq(); },
      [this](const I32_EQZ& instr) { emitter.emitI32Eqz(); },
      [this](const I32_LT_U& instr) { emitter.emitI32Ltu(); },
      [this](const I32_LT_S& instr) { emitter.emitI32Lts(); },
      [this](const I32_LE_U& instr) { emitter.emitI32LeU(); },
      [this](const I32_LE_S& instr) { emitter.emitI32LeS(); },
      [this](const I32_GE_U& instr) { emitter.emitI32GeU(); },
      [this](const I32_GE_S& instr) { emitter.emitI32GeS(); },
      [this](const I32_GT_U& instr) { emitter.emitI32GtU(); },
      [this](const I32_GT_S& instr) { emitter.emitI32GtS(); },
      [this](const I64_CONST& instr) { emitter.emitI64Const(instr.value); },
      [this](const I64_LOAD& instr) {
        emitter.emitI64Load(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I64_STORE& instr) {
        emitter.emitI64Store(instr.memarg.offset, instr.memarg.align);
      },
      [this](const I64_EQ& instr) { emitter.emitI64Eq(); },
      [this](const I64_EQZ& instr) { emitter.emitI64Eqz(); },
      [this](const I64_NEQ& instr) { emitter.emitI64Neq(); },
      [this](const I32_WRAP_I64& instr) { emitter.emitI32WrapI64(); },
      [this](const I32_TRUNC_F64_S& instr) { emitter.emitI32TruncF64S(); },
      [this](const I64_EXTEND_I32_S& instr) { emitter.emitI64ExtendI32S(); },
      [this](const I64_EXTEND_I32_U& instr) { emitter.emitI64ExtendI32U(); },
      [this](const I64_ADD& instr) { emitter.emitI64Add(); },
      [this](const I64_SUB& instr) { emitter.emitI64Sub(); },
      [this](const I64_MUL& instr) { emitter.emitI64Mul(); },
      [this](const I64_OR& instr) { emitter.emitI64Or(); },
      [this](const I64_AND& instr) { emitter.emitI64And(); },
      [this](const I64_XOR& instr) { emitter.emitI64Xor(); },
      [this](const I64_SHR_U& instr) { emitter.emitI64ShrU(); },
      [this](const I64_SHR_S& instr) { emitter.emitI64ShrS(); },
      [this](const I64_SHL& instr) { emitter.emitI64Shl(); },
      [this](const I64_GT_S& instr) { emitter.emitI64GtS(); },
      [this](const I64_GT_U& instr) { emitter.emitI64GtU(); },
      [this](const I64_GE_S& instr) { emitter.emitI64GeS(); },
      [this](const I64_GE_U& instr) { emitter.emitI64GeU(); },
      [this](const I64_LT_S& instr) { emitter.emitI64LtS(); },
      [this](const I64_LT_U& instr) { emitter.emitI64LtU(); },
      [this](const I64_LE_S& instr) { emitter.emitI64LeU(); },
      [this](const I64_LE_U& instr) { emitter.emitI64LeU(); },
      [this](const LOCAL_GET& instr) {
        emitter.emitLocalGet(instr.localIndex);
      },
      [this](const LOCAL_SET& instr) {
        emitter.emitLocalSet(instr.localIndex);
      },
      [this](const GLOBAL_GET& instr) {
        emitter.emitGlobalGet(instr.globalIndex);
      },
      [this](const GLOBAL_SET& instr) {
        emitter.emitGlobalSet(instr.globalIndex);
      },
      [this](const SELECT_INSTR& instr) { emitter.emitSelect(); },
      [this](const UNREACHABLE& instr) { emitter.emitUnreachable(); },
      [this](const DROP&) { emitter.emitDrop(); },
      [this](const CALL_INSTR& instr) {
        emitter.emitCall(instr.functionIndex);
      },
      [this](const CALL_INDIRECT& instr) {
        emitter.emitCallIndirect(instr.typeIndex, instr.tableIndex);
      },
      [this](const RETURN_CALL_INDIRECT& instr) {
        emitter.emitReturnCallIndirect(instr.typeIndex, instr.tableIndex);
      },
      [this](const RETURN_INSTR& instr) { emitter.emitReturn(); },
      [this](const BR_INSTR& instr) {
        if (instr.isConditional()) {
          emitter.emitBrIf(instr.blockIndex);
          emitter.emitBr(*instr.fallthroughBlockIndex);
        } else {
          emitter.emitBr(instr.blockIndex);
        }
      },
      [this](const BR_TABLE& instr) {
        emitter.emitBrTable(instr.blockIndices);
      },
      [this](const NOP_INSTR& instr) { emitter.emitNop(); },
      [this](const END_INSTR& instr) { emitter.emitEnd(); },
      [this](const F32_CONST& instr) { emitter.emitF32Const(instr.value); },
      [this](const F32_NEAREST& instr) { emitter.emitF32Nearest(); },
      [this](const F64_NEAREST& instr) { emitter.emitF64Nearest(); },
      [this](const F32_LT& instr) { emitter.emitF32Lt(); },
      [this](const F32_LE& instr) { emitter.emitF32Le(); },
      [this](const F32_GT& instr) { emitter.emitF32Gt(); },
      [this](const F32_GE& instr) { emitter.emitF32Ge(); },
      [this](const F32_EQ& instr) { emitter.emitF32Eq(); },
      [this](const F32_NEQ& instr) { emitter.emitF32Neq(); },
      [this](const F32_MIN& instr) { emitter.emitF32Min(); },
      [this](const F32_MAX& instr) { emitter.emitF32Max(); },
      [this](const F32_ADD& instr) { emitter.emitF32Add(); },
      [this](const F32_SUB& instr) { emitter.emitF32Sub(); },
      [this](const F32_MUL& instr) { emitter.emitF32Mul(); },
      [this](const F32_DIV& instr) { emitter.emitF32Div(); },
      [this](const F64_ADD& instr) { emitter.emitF64Add(); },
      [this](const F64_SUB& instr) { emitter.emitF64Sub(); },
      [this](const F64_MUL& instr) { emitter.emitF64Mul(); },
      [this](const F64_DIV& instr) { emitter.emitF64Div(); },
      [this](const I32_REINTERPRET_F32& instr) {
        emitter.emitI32ReinterpretF32();
      },
      [this](const I64_REINTERPRET_F64& instr) {
        emitter.emitI64ReinterpretF64();
      },
      [this](const F64_REINTERPRET_I64& instr) {
        emitter.emitF64ReinterpretI64();
      },
      [this](const F64_CONVERT_I32_S& instr) { emitter.emitF64ConvertI32S(); },
      [this](const F64_CONVERT_I64_S& instr) { emitter.emitF64ConvertI64S(); },
      [this](const F64_COPYSIGN& instr) { emitter.emitF64CopySign(); },
      [this](const F32_TRUNC& instr) { emitter.emitF32Trunc(); },
      [this](const F64_TRUNC& instr) { emitter.emitF64Trunc(); },
      [this](const F32_FLOOR& instr) { emitter.emitF32Floor(); },
      [this](const F64_FLOOR& instr) { emitter.emitF64Floor(); },
      [this](const F32_NEG& instr) { emitter.emitF32Neg(); },
      [this](const F64_NEG& instr) { emitter.emitF64Neg(); },
      [this](const F32_SQRT& instr) { emitter.emitF32Sqrt(); },
      [this](const F64_SQRT& instr) { emitter.emitF64Sqrt(); },
      [this](const F64_LT& instr) { emitter.emitF64Lt(); },
      [this](const F64_LE& instr) { emitter.emitF64Le(); },
      [this](const F64_GT& instr) { emitter.emitF64Gt(); },
      [this](const F64_GE& instr) { emitter.emitF64Ge(); },
      [this](const F64_EQ& instr) { emitter.emitF64Eq(); },
      [this](const F64_NEQ& instr) { emitter.emitF64Neq(); },
      [this](const F64_MIN& instr) { emitter.emitF64Min(); },
      [this](const F64_MAX& instr) { emitter.emitF64Max(); },
      [this](const F64_CONST& instr) { emitter.emitF64Const(instr.value); },
      [this](const F32_LOAD& instr) {
        emitter.emitF32Load(instr.memarg.offset, instr.memarg.align);
      },
      [this](const F32_STORE& instr) {
        emitter.emitF32Store(instr.memarg.offset, instr.memarg.align);
      },
      [this](const F64_LOAD& instr) {
        emitter.emitF64Load(instr.memarg.offset, instr.memarg.align);
      },
      [this](const F64_STORE& instr) {
        emitter.emitF64Store(instr.memarg.offset, instr.memarg.align);
      });
}

}  // namespace js::jit::wasm32
