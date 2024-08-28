#include "MemoryPool.h"
#include <assert.h>
#include <iostream>
MemoryPool::MemoryPool() {}

MemoryPool::~MemoryPool() {
  Slot *temp;
  while (BlockHead_) {
    temp = BlockHead_;
    BlockHead_ = BlockHead_->next;
    operator delete(
        reinterpret_cast<void *>(temp)); // 转化为void*就不用调用析构了
  }
  //   Slot *cur = BlockHead_;
  //   while (cur) {
  //     Slot *next = cur->next;
  //     // free(reinterpret_cast<void *>(cur));
  //     // 转化为 void 指针，是因为 void 类型不需要调用析构函数,只释放空间
  //     operator delete(reinterpret_cast<void *>(cur));
  //     cur = next;
  //   }
}

void MemoryPool::init(int size) {
  assert(size > 0);
  slotSize_ = size; // 小内存块的大小
  BlockHead_ = NULL;
  BlockLast_ = NULL;
  freeSlot_ = NULL;
  curBlockSlot_ = NULL;
}

inline size_t MemoryPool::pad(char *p, size_t align) {
  size_t result = reinterpret_cast<size_t>(p);
  return (align - result) % align;
}

Slot *MemoryPool::allocateBlock() {
  // 使用malloc从内存中要内存块，一次要一大块
  char *newBlock = reinterpret_cast<char *>(operator new(BlockSize));
  char *body = newBlock + sizeof(Slot *); // 大块再分块
  size_t padding = pad(body, static_cast<size_t>(slotSize_));

  Slot *useSlot;
  {
    //MutexLockGuard lock(mutexOther_);

    // 将新申请的大块内存连到原有block头部
    reinterpret_cast<Slot *>(newBlock)->next = BlockHead_;
    BlockHead_ = reinterpret_cast<Slot *>(newBlock);
    // 新申请的内存都是free的
    curBlockSlot_ = reinterpret_cast<Slot *>(body + padding);
    BlockLast_ = reinterpret_cast<Slot *>(newBlock + BlockSize - slotSize_ + 1);
    useSlot = curBlockSlot_;
    curBlockSlot_ += (slotSize_ >> 3); // 相当于处理8bit，算出字节数
  }
  return useSlot;
}

Slot *MemoryPool::nonfree() {
  if (curBlockSlot_ >= BlockLast_) {
    return allocateBlock();
  }
  Slot *useSlot;
  {
    //MutexLockGuard lock(mutexOther_);
    useSlot = curBlockSlot_;
    curBlockSlot_ += (slotSize_ >> 3);
  }
  return useSlot;
}

Slot *MemoryPool::allocate() {
  if (freeSlot_) {
    {
      //MutexLockGuard lock(mutexfreeSlot_);
      if (freeSlot_) {
        Slot *result = freeSlot_;
        freeSlot_ = freeSlot_->next;
        return result;
      }
    }
  }
  return nonfree();
}
inline void MemoryPool::release(Slot *p) {
  if (p) {
    //MutexLockGuard lock(mutexfreeSlot_);
    p->next = freeSlot_;
    freeSlot_ = p; // 头插
  }
}
namespace QueMemory {

// 单例模式封装
MemoryPool &getMemory(int id) {
  static MemoryPool pool_[64];
  return pool_[id];
}

void init() {
  for (int i = 0; i < 64; i++) {
    getMemory(i).init((i + 1) << 3); // 8--512
  }
}

void *useMemory(size_t size) {
  if (!size) {
    return nullptr;
  } else if (size > 512) {
    return operator new(size);
  }
  // 否则从内存池里取
  return reinterpret_cast<void *>(getMemory(((size + 7) >> 3) - 1).allocate());
}

void freeMemory(size_t size, void *p) {
  if (!p) {
    return;
  }

  if (size > 512) {
    operator delete(p);
    return;
  }

  getMemory(((size + 7) >> 3) - 1).release(reinterpret_cast<Slot *>(p));
}
} // namespace QueMemory
