#pragma once
// 主要实现定长内存池

#include "mutexLock.h"
#include <utility>
#include<iostream>
#define BlockSize 16384
struct Slot {
  Slot *next;
};

// 先定义单一定长内存池，再通过单例模式封装多种长度定长内存池
class MemoryPool {
public:
  MemoryPool();
  ~MemoryPool();
  void init(int size);
  Slot *allocate();
  void release(Slot *p);

  // 首先考虑一个问题，内存池本质是一块被分成等长块的，连续内存,
  // 但这里为每个slot增加了指针，这样一方面便于管理，另一方面为后续从堆申请离散内存创造了可能
  // 但是可用内存并不一定是连续的，一些被申请后又被释放调度内存块可能是不连续的，这时需要一个链表进行管理
private:
  int slotSize_;
  Slot *freeSlot_; // 链表头
  Slot *curBlockSlot_; // 连续内存块中，下一个未使用内存块的指针
  // 同时要记录该内存块的首位
  Slot *BlockHead_;
  Slot *BlockLast_;

  // 锁
  MutexLock mutexfreeSlot_;
  MutexLock mutexOther_;

  size_t pad(char *p, size_t align);
  Slot *allocateBlock(); // 申请内存块
  Slot *nonfree();
};
namespace QueMemory{
// 单例模式封装
MemoryPool &getMemory(int id);

void init();

void *useMemory(size_t size);

void freeMemory(size_t size, void *p);

template <typename T, typename... Args> T *newElement(Args &&... args) {
  T *p;
  if ((p = reinterpret_cast<T *>(useMemory(sizeof(T)))) != nullptr)
    // new(p) T1(value);
    // placement new:在指针p所指向的内存空间创建一个T1类型的对象，类似与 realloc
    // 把已有的空间当成一个缓冲区来使用，减少了分配空间所耗费的时间
    // 因为直接用new操作符分配内存的话，在堆中查找足够大的剩余空间速度是比较慢的
    new (p) T(std::forward<Args>(args)...); // 完美转发

  return p;
}

// 调用p的析构函数，然后将其总内存池中释放
template <typename T> void deleteElement(T *p) {
  // printf("deleteElement...\n");
  if (p)
    p->~T();
  freeMemory(sizeof(T), reinterpret_cast<void *>(p));
  // printf("deleteElement success\n");
}
}
