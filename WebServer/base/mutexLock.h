#pragma once
#include "noncopyable.h"
#include <pthread.h>
// 定义锁

//首先定义基本的线程加锁操作

class MutexLock : noncopyable {
public:
  MutexLock();
  ~MutexLock();
  void lock();
  void unlock();
  pthread_mutex_t *get() { return &mutex_; }

private:
  //互斥信号量
  pthread_mutex_t mutex_;
  friend class Condition;
};

// RAII分割封装互斥锁

class MutexLockGuard : noncopyable {
public:
  explicit MutexLockGuard(MutexLock &mutex);
  ~MutexLockGuard();

private:
  MutexLock &
      mutex_; // 存一个信号量引用，引用却不拥有，故mutexLockGuard析构时mutex_不析构，MutexLockGuard只是借用了mutex的使用权
};

// 封装条件按变量，条件等待、定时器、通知
class Condition : noncopyable {
public:
  explicit Condition(MutexLock &mutex);
  ~Condition();
  void wait();
  bool waitSeconds(int seconds);
  void notify();
  void notifyBoardCast();

private:
  MutexLock &mutex_;
  pthread_cond_t cond_;
};