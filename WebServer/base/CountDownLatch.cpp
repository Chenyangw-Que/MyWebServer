#include "CountDownLatch.h"
#include "mutexLock.h"
CountDownLatch::CountDownLatch(int count)
    : mutex_(), cond_(mutex_), count_(count) {}

CountDownLatch::~CountDownLatch() {}

void CountDownLatch::wait() {
  MutexLockGuard lock(mutex_);
  while (count_) {
    cond_.wait(); // wait这段期间把锁释放掉了
  }
}


void CountDownLatch::countDown(){
    // 减count时加锁
    MutexLockGuard lock(mutex_);
    if(!--count_) cond_.notifyBoardCast();
}