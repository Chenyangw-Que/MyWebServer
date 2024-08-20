#include "mutexLock.h"
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
MutexLock::MutexLock() {
  // 初始化
  pthread_mutex_init(&mutex_, NULL);
}

MutexLock::~MutexLock() {
  // 先拿到控制权在销毁
  pthread_mutex_lock(&mutex_);
  pthread_mutex_destroy(&mutex_);
}

void MutexLock::lock() { pthread_mutex_lock(&mutex_); }

void MutexLock::unlock() { pthread_mutex_unlock(&mutex_); }

pthread_mutex_t *MutexLock::get() { return &mutex_; }

MutexLockGuard::MutexLockGuard(MutexLock &mutex) : mutex_(mutex) {
  mutex_.lock();
}

MutexLockGuard::~MutexLockGuard() { mutex_.unlock(); };

Condition::Condition(MutexLock &mutex) : mutex_(mutex) {
  pthread_cond_init(&cond_, NULL);
}

Condition::~Condition() { pthread_cond_destroy(&cond_); }

void Condition::wait() { pthread_cond_wait(&cond_, mutex_.get()); }

bool Condition::waitSeconds(int second) {
  struct timespec time;
  clock_gettime(CLOCK_REALTIME, &time);
  time.tv_sec += static_cast<time_t>(second);
  return ETIMEDOUT ==
         pthread_cond_timedwait(&cond_, mutex_.get(), &time); // 超时返回
}

void Condition::notify() { pthread_cond_signal(&cond_); }

void Condition::notifyBoardCast() { pthread_cond_broadcast(&cond_); }