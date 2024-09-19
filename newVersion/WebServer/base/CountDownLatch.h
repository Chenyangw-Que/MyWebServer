#pragma once

// 封装countdownlatch
#include"noncopyable.h"
#include"mutexLock.h"

class CountDownLatch:noncopyable{
public:
    explicit CountDownLatch(int count);

    ~CountDownLatch();

    void wait();

    void countDown();

private:
    mutable MutexLock mutex_; // MutexLockGuard只是控制锁用于协调某一资源，CountDownLatch是造了一把锁用于管理对象间关系
    int count_;
    Condition cond_;
};