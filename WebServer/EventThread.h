#pragma once

#include "EventLoop.h"
#include "base/Thread.h"
#include "base/mutexLock.h"
#include "base/noncopyable.h"
class EventThread : noncopyable {
public:
  EventThread();
  ~EventThread();

  EventLoop *startLoop(); // 创建线程，启动线程函数

private:
  void Run(); // 线程函数
  thread thread_;
  EventLoop *loop_;
  mutable MutexLock mutex_;
  Condition cond_;
  bool exiting_;
};