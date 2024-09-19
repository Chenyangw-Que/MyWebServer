#include "EventThread.h"
#include <assert.h>
#include <functional>
#include <iostream>
#include <memory>
#include <unistd.h>
EventThread::EventThread()
    : loop_(NULL), exiting_(false), mutex_(), cond_(mutex_),
      thread_(std::bind(&EventThread::Run, this), "EventThread") {}

EventThread::~EventThread() {
  exiting_ = true;
  if (loop_) {
    loop_->StopLoop();
    thread_.join();
  }
}

EventLoop *EventThread::startLoop() {
  assert(!thread_.isStart());
  thread_.start();
  {
    MutexLockGuard lock(mutex_);
    while (loop_ == NULL) {
      cond_.wait();
    }
  }
  return loop_;
}

void EventThread::Run() {
  EventLoop loop;
  {
    MutexLockGuard lock(mutex_);
    loop_ = &loop;
    cond_.notify();
  }

  loop.Loop();
  loop_ = NULL;
}