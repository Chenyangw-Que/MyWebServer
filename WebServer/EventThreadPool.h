#pragma once
#include "EventLoop.h"
#include "EventThread.h"
#include "base/noncopyable.h"
#include <memory>
#include <vector>
class EventThreadPool : noncopyable {
public:
  EventThreadPool(EventLoop *mainLoop, int threadNum);
  ~EventThreadPool();
  void start();
  EventLoop *getNextLoop();

private:
  EventLoop *mainLoop_;
  bool isStart_;
  int threadNum_;
  int next_;
  std::vector<std::shared_ptr<EventThread>> loopThreads_;
  std::vector<EventLoop *> loops_;
};