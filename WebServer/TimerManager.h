#pragma once
// timer管理器
#include "base/noncopyable.h"
#include "base/skipList.h"
#include "timer.h"
#include <deque>
#include <memory>
#include <queue>
class httpConnection;

struct cmpMethod {
  bool operator()(const std::shared_ptr<TimerNode> &a,
                  const std::shared_ptr<TimerNode> &b) const {
    return a->expireTime() >
           b->expireTime(); // 建堆过程中的上调操作，当目标节点的父节点大于目标节点，将目标节点上调，建成小根堆
  }
};

// 用小根堆管理定时器
class TimerHeap : noncopyable {
public:
  TimerHeap() {}
  ~TimerHeap() {}
  void pushTimer(std::shared_ptr<httpConnection> httpConn, int timeout);
  void handleExpireEvent(); // 处理超时事件

private:
  // 首先肯定有一个小根堆
  std::priority_queue<std::shared_ptr<httpConnection>,
                      std::deque<std::shared_ptr<TimerNode>>, cmpMethod>
      timerheap_;
};


// skiplist todo