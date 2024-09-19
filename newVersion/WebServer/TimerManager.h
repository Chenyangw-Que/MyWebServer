#ifndef TIMER_TIMER_HEAP_H_
#define TIMER_TIMER_HEAP_H_

#include "timer.h"

#include <deque>
#include <memory>
#include <queue>

#include "base/noncopyable.h"
#include "Connection.h"
//类的前置声明
class BaseConnection;

//定时器比较仿函数 升序
struct TimerCompare {
  bool operator()(const std::shared_ptr<TimerNode> &a,
                  const std::shared_ptr<TimerNode> &b) const {
    return a->expire_time() > b->expire_time();
  }
};

//定时器小根堆
class TimerHeap : noncopyable {
public:
  TimerHeap() {}
  ~TimerHeap() {}

  //添加定时器 将其添加到小根堆中
  //void AddTimer(std::shared_ptr<httpConnection> http_connection, int timeout);
  void AddTimer(std::shared_ptr<BaseConnection> http_connection, int timeout);

  //处理到期事件 如果定时器被删除或者已经到期 就从小根堆中删除
  void HandleExpireEvent();

private:
  //优先级队列 小顶堆
  std::priority_queue<std::shared_ptr<TimerNode>,
                      std::deque<std::shared_ptr<TimerNode>>, TimerCompare>
      timer_heap_;
};

#endif // namespace TIMER_TIMER_H_