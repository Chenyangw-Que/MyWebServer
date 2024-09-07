#include "EventThreadPool.h"
#include "EventThread.h"
#include "base/Logging.h"
#include <assert.h>
EventThreadPool::EventThreadPool(EventLoop *mainLoop, int threadNum)
    : mainLoop_(mainLoop), isStart_(false), threadNum_(threadNum), next_(0) {
  if (threadNum_ <= 0) {
    LOG(FATAL) << "try to create eventThreadPool <=0";
    loopThreads_.reserve(threadNum_);
    loops_.reserve(threadNum_);
  }
}

EventThreadPool::~EventThreadPool() { LOG(DEBUG) << "~EventThreadPool()"; }

void EventThreadPool::start() {
  assert(mainLoop_->is_in_loop_thread()); // 只能由mainthread调用
  isStart_ = true;
  // 创建event_loop_thread_pool,并将开始Loop事件循环的EventLoop对象存入array中
  for (int i = 0; i < threadNum_; ++i) {
    auto eventThread = std::make_shared<EventThread>();
    loopThreads_.push_back(eventThread);
    loops_.push_back(eventThread->startLoop());
  }
}

EventLoop *EventThreadPool::getNextLoop() {
  // 调用这个函数的必须是主loop线程
  assert(mainLoop_->is_in_loop_thread());
  assert(isStart_);

  // 如果此时还没有开始Loop的EventLoop对象 就返回主loop
  auto event_loop = mainLoop_;
  if (!loops_.empty()) {
    event_loop = loops_[next_];
    next_ = (next_ + 1) % threadNum_;
  }
  return event_loop;
}