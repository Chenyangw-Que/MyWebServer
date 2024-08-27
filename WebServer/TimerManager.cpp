#include "TimerManager.h"
#include "httpConnection.h"
#include "timer.h"
void TimerHeap::pushTimer(std::shared_ptr<httpConnection> httpConn,
                          int timeout) {
  auto timer = std::make_shared<TimerNode>(httpConn, timeout);
  timerheap_.push(timer);
  httpConn->setTimer(timer);
}

void TimerHeap::handleExpireEvent() {
  while (!timerheap_.empty()) {
    // 超时的全干掉
    auto timer = timerheap_.top();
    if (timer->isDeleted() || timer->isExpired()) {
      timerheap_.pop();
    } else
      break;
  }
}