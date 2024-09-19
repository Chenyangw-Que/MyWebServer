#include "TimerManager.h"
#include <memory>
#include "httpConnection.h"
#include "Connection.h"
// //向小根堆中添加定时器
// void TimerHeap::AddTimer(std::shared_ptr<httpConnection> http_connection,
//                          int timeout) {
//   // new一个定时器 加上到小根堆中
//   auto timer = std::make_shared<TimerNode>(http_connection, timeout);
//   timer_heap_.push(timer);
//   http_connection->set_timer(timer);
// }

//向小根堆中添加定时器
void TimerHeap::AddTimer(std::shared_ptr<BaseConnection> http_connection,
                         int timeout) {
  // new一个定时器 加上到小根堆中
  auto timer = std::make_shared<TimerNode>(http_connection, timeout);
  timer_heap_.push(timer);
  http_connection->set_timer(timer);
}



// 处理到期事件 如果定时器被删除或者已经到期 就从小根堆中删除
// 定时器析构的时候 会调用http->close 会关闭http连接，EpollDel绑定的connect_fd
void TimerHeap::HandleExpireEvent() {
  while (!timer_heap_.empty()) {
    auto timer = timer_heap_.top();
    // 如果是被删除了(惰性删除 已经到期了但是还没被访问所以没被删)
    // 或者到期了都会pop定时器
    if (timer->is_deleted() || timer->is_expired()) {
      timer_heap_.pop();
    } else {
      break;
    }
  }
}
