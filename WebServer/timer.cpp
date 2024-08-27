#include "timer.h"
#include "base/Logging.h"
#include "httpConnection.h"
#include <memory>
#include <sys/time.h>
TimerNode::TimerNode(std::shared_ptr<httpConnection> http, int timeout)
    : expireTime_(0), httpConn_(http), deleted_(false) {
  // 计算出一个初始的超时时间，也就是当前时间+超时时间
  update(timeout);
}

TimerNode::TimerNode(const TimerNode &timer)
    : httpConn_(timer.httpConn_), expireTime_(0) {}

TimerNode::~TimerNode() {
  if (httpConn_) {
    // 如果不是主动释放
    LOG(INFO) << "Time out, close fd: " << httpConn_->connectfd();
    httpConn_->Delete();
  }
}

void TimerNode::update(int timeout) {
  struct timeval now;
  gettimeofday(&now, NULL);
  expireTime_ =
      (((now.tv_sec % 10000) * 1000) + (now.tv_usec / 1000)) + timeout;
}

bool TimerNode::isExpired() {
  struct timeval now;
  gettimeofday(&now, NULL);
  size_t cur = (((now.tv_sec % 10000) * 1000) + (now.tv_usec / 1000));
  if (cur >= expireTime_) {
    deleted_ = true; // 用于惰性删除
    return true;     // 标记为删除，却不释放资源
  }
  return false;
}

void TimerNode::release() {
  httpConn_.reset();
  deleted_ = true;
}

int TimerNode::expireTime() { return expireTime_; }

bool TimerNode::isDeleted() { return deleted_; }