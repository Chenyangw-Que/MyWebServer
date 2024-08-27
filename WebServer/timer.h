#pragma once
#include <memory>
// timerNode负责管理channel，(实际上是通过httpConnection，因为http拥有channel)

// 做一个前置声明
class httpConnection;

class TimerNode {
public:
  TimerNode(std::shared_ptr<httpConnection> http, int timeout);
  TimerNode(const TimerNode &timer);
  ~TimerNode();
  void update(int timeout); // 更新超时时间
  bool isExpired();
  void release();
  int expireTime();
  bool isDeleted();

private:
    // 所管理的HttpConnection
    std::shared_ptr<httpConnection> httpConn_;
    int expireTime_;
    bool deleted_;
};