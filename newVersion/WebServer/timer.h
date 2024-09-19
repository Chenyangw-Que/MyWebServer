#pragma once
#include <memory>
// timerNode负责管理channel，(实际上是通过httpConnection，因为http拥有channel)
class BaseConnection;
class httpConnection;


//定时器类
class TimerNode {
public:
  // TimerNode(std::shared_ptr<httpConnection> http, int timeout);
  TimerNode(std::shared_ptr<BaseConnection> http, int timeout);
  //拷贝构造
  TimerNode(TimerNode &timer);
  ~TimerNode();

  //更新到期时间 = 当前时间 + 超时时间
  void Update(int timeout);
  //是否到期
  bool is_expired();
  //释放http
  void Release();

  //得到到期时间
  int expire_time() const { return expire_time_; }

  // http是否已经删除
  bool is_deleted() const { return is_deleted_; }

private:
  // std::shared_ptr<httpConnection> http_connection_;
  std::shared_ptr<BaseConnection> http_connection_;
  int expire_time_;
  bool is_deleted_;
};