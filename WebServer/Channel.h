// 负责如下事务:
// 绑定fd，记录fd的感兴趣事件即实际触发事件
// 创建fd针对不同事件的回调
#pragma once
#include "base/noncopyable.h"
#include "httpConnection.h"
#include <functional>
#include <memory>
class Channel {
public:
  typedef std::function<void()> CallBack;
  Channel();
  explicit Channel(int fd);
  ~Channel();

  void handleEvents();
  void handleRead();
  void handleWrite();
  void handleUpdate();
  void handleError();

  int fd() const;
  void setfd(int fd);
  std::shared_ptr<httpConnection> getHolder_();

  void setHolder(std::shared_ptr<httpConnection> holder);
  void setReadHandler(CallBack &&);
  void setWriteHandler(CallBack &&);
  void setUpdateHandler(CallBack &&);
  void setErrorHandler(CallBack &&);

  void setREvents(int);
  void setEvents(int);
  int &getEvents();
  int getLastEvents() const;
  bool updateLastEvents();

private:
  int events_;
  int revents_;
  int lastevents_;
  CallBack readHandler_;
  CallBack writeHandler_;
  CallBack updateHander_;
  CallBack errorHander_;
  std::weak_ptr<httpConnection> holder_;
  int fd_;
};