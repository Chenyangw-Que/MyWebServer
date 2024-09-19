#pragma once
#include "Poller.h"
#include "base/Thread.h"
#include "base/mutexLock.h"
#include "base/socketUtils.h"
class Channel;
class EventLoop {
public:
  typedef std::function<void()> Function;

  //初始化poller, event_fd，给event_fd注册到epoll中并注册其事件处理回调
  EventLoop();
  ~EventLoop();

  //开始事件循环
  //调用该函数的线程必须是该EventLoop所在线程，也就是Loop函数不能跨线程调用
  void Loop();

  //停止Loop
  void StopLoop();

  //如果当前线程就是创建此EventLoop的线程 就调用callback(关闭连接 EpollDel)
  //否则就放入等待执行函数区
  void RunInLoop(Function &&func);
  //把此函数放入等待执行函数区 如果当前是跨线程 或者正在调用等待的函数则唤醒
  void QueueInLoop(Function &&func);

  //把fd和绑定的事件注册到epoll内核事件表
  void PollerAdd(std::shared_ptr<Channel> channel, int timeout = 0) {
    poller_->EpollAdd(channel, timeout);
  }

  void PollerMod(std::shared_ptr<Channel> channel, int timeout = 0) {
    poller_->EpollMod(channel, timeout);
  }

  void PollerDel(std::shared_ptr<Channel> channel) {
    poller_->EpollDel(channel);
  }

  void ShutDown(std::shared_ptr<Channel> channel);

  bool is_in_loop_thread() const { return threadId_ == threadData::getpid(); }

private:
  //创建eventfd 类似管道的 进程间通信方式
  static int CreateEventfd();
  void
  HandleRead(); // eventfd的读回调函数(因为event_fd写了数据，所以触发可读事件，从event_fd读数据)
  void HandleUpdate(); // eventfd的更新事件回调函数(更新监听事件)
  void WakeUp(); //异步唤醒SubLoop的epoll_wait(向event_fd中写入数据)
  void
  PerformPendingFunctions(); //执行正在等待的函数(SubLoop注册EpollAdd连接套接字以及绑定事件的函数)

private:
  std::shared_ptr<Poller> poller_; // io多路复用 分发器
  int eventFd_; //用于异步唤醒SubLoop的Loop函数中的Poll(epoll_wait因为还没有注册fd会一直阻塞)
  std::shared_ptr<Channel> wakeupChannel_; //用于异步唤醒的channel
  pid_t threadId_;                         //线程id

  mutable MutexLock mutex_;
  std::vector<Function> pendingFunctions_; //正在等待处理的函数

  bool isStop_;                      //是否停止事件循环
  bool isLooping_;                   //是否正在事件循环
  bool isEventHandling_;            //是否正在处理事件
  bool isCallingPendingFunctions_; //是否正在调用等待处理的函数
};