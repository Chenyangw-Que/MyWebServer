#include "EventLoop.h"
#include "EventThread.h"
#include "EventThreadPool.h"
#include "Channel.h"
#include <memory>
class mainReactor {
public:
  static mainReactor &GetInstance();
  void Initialize(EventLoop *event_loop, int thread_num, int port);

  EventLoop *event_loop() { return eventLoop_; }

  void Start();

private:
  mainReactor() {}
  mainReactor(EventLoop *event_loop, int thread_num, int port);
  ~mainReactor() {}

  mainReactor(const mainReactor &) = delete;
  mainReactor &operator=(const mainReactor &) = delete;

  void HandleNewConnect();
  void HandelCurConnect();

private:
  static const int MAX_FD_NUM = 100000;

  EventLoop *eventLoop_;
  std::unique_ptr<EventThreadPool> eventLoopThreadPool_;
  std::shared_ptr<Channel> acceptChannel_;

  int port_;
  int listenFd_;
  int threadNum_;
  bool isStarted_;
};