#include "mainReactor.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>

#include "base/Logging.h"
#include "base/socketUtils.h"
#include "httpConnection.h"
#include <functional>
#include <memory>

mainReactor::mainReactor(EventLoop *event_loop, int thread_num, int port)
    : eventLoop_(event_loop), threadNum_(thread_num), port_(port),
      isStarted_(false) {
  // new一个事件循环线程池 和用于接收的Channel
  eventLoopThreadPool_ = std::unique_ptr<EventThreadPool>(
      new EventThreadPool(eventLoop_, thread_num));
  acceptChannel_ = std::make_shared<Channel>();

  // 绑定服务器ip和端口 监听端口
  listenFd_ = SocketListen(port_);
  acceptChannel_->set_fd(listenFd_);
  HandlePipeSignal();

  // 设置NIO非阻塞套接字
  if (SetSocketNonBlocking(listenFd_) < 0) {
    perror("set socket non block failed");
    abort();
  }
}

// 初始化
void mainReactor::Initialize(EventLoop *event_loop, int thread_num, int port) {
  eventLoop_ = event_loop;
  threadNum_ = thread_num;
  port_ = port;
  isStarted_ = false;

  // new一个事件循环线程池 和用于接收的Channel
  eventLoopThreadPool_ = std::unique_ptr<EventThreadPool>(
      new EventThreadPool(eventLoop_, thread_num));
  acceptChannel_ = std::make_shared<Channel>();

  // 绑定服务器ip和端口 监听端口
  listenFd_ = SocketListen(port_);
  acceptChannel_->set_fd(listenFd_);
  HandlePipeSignal();

  // 设置NIO非阻塞套接字
  if (SetSocketNonBlocking(listenFd_) < 0) {
    perror("set socket non block failed");
    abort();
  }
}

// 单例模式（懒汉式）
mainReactor &mainReactor::GetInstance() {
  static mainReactor webServer_;
  return webServer_;
}

//开始
void mainReactor::Start() {
  //开启event_loop线程池
  eventLoopThreadPool_->start();
  // accept的管道
  acceptChannel_->set_events(EPOLLIN | EPOLLET);
  acceptChannel_->set_read_handler(
      std::bind(&mainReactor::HandleNewConnect, this));
  acceptChannel_->set_update_handler(
      std::bind(&mainReactor::HandelCurConnect, this));

  eventLoop_->PollerAdd(acceptChannel_, 0);
  isStarted_ = true;
}

// 分发channel 给subLoop
void mainReactor::HandleNewConnect() {
  struct sockaddr_in client_addr;
  memset(&client_addr, 0, sizeof(struct sockaddr_in));
  socklen_t client_addr_len = sizeof(client_addr);

  while (true) {
    int connect_fd =
        accept(listenFd_, (struct sockaddr *)&client_addr, &client_addr_len);
    EventLoop *event_loop = eventLoopThreadPool_->getNextLoop();
    LOG(INFO) << "New connection from " << inet_ntoa(client_addr.sin_addr)
              << ":" << ntohs(client_addr.sin_port);

    // todo
    if (connect_fd < 0) {
      LOG(WARNING) << "Accept failed!";
      break;
    }
    // 限制服务器的最大并发连接数
    if (connect_fd >= MAX_FD_NUM) {
      LOG(WARNING) << "Internal server busy!";
      close(connect_fd);
      break;
    }
    // 设为非阻塞模式
    if (SetSocketNonBlocking(connect_fd) < 0) {
      LOG(WARNING) << "set socket nonblock failed!";
      close(connect_fd);
      break;
    }
    //设置套接字
    SetSocketNoDelay(connect_fd);
    // setSocketNoLinger(connect_fd);

    std::shared_ptr<httpConnection> reqInfo(
        new httpConnection(event_loop, connect_fd));
    // channel 申请由httpConnect来做， 添加epoll也是
    reqInfo->connect_channel()->set_holder(reqInfo);

    event_loop->QueueInLoop(std::bind(&httpConnection::Register, reqInfo));
  }

  acceptChannel_->set_events(EPOLLIN | EPOLLET);
}

void mainReactor::HandelCurConnect() { eventLoop_->PollerMod(acceptChannel_); }
