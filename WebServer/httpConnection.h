#pragma once
#include "Channel.h"
#include "base/noncopyable.h"
#include "timer.h"
#include <map>
#include <memory>
#include <sys/epoll.h>
#include "base/httpDefine.h"
// 解析、处理http请求
// 对于这个类的定位：针对http请求的解析方法，对http请求的实际拥有者，对connfd的实际拥有者对channel的委托人
class EventLoop;

class httpConnection : noncopyable {
public:
  httpConnection(EventLoop *loop, int connfd);
  ~httpConnection();
  void Register(); // 注册fd的默认事件
  void reset();
  void resetTimer();
  void setTimer(std::shared_ptr<TimerNode> timer);
  int connectfd();
  void Delete();
  //std::shared_ptr<Channel> connectChannel();

private:
  static constexpr int kDefaultEvent = EPOLLIN | EPOLLET | EPOLLONESHOT;
  static constexpr int kDefaultTimeOut = 2 * 1000;
  static constexpr int kDefaultKeepAliveTime = 5 * 60 * 1000;

  int connectfd_;
  EventLoop *loop_;
  //std::shared_ptr<Channel> connectChannel_;
  std::weak_ptr<TimerNode> timer_;
  std::string readBuffer_;
  std::string writeBuffer_;

  httpDefine::ProcessState processState_;
  httpDefine::ConnectionState connectionState_;
  httpDefine::ParseState parseState_;
  httpDefine::RequestMethod method_;
  httpDefine::HttpVersion version_;

  std::string filename_;   // 请求文件名
  std::string resourceDir; //资源路径
  std::map<std::string, std::string> requestHeaders_;
  bool isError_;
  bool isKeepAlive_;

private:
  httpDefine::LineState ParseLine();
  httpDefine::HeaderState ParseHeader();
  httpDefine::ResponseState BuildResponse();
  // 定义传给channel的回调
  void handleRead();
  void handleWrite();
  void handleUpdate();
  void ReturnErrorMsg(int fd, int errorcode, std::string errorMsg);
};