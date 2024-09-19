#pragma once
#include <sys/epoll.h>
#include <unistd.h>
#include <sys/uio.h>
#include "Channel.h"
#include "EventLoop.h"
#include "base/LFUCache.h"
#include "timer.h"
#include <functional>
#include <map>
#include <memory>
#include <string>

class Channel;
class EventLoop;

class BaseConnection : public std::enable_shared_from_this<BaseConnection>
{
public:
    BaseConnection(EventLoop *event_loop, int connect_fd) : event_loop_(event_loop), connect_fd_(connect_fd)
    {
    }
    virtual ~BaseConnection() { close(connect_fd_);}

    virtual void Register() = 0;     // 注册事件，留给派生类实现
    virtual void Delete() = 0;       // 删除事件，留给派生类实现
    virtual void Reset() = 0;        // 重置连接状态，留给派生类实现
    virtual void HandleRead() = 0;   // 处理读事件
    virtual void HandleWrite() = 0;  // 处理写事件
    virtual void HandleUpdate() = 0; // 处理更新事件

    void set_timer(std::shared_ptr<TimerNode> timer) { timer_ = timer; }

    std::shared_ptr<Channel> connect_channel() { return connect_channel_; }
    int connect_fd() const { return connect_fd_; }

protected:
    int connect_fd_;                           // 连接套接字fd
    EventLoop *event_loop_;                    // 事件循环
    std::shared_ptr<Channel> connect_channel_; // 连接套接字的管道
    std::weak_ptr<TimerNode> timer_;           // 定时器
    std::string read_buffer_;                  // 读缓冲区
    std::string write_buffer_;                 // 写缓冲区
};
