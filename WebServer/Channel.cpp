#include "Channel.h"
#include "sys/epoll.h"
Channel::Channel() : fd_(0), events_(0), lastevents_(0) {}

Channel::Channel(int fd) : fd_(fd), events_(0), lastevents_(0) {}

Channel::~Channel() {}

void Channel::handleEvents() {
  events_ = 0;
  if ((revents_ & EPOLLHUP) && !(revents_ & EPOLLIN)) {
    // 挂起且无可读事件
    events_ = 0;
    return;
  }
  if (revents_ & EPOLLERR) {
    handleError();
    events_ = 0;
    return;
  }

  if (revents_ & (EPOLLIN | EPOLLPRI | EPOLLRDHUP)) {
    handleRead();
  }
  if (revents_ & EPOLLOUT) {
    handleWrite();
  }
  handleUpdate();
}

void Channel::handleRead() {
  if (readHandler_)
    readHandler_();
}

void Channel::handleWrite() {
  if (writeHandler_)
    writeHandler_();
}

void Channel::handleError() {
  if (errorHander_)
    errorHander_();
}

void Channel::handleUpdate() {
  if (updateHander_)
    updateHander_();
}

void Channel::setReadHandler(CallBack &&cb) { readHandler_ = cb; }
void Channel::setWriteHandler(CallBack &&cb) { writeHandler_ = cb; }
void Channel::setUpdateHandler(CallBack &&cb) { updateHander_ = cb; }
void Channel::setErrorHandler(CallBack &&cb) { errorHander_ = cb; }

int Channel::fd() const { return fd_; }

void Channel::setfd(int fd) { fd_ = fd; }

void Channel::setEvents(int event) { events_ = event; }
void Channel::setREvents(int event) { revents_ = event; }
bool Channel::updateLastEvents() {
  bool isChange = (lastevents_ == events_);
  lastevents_ = events_;
  return isChange;
}

int &Channel::getEvents() { return events_; }

int Channel::getLastEvents() const { return lastevents_; }

std::shared_ptr<httpConnection> Channel::getHolder_() {
  std::shared_ptr<httpConnection> http(holder_.lock());
  return http;
}

void Channel::setHolder(std::shared_ptr<httpConnection> holder) {
  holder_ = holder;
}