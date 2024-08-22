#pragma once
// 之前在logUtil提到过，文件类就是对缓冲区的应用：
// 任务线程写日志到前端缓冲区->
// 前端缓冲区于log类的后端缓冲区组成双缓冲区结构，将前端缓冲区置换到后端缓冲区
// 后端缓冲区维护一个待写入缓冲区队列，进行写操作。
// 其中对于一次写操作来说，程序先将日志写到写缓冲区，在某一时机将写缓冲内容统一写入文件

// 本文间封装 任务协程所用的，将日志写入前端缓冲区的接口

// 上边也讲了关键是缓冲区操作，所以首先定义一个缓冲区
#include "noncopyable.h"
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <string>

#define COLOR_END "\e[0m"
#define BLACK "\e[0;30m"
#define BRIGHT_BLACK "\e[1;30m"
#define RED "\e[0;31m"
#define BRIGHT_RED "\e[1;31m"
#define GREEN "\e[0;32m"
#define BRIGHT_GREEN "\e[1;32m"
#define BROWN "\e[0;33m"
#define YELLOW "\e[1;33m"
#define BLUE "\e[0;34m"
#define BRIGHT_BLUE "\e[1;34m"
#define PURPLE "\e[0;35m"
#define BRIGHT_PURPLE "\e[1;35m"
#define CYAN "\e[0;36m"
#define BRIGHT_CYAN "\e[1;36m"
#define GRAY "\e[0;37m"
#define WHITE "\e[1;37m"

#define BOLD "\e[1m"
#define UNDERLINE "\e[4m"
#define BLINK "\e[5m"
#define REVERSE "\e[7m"
#define HIDE "\e[8m"
#define CLEAR "\e[2J"
#define CLRLINE "\r\e[K"

constexpr int kSmallBuff = 4000;        // 4k大小
constexpr int kLargeBuff = 4000 * 1000; // 4m大小

template <int buffer_size> class logBuffer : noncopyable {
public:
  logBuffer() : size_(0) {
    bzero();
    curBuffPos = 0;
  }

  ~logBuffer() {}

  void bzero() {
    char buffer_[buffer_size];
    memset(buffer_, 0, sizeof(buffer_));
  }
  void moveForward(size_t len) {
    curBuffPos += len;
    size_ += len;
  }
  void writebuff(const char *log, int size) {
    if (preserveSize() > size) {
      memcpy(curBuffPos, log, size);
      moveForward(size);
    }
  }

  const char *getBuffer() { return buffer_; }

  char *getCurPos() { return curBuffPos; }

  int buffSize() { return size_; }

  int preserveSize() { return buffer_size - size_; }

  void reset() {
    curBuffPos = buffer_;
    size_ = 0;
  }

private:
  // 要不停往buffer里写东西，要记录下一次可写的位置
  char buffer_[buffer_size];
  char *curBuffPos;
  int size_;
};

// buffer定义完成，重新定义buff流操作，即相当于重定义对于工作线程的buff写入接口
// 实际上就说穷举各种可能的输入，将其转化为向buff字符数组的写入
class logStream : noncopyable {
public:
  using Buffer = logBuffer<kSmallBuff>; // 工作线程向前端缓冲区写入用小buffer
  logStream() {}
  ~logStream() {}
  const Buffer &buffer() const { return buffer_; }
  void reset_buffer() { buffer_.reset(); }
  logStream &operator<<(char log);
  logStream &operator<<(const char *log);
  logStream &operator<<(unsigned const char *log);
  logStream &operator<<(std::string &log);
  logStream &operator<<(bool log);
  logStream &operator<<(int log);
  logStream &operator<<(unsigned log);
  logStream &operator<<(double log);
  logStream &operator<<(long double log);
  logStream &operator<<(float log);
  logStream &operator<<(long log);
  logStream &operator<<(unsigned long log);
  logStream &operator<<(long long log);
  logStream &operator<<(unsigned long long log);
  logStream &operator<<(short log);
  logStream &operator<<(unsigned short log);

private:
  template <typename T> void IntegerToBuffer(T input);
  static constexpr int kMaxNumberSize = 32;
  Buffer buffer_;
};