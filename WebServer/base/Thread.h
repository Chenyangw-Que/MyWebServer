#pragma once
// 基本线程类
#include <functional>
#include <memory>
#include <pthread.h>
#include <sys/syscall.h>
#include <unistd.h>
class thread {
  // 定义线程类无非是要封装基本的线程创建、join等操作
  // 想想需要什么首先线程创建肯定基于pthread_create，需要变量接受threadid，需要线程启动函数，需要传入线程参数

public:
  typedef std::function<void()> threadFunc; // 为线程函数起个别名
  explicit thread(const threadFunc &, const std::string &name = "");
  ~thread();
  void start();
  int join();
  bool isStart() { return started_; }
  pid_t gettid() { return tid_; }
  std::string getThreadName() { return threadName_; }

private:
  // 考虑到线程中执行的程序有打印日志的需要，在内存中存了一份表去缓存线程信息
  // 设置线程启动函数来填充信息表，并调用传入的线程任务函数
  static void *run(void *args);
  void cacheData();

private:
  pthread_t threadId_; // 线程创建时写入线程id
  pid_t tid_; // 由于线程再linux本质上是一个轻量级进程，考虑到可能的线程间通信，
              // 存一下线程的实际id(进程id)
  threadFunc func_;        //定义线程启动函数，之后具体再绑定
  std::string threadName_; //考虑到日志，给一个线程名称
  bool started_;           //  启动标识
  bool joined_;            // joined标识
};