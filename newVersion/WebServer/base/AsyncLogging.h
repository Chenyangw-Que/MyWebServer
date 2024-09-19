#pragma once
// 如先前所说，本类负责构建异步写入流程，包括接受工作线程写入的日志
// 组建前后端双缓冲区，构建异步写入队列，负责将写入队列的内容写入硬盘
#include "CountDownLatch.h"
#include "Thread.h"
#include "logStream.h"
#include "mutexLock.h"
#include "noncopyable.h"
#include <vector>
class AsyncLogging : noncopyable {
  // 异步写入端用大buffer
public:
  using Buffer = logBuffer<kLargeBuff>;
  // 异步写入本质上还是去调一个线程去写

  AsyncLogging(const std::string filename, int timeout = 2);
  ~AsyncLogging();
  void writeLog(const char *singleLog, int size, bool is_quit = false);
  void start();
  void stop();

private:
  // 首先要维护一个日志文件名
  std::string filename_;
  const int timeout_;
  bool is_running_;

  std::shared_ptr<Buffer> curBuffer_;
  std::shared_ptr<Buffer> nextBuffer_;
  std::vector<std::shared_ptr<Buffer>> buffers_;

  thread thread_;
  mutable MutexLock mutex_;
  Condition cond_;
  CountDownLatch latch_;

  void asyncWriteLog(); // 定义线程任务函数
};