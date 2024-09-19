#pragma once
// log这块关键是对缓冲区的操作，从线程缓冲区到
// log处理缓冲区，再由log处理缓冲区写入文件 首先封装文件操作
#include "mutexLock.h"
#include "noncopyable.h"
#include <memory>
#include <string>
// 单次 log处理缓冲区-》硬盘写入工具

// 未加锁的写入
class logUtils {
public:
  explicit logUtils(std::string filename);
  ~logUtils();
  void writeFileBase(const char *singleLog, const size_t size);
  void flush();

private:
  FILE *fp_;
  char buffer_[64 * 1024]; // 64k
};

// 加速写入，为什么要加锁，后端的写入线程貌似就一个，加锁操作能否再往上封装一层

// 日志文件 封装了FileUtils
class LogFile : noncopyable {
public:
  // 每写flush_every_n次，就会flush一次
  LogFile(const std::string &file_name, int flush_every_n = 1024)
      : file_name_(file_name), flush_every_n_(flush_every_n), count_(0),
        mutex_() {
    file_.reset(new logUtils(file_name));
  }

  ~LogFile() {}

  // 写日志到文件
  void Write(const char *single_log, int size) {
    MutexLockGuard lock(mutex_);
    {
      // 每写flush_every_n次，就会flush一次
      file_->writeFileBase(single_log, size);
      ++count_;
      if (count_ >= flush_every_n_) {
        count_ = 0;
        file_->flush();
      }
    }
  }

  void Flush() {
    MutexLockGuard lock(mutex_);
    { file_->flush(); }
  }

private:
  const std::string file_name_;
  const int flush_every_n_;
  int count_;
  MutexLock mutex_;
  std::unique_ptr<logUtils> file_;
};