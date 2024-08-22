#include "AsyncLogging.h"
#include "logUtil.h"
#include <assert.h>
AsyncLogging::AsyncLogging(const std::string filename, int timeout)
    : filename_(filename_), timeout_(timeout), mutex_(), cond_(mutex_),
      latch_(1),
      thread_(std::bind(&AsyncLogging::asyncWriteLog, this), "logging"),
      curBuffer_(new Buffer), nextBuffer_(new Buffer), buffers_(),
      is_running_(false) {
  assert(filename_.size() > 1);
  curBuffer_->bzero();
  nextBuffer_->bzero();
  buffers_.reserve(16);
}

AsyncLogging::~AsyncLogging() {
  if (is_running_) {
    stop();
  }
}

void AsyncLogging::writeLog(const char *singleLog, int size, bool is_quit) {
  // 写的时候要加锁
  MutexLockGuard lock(mutex_);
  {
    if (curBuffer_->preserveSize() > size) {
      curBuffer_->writebuff(singleLog, size);
      // 日志分级
    }

    else {
      // 若写满了将当前buffer存进列表，然后换nextbuffer
      buffers_.emplace_back(curBuffer_);
      curBuffer_.reset();
      if (nextBuffer_) {
        curBuffer_ = std::move(nextBuffer_);
      } else {
        curBuffer_.reset(new Buffer);
      }
      curBuffer_->writebuff(singleLog, size);
      // 日志分级
      cond_.notify();
    }
  }
}

void AsyncLogging::start() {
  is_running_ = true;
  thread_.start();
  latch_.wait(); // 线程结束也别着急退出
}

void AsyncLogging::stop() {
  is_running_ = false;
  cond_.notify();
  thread_.join();
}

// 开线程异步写入
void AsyncLogging::asyncWriteLog() {
  assert(is_running_);
  //
  latch_.countDown();
  // 应该不用加锁也可以啊，这里先用无锁写
  logUtils logHolder(filename_);
  std::vector<std::shared_ptr<Buffer>> localBuffers;
  while (is_running_) {
    {
      // 从后端缓冲区拿数据时加个锁
      MutexLockGuard lock(mutex_);
      // 如果后端代谢缓冲区是空的就等一等
      while (buffers_.empty()) {
        // 等待期间锁是放掉的
        cond_.waitSeconds(timeout_);
      }
      buffers_.emplace_back(curBuffer_); // 顺道把刚写入写缓冲区的也带上
      curBuffer_.reset();
      curBuffer_.reset(new Buffer);
      // 将待写入缓冲区的内容拿到线程本地，然后就可以释放锁了
      localBuffers.swap(buffers_);
      if (!nextBuffer_) {
        nextBuffer_.reset(new Buffer);
      }
      if (localBuffers.size() > 25) {
        // 丢掉一些内容
        localBuffers.erase(localBuffers.begin() + 2, localBuffers.end());
      }
      for(int i = 0 ; i < localBuffers.size(); i++){
        logHolder.writeFileBase(localBuffers[i]->getBuffer(), localBuffers[i]->buffSize());
      }
      localBuffers.clear();
      logHolder.flush();
    }
    logHolder.flush();
  }
}