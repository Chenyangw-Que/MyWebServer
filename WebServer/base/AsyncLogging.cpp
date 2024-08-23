// #include "AsyncLogging.h"
// #include "logUtil.h"
// #include <assert.h>
// AsyncLogging::AsyncLogging(const std::string filename, int timeout)
//     : filename_(filename_), timeout_(timeout), mutex_(), cond_(mutex_),
//       latch_(1),
//       thread_(std::bind(&AsyncLogging::asyncWriteLog, this), "logging"),
//       curBuffer_(new Buffer), nextBuffer_(new Buffer), buffers_(),
//       is_running_(false) {
//   assert(filename_.size() > 1);
//   curBuffer_->bzero();
//   nextBuffer_->bzero();
//   buffers_.reserve(16);
// }

// AsyncLogging::~AsyncLogging() {
//   if (is_running_) {
//     stop();
//   }
// }

// void AsyncLogging::writeLog(const char *singleLog, int size, bool is_quit) {
//   // 写的时候要加锁
//   MutexLockGuard lock(mutex_);
//   {
//     if (curBuffer_->preserveSize() > size) {
//       curBuffer_->writebuff(singleLog, size);
//       // 日志分级
//     }

//     else {
//       // 若写满了将当前buffer存进列表，然后换nextbuffer
//       buffers_.emplace_back(curBuffer_);
//       curBuffer_.reset();
//       if (nextBuffer_) {
//         curBuffer_ = std::move(nextBuffer_);
//       } else {
//         curBuffer_.reset(new Buffer);
//       }
//       curBuffer_->writebuff(singleLog, size);
//       // 日志分级
//       cond_.notify();
//     }
//   }
// }

// void AsyncLogging::start() {
//   is_running_ = true;
//   thread_.start();
//   latch_.wait(); // 线程结束也别着急退出
// }

// void AsyncLogging::stop() {
//   is_running_ = false;
//   cond_.notify();
//   thread_.join();
// }

// // 开线程异步写入
// void AsyncLogging::asyncWriteLog() {
//   assert(is_running_);
//   //
//   latch_.countDown();
//   // 应该不用加锁也可以啊，这里先用无锁写
//   //logUtils logHolder(filename_);
//   LogFile logHolder(filename_);
  
//   std::vector<std::shared_ptr<Buffer>> localBuffers;
//   while (is_running_) {
//     {
//       // 从后端缓冲区拿数据时加个锁
//       MutexLockGuard lock(mutex_);
//       // 如果后端代谢缓冲区是空的就等一等
//       while (buffers_.empty()) {
//         // 等待期间锁是放掉的
//         cond_.waitSeconds(timeout_);
//       }
//       buffers_.emplace_back(curBuffer_); // 顺道把刚写入写缓冲区的也带上
//       curBuffer_.reset();
//       curBuffer_.reset(new Buffer);
//       // 将待写入缓冲区的内容拿到线程本地，然后就可以释放锁了
//       localBuffers.swap(buffers_);
//       if (!nextBuffer_) {
//         nextBuffer_.reset(new Buffer);
//       }
//       if (localBuffers.size() > 25) {
//         // 丢掉一些内容
//         localBuffers.erase(localBuffers.begin() + 2, localBuffers.end());
//       }
//       for(int i = 0 ; i < localBuffers.size(); i++){
//         logHolder.Write(localBuffers[i]->getBuffer(), localBuffers[i]->buffSize());
//       }
//       localBuffers.clear();
//       logHolder.Flush();
//     }
//     logHolder.Flush();
//   }
// }


#include "AsyncLogging.h"
#include "logUtil.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include <functional>


AsyncLogging::AsyncLogging(std::string file_name, int timeout)
    : filename_(file_name),
      timeout_(timeout),
      mutex_(),
      cond_(mutex_),
      latch_(1),
      thread_(std::bind(&AsyncLogging::asyncWriteLog, this), "Logging"),
      curBuffer_(new Buffer),
      nextBuffer_(new Buffer),
      buffers_(),
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

//将日志写入buffer输出缓冲区中
void AsyncLogging::writeLog(const char* single_log, int size, bool is_fatal) {
    MutexLockGuard lock(mutex_);
    {
        //buffer没写满 就一直写
        if (curBuffer_->preserveSize() > size) {
            curBuffer_->writebuff(single_log, size);
            if (is_fatal) {
                stop();
                abort();
            }
        } else {
            //这里buffer写满了 将buffer存入buffers
            buffers_.push_back(curBuffer_);
            curBuffer_.reset();
            //当前buffer满了 就new一个新buffer写 如果next buffer有值 就把所有权给当前buffer
            if (nextBuffer_) {
                curBuffer_ = std::move(nextBuffer_);
            } else {
                curBuffer_.reset(new Buffer);
            }
            curBuffer_->writebuff(single_log, size);
            //这里如果是fatal的日志，那么写完这条日志就停止线程 然后abort
            if (is_fatal) {
                stop();
                abort();
            }
            //唤醒线程拿buffer的数据 写入日志文件
            cond_.notify();
        }
    }
}

//开始线程
void AsyncLogging::start() {
    is_running_ = true;
    thread_.start();
    latch_.wait();
}

//停止线程
void AsyncLogging::stop() {
    is_running_ = false;
    cond_.notify();
    thread_.join();
}

//线程函数 程序里写日志都是写到buffer中，而从buffer写入磁盘速度很慢(文件io)，所以开个线程异步写
void AsyncLogging::asyncWriteLog() {
    assert(is_running_);
    //倒计时
    latch_.countDown();
    //log file
    LogFile log_file(filename_);
    auto new_buffer1 = std::make_shared<Buffer>();
    auto new_buffer2 = std::make_shared<Buffer>();
    new_buffer1->bzero();
    new_buffer2->bzero();
    std::vector<std::shared_ptr<Buffer>> buffers;
    buffers.reserve(16);

    while (is_running_) {
        assert(new_buffer1 && new_buffer1->buffSize() == 0);
        assert(new_buffer2 && new_buffer2->buffSize() == 0);
        assert(buffers.empty());
        {
            MutexLockGuard lock(mutex_);
            //这里等待buffer写满存入buffers
            if (buffers_.empty()) {
                cond_.waitSeconds(timeout_);
            }
            //buffers再加上刚才写的buffer
            buffers_.push_back(curBuffer_);
            curBuffer_.reset();
            curBuffer_ = std::move(new_buffer1);
            //将buffers move到本地来
            buffers.swap(buffers_);
            if (!nextBuffer_) {
                nextBuffer_ = std::move(new_buffer2);
            }
        }
        assert(!buffers.empty());
        
        if (buffers.size() > 25) {
            buffers.erase(buffers.begin() + 2, buffers.end());
        }

        //遍历buffers 每个buffer数据写入log文件
        for (int i = 0; i < buffers.size(); ++i) {
            log_file.Write(buffers[i]->getBuffer(), buffers[i]->buffSize());
        }

        if (!new_buffer1) {
            assert(!buffers.empty());
            new_buffer1 = buffers.back();
            buffers.pop_back();
            new_buffer1->reset();
        }

        if (!new_buffer2) {
            assert(!buffers.empty());
            new_buffer2 = buffers.back();
            buffers.pop_back();
            new_buffer2->reset();
        }

        //清理buffers 
        buffers.clear();
        //flush日志文件
        log_file.Flush();
    }
    log_file.Flush();
}
