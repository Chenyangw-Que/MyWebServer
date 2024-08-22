#include "logUtil.h"
#include <errno.h>
#include <stdio.h>
// 文件写工具的构造和析构关键就是文件描述符的获取与释放
logUtils::logUtils(std::string filename) : fp_(fopen(filename.c_str(), "ae")) {
  setbuffer(fp_, buffer_, sizeof(buffer_)); // 将buffer设置为写缓冲区
}

logUtils::~logUtils() { fclose(fp_); }

// 提高效率先写入写缓冲区，在适当的场合使用flush进行批量磁盘写入
void logUtils::writeFileBase(const char *singleLog, const size_t size) {
  // 无锁写，互斥写由上层包装实现
  size_t writen_len = fwrite_unlocked(singleLog, 1, size, fp_);
  size_t remain_size = size - writen_len;
  while (remain_size > 0) {
    size_t bytes = fwrite_unlocked(singleLog + writen_len, 1, remain_size, fp_);
    if (bytes == 0) {
      if (ferror(fp_)) {
        perror("write log to writebuf failed");
      }
      break;
    }
    writen_len += bytes;
    remain_size = size - bytes;
  }
}

void logUtils::flush() { fflush(fp_); }