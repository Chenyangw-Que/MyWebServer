#pragma once
// log这块关键是对缓冲区的操作，从线程缓冲区到
// log处理缓冲区，再由log处理缓冲区写入文件 首先封装文件操作
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
