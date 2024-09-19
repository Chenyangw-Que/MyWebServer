#include "logStream.h"
#include "string.h"
#include <stdio.h>
#include <string>
#include <algorithm>
#include<assert.h>
logStream &logStream::operator<<(char log) {
  buffer_.writebuff(&log, 1);
  return *this;
}

logStream &logStream::operator<<(const char *log) {
  buffer_.writebuff(log, strlen(log));
  return *this;
}

logStream &logStream::operator<<(unsigned const char *log) {
  return operator<<(reinterpret_cast<const char *>(log));
}

logStream &logStream::operator<<(std::string &log) {
  buffer_.writebuff(log.c_str(), log.size());
  return *this;
}

logStream &logStream::operator<<(bool log) {
  return operator<<(log ? "1" : "0");
}

static const char digits[] = "9876543210123456789";
static const char *zero = digits + 9;

// 将数字转换为字符
template <typename T> void logStream::IntegerToBuffer(T number) {
  if (buffer_.preserveSize() >= kMaxNumberSize) {
    // 写入buffer

    char *cur = buffer_.getCurPos();
    char *buf = cur;
    assert(buf);
    do {
      int index = static_cast<int>(number % 10);
      number /= 10;
      *(buf++) = zero[index];
    } while (number != 0);

    if (number < 0) {
      *buf++ = '-';
    }
    *buf = '\0';
    std::reverse(cur, buf);
    buffer_.moveForward(buf - cur);
  }
}

logStream &logStream::operator<<(int log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(unsigned int log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(long log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(unsigned long log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(long long log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(unsigned long long log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(short log) {
  IntegerToBuffer(log);
  return *this;
}
logStream &logStream::operator<<(unsigned short log) {
  IntegerToBuffer(log);
  return *this;
}

logStream &logStream::operator<<(double log) {
  if (buffer_.preserveSize() >= kMaxNumberSize) { //太长的数也丢弃
    int size = snprintf(buffer_.getCurPos(), kMaxNumberSize, "%.12Lg", log);
    buffer_.moveForward(size);
  }
  return *this;
}
logStream &logStream::operator<<(long double log) {
  if (buffer_.preserveSize() >= kMaxNumberSize) { //太长的数也丢弃
    int size = snprintf(buffer_.getCurPos(), kMaxNumberSize, "%.12Lg", log);
    buffer_.moveForward(size);
  }
  return *this;
}
logStream &logStream::operator<<(float log) {
  return operator<<(static_cast<double>(log));
}
