#include "Thread.h"
#include <assert.h>
#include <sys/prctl.h>
namespace threadData {
__thread pid_t cachePid = 0;
__thread const char *threadName;
// 返回线程pid
inline pid_t getpid() {
  if (__builtin_expect(cachePid == 0, 0)) {
    cachePid = static_cast<pid_t>(::syscall(SYS_gettid));
  }
  return cachePid;
}
inline const char *getThreadName() { return threadName; }
} // namespace threadData

thread::thread(const threadFunc &func, const std::string &name)
    : func_(func), threadId_(0), tid_(0), started_(false), joined_(false),
      threadName_(name) {
  if (threadName_.empty()) {
    threadName_ = "Thread";
  }
}

thread::~thread() {
  // 没在等待
  if (started_ && !joined_)
    pthread_detach(threadId_);
}

void thread::start() {
  // 首先判断一些状态信息
  assert(!started_);
  assert(!joined_);
  // 其实就是创建线程，并执行启动函数
  started_ = true;
  if ((pthread_create(&threadId_, NULL, &run, this)) != 0) {
    started_ = false;
  }
}

int thread::join() {
  assert(started_);
  assert(!joined_);
  joined_ = true;
  return pthread_join(threadId_, NULL);
}

void *thread::run(void *args) {
  // 解析参数，这里之间把本体扔进去了
  thread *td = static_cast<thread *>(args);
  // 调用赋值函数
  td->cacheData();
  // 调用任务函数
  td->func_();
  threadData::threadName = "finished";
  return NULL;
}

void thread::cacheData() {
  tid_ = threadData::getpid();
  threadData::threadName = threadName_.empty() ? "Thread" : threadName_.c_str();
  // 修改线程名称
  prctl(PR_SET_NAME, threadData::threadName);
}
