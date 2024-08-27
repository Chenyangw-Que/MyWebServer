#include"base/noncopyable.h"
#include "timer.h"
#include <memory>
class httpConnection:noncopyable{
public:
int connectfd();
void Delete();
void setTimer(std::shared_ptr<TimerNode> timer);
};