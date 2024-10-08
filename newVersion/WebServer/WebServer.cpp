#include <getopt.h>

#include "mainReactor.h"
#include "EventLoop.h"
#include "base/Logging.h"
#include "base/MemoryPool.h"
#include "base/LFUCache.h"

namespace configure {
//默认值
static int port = 8888;
static int thread_num = 8;
static std::string log_file_name = "./web_server.log";
static bool open_log = true;
static bool log_to_stderr = false;
static bool color_log_to_stderr = false;
static int min_log_level = INFO;
static int capacity = 10;

static void ParseArg(int argc, char* argv[]) {
    int opt;
    const char* str = "p:t:f:o:s:c:l:d:";
    while ((opt = getopt(argc, argv, str)) != -1) {
        switch (opt) {
            case 'p': {
                port = atoi(optarg);
                break;
            }
            case 't': {
                thread_num = atoi(optarg);
                break;
            }
            case 'f': {
                log_file_name = optarg;
                break;
            }
            case 'o': {
                open_log = atoi(optarg);
                break;
            }
            case 's': {
                log_to_stderr = atoi(optarg);
                break;
            }
            case 'c': {
                color_log_to_stderr = atoi(optarg);
                break;
            }
            case 'l': {
                min_log_level = atoi(optarg);
                break;
            }
            case 'd': {
                capacity = atoi(optarg);
                break;
            }
            default: {
                break;
            }
        }
    }
}

}  // namespace configure

int main(int argc, char* argv[]) {
    //解析参数
    configure::ParseArg(argc, argv);
    
    // 设置日志文件
    Logging::set_log_file_name(configure::log_file_name);
    // 开启日志
    Logging::set_open_log(configure::open_log);
    // 设置日志输出标准错误流
    Logging::set_log_to_stderr(configure::log_to_stderr);
    // 设置日志输出颜色
    Logging::set_color_log_to_stderr(configure::color_log_to_stderr);
    // 设置最小日志等级
    Logging::set_min_log_level(configure::min_log_level);
    // 初始化内存池
    QueMemory::init();
    // 初始化缓存
    LFUCache::GetInstance().Initialize(configure::capacity);

    // 主loop  初始化poller, 把eventfd注册到epoll中并注册其事件处理回调
    EventLoop mainloop;

    // 创建监听套接字绑定服务器，监听端口，设置监听套接字为NIO，屏蔽管道信号
    mainReactor::GetInstance().Initialize(&mainloop, configure::thread_num, configure::port);
    
    mainReactor::GetInstance().Start();
    // myHttpServer.Start();

    // 主loop开始事件循环  epoll_wait阻塞 等待就绪事件(主loop只注册了监听套接字的fd，所以只会处理新连接事件)
    std::cout << "================================================Start Web Server================================================" << std::endl;
    mainloop.Loop();
    std::cout << "================================================Stop Web Server=================================================" << std::endl;

    return 0;
}
