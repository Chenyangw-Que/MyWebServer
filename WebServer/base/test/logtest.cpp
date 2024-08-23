// @Author Lin Ya
// @Email xxbbb@vip.qq.com
#include "../Logging.h"
#include "../Thread.h"
#include <string>
#include <unistd.h>
#include <vector>
#include <memory>
#include <iostream>
using namespace std;

void threadFunc()
{
    for (int i = 0; i < 100000; ++i)
    {
        LOG(INFO) << i;
    }
}

void type_test()
{
    // 13 lines
    cout << "----------type test-----------" << endl;
    LOG(INFO) << 0;
    LOG(INFO) << 1234567890123;
    LOG(INFO) << 1.0f;
    LOG(INFO) << 3.1415926;
    LOG(INFO) << (short) 1;
    LOG(INFO) << (long long) 1;
    LOG(INFO) << (unsigned int) 1;
    LOG(INFO) << (unsigned long) 1;
    LOG(INFO) << (long double) 1.6555556;
    LOG(INFO) << (unsigned long long) 1;
    LOG(INFO) << 'c';
    LOG(INFO) << "abcdefg";
    //LOG(INFO) << string("This is a string");
}

void stressing_single_thread()
{
    // 100000 lines
    cout << "----------stressing test single thread-----------" << endl;
    for (int i = 0; i < 100000; ++i)
    {
        LOG(INFO) << i;
    }
}

void stressing_multi_threads(int threadNum = 4)
{
    // threadNum * 100000 lines
    cout << "----------stressing test multi thread-----------" << endl;
    vector<shared_ptr<thread>> vsp;
    for (int i = 0; i < threadNum; ++i)
    {
        shared_ptr<thread> tmp(new thread(threadFunc, "testFunc"));
        vsp.push_back(tmp);
    }
    for (int i = 0; i < threadNum; ++i)
    {
        vsp[i]->start();
    }
    sleep(3);
}

void other()
{
    // 1 line
    cout << "----------other test-----------" << endl;
    LOG(INFO) << "fddsa" << 'c' << 0 << 3.666 << std::string("This is a string").c_str();
}


int main()
{
    log::Logging::set_log_file_name("./weblog.log");
    log::Logging::set_open_log(true);
    // 共500014行
    type_test();
    sleep(3);

    stressing_single_thread();
    sleep(3);

    other();
    sleep(3);

    stressing_multi_threads();
    sleep(3);
    return 0;
}