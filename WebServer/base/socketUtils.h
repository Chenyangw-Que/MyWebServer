#pragma once
#include <string>

// 封装基本的socket操作
int getListenFd(int port); // 绑定端口，获取listenfd

int sockWrite(int fd, void *writeBuffer, int size);

int sockWrite(int fd, std::string &writeBuffer);

int sockRead(int fd, void *readBuffer, int size);

int sockRead(int fd, std::string &readBuffer, bool &isReadZero);

int sockRead(int fd, std::string &readBuffer);

// 设置非阻塞模式
int setSocketNonBlocking(int fd);

void setSocketNoDelay(int fd);

void ShutDownWR(int fd);

void HandlePipeSignal();
