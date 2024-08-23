#include "socketUtils.h"
#include "Logging.h"
#include "arpa/inet.h"
#include "netinet/in.h"
#include <errno.h>
#include <unistd.h>

const int MAX_BUFFER_SIZE = 4096;

int getListenFd(int port) {
  // 输入检测
  if (port < 0 || port > 65535) {
    LOG(FATAL) << "Error Port While Create Listenfd";
  }

  // 创建socket
  int listenfd = socket(AF_INET, SOCK_STREAM, 0); // IPV4 TCP
  //检测创建是否成功
  if (listenfd == -1)
    LOG(FATAL) << "Create Listenfd Failed, " << strerror(errno);
  // 有时所选端口可能没有被释放，这时考虑使用setsockopt接触端口号绑定限制
  int flag = 1;
  if ((setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) ==
      -1) {
    LOG(FATAL) << "Set Socket Option Error " << strerror(errno);
  }

  // 建立网络地址结构体
  struct sockaddr_in serverAddr;
  memset(&serverAddr, 0, sizeof serverAddr);
  // 进行结构提填充
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddr.sin_port = htons((unsigned short)(port));
  if ((bind(listenfd, (struct sockaddr *)&serverAddr, sizeof serverAddr)) ==
      -1) {
    LOG(FATAL) << "Bind Port Failed " << strerror(errno);
    close(listenfd);
  }

  // 开始监听
  if (listen(listenfd, 2048) == -1) {
    LOG(FATAL) << "Listen Port Failed " << strerror(errno);
    close(listenfd);
  }

  if (listenfd == -1) {
    LOG(FATAL) << "Invalid Listenfd " << strerror(errno);
    close(listenfd);
  }

  return listenfd;
}

int sockWrite(int fd, void *writeBuffer, int size) {
  int writeOneLoop = 0;
  int writeSum = 0;
  char *buffer = (char *)writeBuffer;
  // epoll+ET的前提下写完写到eagain为止
  while (size > 0) {
    if ((writeOneLoop = write(fd, buffer, size)) <= 0) {
      // 判断时因为中断，还是因为写完了
      if (writeOneLoop < 0) {
        if (errno == EINTR) {
          writeOneLoop = 0; //还原一下进度
          continue;
        } else if (errno == EAGAIN) {
          return writeSum;
        } else {
          return -1;
        }
      }
    }
    // 移动缓冲器指针，更新剩余buffer和写入总量
    writeSum += writeOneLoop;
    size -= writeOneLoop;
    buffer += writeOneLoop;
  }
  return writeSum;
}

int sockWrite(int fd, std::string &writeBuffer) {
  int size = writeBuffer.size();
  int writeOneLoop = 0;
  int writeSum = 0;
  const char *buffer = writeBuffer.c_str();
  // epoll+ET的前提下写完写到eagain为止
  while (size > 0) {
    if ((writeOneLoop = write(fd, buffer, size)) <= 0) {
      // 判断时因为中断，还是因为写完了
      if (writeOneLoop < 0) {
        if (errno == EINTR) {
          writeOneLoop = 0; //还原一下进度
          continue;
        } else if (errno == EAGAIN) {
          // EAGAIN表明缓冲区数据写完了,得等一等了
          break;
        } else {
          return -1;
        }
      }
    }
    // 移动缓冲器指针，更新剩余buffer和写入总量
    writeSum += writeOneLoop;
    size -= writeOneLoop;
    buffer += writeOneLoop;
  }
  if (writeSum == writeBuffer.size()) {
    writeBuffer.clear();
  } else {
    writeBuffer = writeBuffer.substr(writeSum);
  }
  return writeSum;
}

int sockRead(int fd, void *readBuffer, int size) {
  int readOneLoop = 0;
  int readSum = 0;
  char *buffer = (char *)readBuffer;
  while (size > 0) {
    if ((readOneLoop = read(fd, buffer, size)) < 0) {
      // 判断问题
      if (errno == EINTR) {
        readOneLoop = 0;
        continue;
      } else if (errno == ERANGE) {
        return readSum;
      } else {
        return -1;
      }
    } else if (readOneLoop == 0) {
      break;
    }

    readSum += readOneLoop;
    size -= readOneLoop;
    buffer += readOneLoop;
  }
  return readOneLoop;
}

int sockRead(int fd, std::string &readBuffer, bool &isReadZero) {
  int readOneLoop = 0;
  int readSum = 0;
  while (true) {
    char buffer[MAX_BUFFER_SIZE];
    if ((readOneLoop = read(fd, buffer, MAX_BUFFER_SIZE)) < 0) {
      // 判断问题
      if (errno == EINTR) {
        readOneLoop = 0;
        continue;
      } else if (errno == ERANGE) {
        return readSum;
      } else {
        return -1;
      }
    } else if (readOneLoop == 0) {
      isReadZero = true;
      break;
    }
    readSum += readOneLoop;
    readBuffer += std::string(buffer, buffer + readOneLoop);
  }
  return readSum;
}

int sockRead(int fd, std::string &readBuffer) {
  int readOneLoop = 0;
  int readSum = 0;
  while (true) {
    char buffer[MAX_BUFFER_SIZE];
    if ((readOneLoop = read(fd, buffer, MAX_BUFFER_SIZE)) < 0) {
      // EINTR是中断引起的 所以重新读就行
      if (errno == EINTR) {
        continue;
      } else if (errno == EAGAIN) {
        // EAGAIN表明数据读完了
        return readSum;
      } else {
        LOG(ERROR) << "Read from sockfd error, " << strerror(errno);
        return -1;
      }
    } else if (readOneLoop == 0) {
      break;
    }

    readSum += readOneLoop;
    readBuffer += std::string(buffer, buffer + readOneLoop);
  }

  return readSum;
}

