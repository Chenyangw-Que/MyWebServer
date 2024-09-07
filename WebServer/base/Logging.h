#ifndef LOG_LOGGING_H_
#define LOG_LOGGING_H_

#define OPEN_LOGGING
// 具体调用这块，首先前端每次写的时候都调用 LOG<<  此时相当于以RAII的风格初始化一个logging（loging里带一个logstream，同时logging能够访问到一个静态的asynclogging指针）
// logging 构造时会向4kbuffer中预先写入时间和当前文件，之后接受前端内容。logger析构时会将4kbuffer中的内容写入asynclogging里的前端缓冲区
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>

#include "logStream.h"


class Logging {
 public:
    Logging(const char* file_name, int line, int level);
    ~Logging();

    static std::string log_file_name() {
        return log_file_name_;
    }

    static void set_log_file_name(std::string log_file_name) {
        log_file_name_ = log_file_name;
    }

    static bool open_log() {
        return open_log_;
    }

    static void set_open_log(bool open_log) {
        open_log_ = open_log;
    }

    static void set_log_to_stderr(bool log_to_stderr) {
        log_to_stderr_ = log_to_stderr;
    }

    static void set_color_log_to_stderr(bool color_log_to_stderr) {
        color_log_to_stderr_ = color_log_to_stderr;
    }

    static void set_min_log_level(int min_log_level) {
        min_log_level_ = min_log_level;
    }

    logStream& stream() {
        return impl_.stream_;
    }

 private:
    class Impl {
     public:
        Impl(const char* file_name, int line, int level);
        void FormatLevel();
        void FormatTime();

        logStream stream_;
        std::string file_name_;
        int line_;
        int level_;
        std::string level_str_;
        std::string log_color_;
        char time_str_[26];
        bool is_fatal_;
    };

 private:
    static std::string log_file_name_;
    static bool open_log_;
    static bool log_to_stderr_;
    static bool color_log_to_stderr_;
    static int min_log_level_;

    Impl impl_;
};



enum LogLevel {
    DEBUG = 0,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

//宏定义
#ifdef OPEN_LOGGING

    #define LOG(level)  Logging(__FILE__, __LINE__, level).stream() 
    #define LOG_DEBUG   Logging(__FILE__, __LINE__, DEBUG).stream()
    #define LOG_INFO    Logging(__FILE__, __LINE__, INFO).stream()
    #define LOG_WARNING Logging(__FILE__, __LINE__, WARNING).stream()
    #define LOG_ERROR   Logging(__FILE__, __LINE__, ERROR).stream()
    #define LOG_FATAL   Logging(__FILE__, __LINE__, FATAL).stream()

#else
    #define LOG(level) LogStream()
#endif

#endif  // LOG_LOGGING_H_
