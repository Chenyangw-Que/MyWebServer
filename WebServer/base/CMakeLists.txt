set(LIB_SRC
    CountDownLatch.cpp
    logUtil.cpp
    mutexLock.cpp
    Thread.cpp
    logStream.cpp
    AsyncLogging.cpp
    Logging.cpp
    socketUtils.cpp
    MemoryPool.cpp
    LFUCache.cpp
    httpDefine.cpp
    httpDataUtil.cpp
)

add_library(base64 base64.cpp)

find_package(OpenCV REQUIRED)
include_directories(libserver_base ${OpenCV_INCLUDE_DIRS})
add_library(libserver_base ${LIB_SRC})
target_link_libraries(libserver_base pthread rt jsoncpp ${OpenCV_LIBS} base64)

set_target_properties(libserver_base PROPERTIES OUTPUT_NAME "server_base")
add_subdirectory(test)
