set(LIB_SRC
    CountDownLatch.cpp
    logUtil.cpp
    mutexLock.cpp
    Thread.cpp
    logStream.cpp
    AsyncLogging.cpp
    Logging.cpp
    socketUtils.cpp
    )

add_library(libserver_base ${LIB_SRC})
target_link_libraries(libserver_base pthread rt)

set_target_properties(libserver_base PROPERTIES OUTPUT_NAME "server_base")
add_subdirectory(test)
