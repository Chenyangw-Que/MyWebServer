project(MyWebServer)
cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
add_subdirectory(base)
add_subdirectory(detectUtil)
set(SRC
    Channel.cpp
    httpConnection.cpp
    timer.cpp
    TimerManager.cpp
    Poller.cpp
    EventLoop.cpp
    EventThread.cpp
    EventThreadPool.cpp
    mainReactor.cpp
    cgiHandler.cpp
    )
set(CMAKE_BUILD_TYPE Release)
add_library(server ${SRC})
target_link_libraries(server libserver_base pthread rt jsoncpp)

add_executable(WebServer WebServer.cpp)
target_link_libraries(WebServer libserver_base server detectMark)