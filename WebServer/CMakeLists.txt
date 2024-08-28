project(MyWebServer)
cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
add_subdirectory(base)

set(SRC
    Channel.cpp
    httpConnection.cpp
    timer.cpp
    TimerManager.cpp
    )

add_library(server ${SRC})
target_link_libraries(server libserver_base pthread rt)