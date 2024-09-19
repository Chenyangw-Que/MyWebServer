add_executable(logtest logtest.cpp)
target_link_libraries(logtest libserver_base)

add_executable(testSkiplist testSkiplist.cpp)
target_link_libraries(testSkiplist -pthread)

add_executable(testMemoryPool memoryPooltest.cpp)
target_link_libraries(testMemoryPool libserver_base)
