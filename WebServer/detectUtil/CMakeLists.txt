project(detMark)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)


add_definitions(-D LINK_OPENCV_LIB_AUTO)
find_package(OpenCV REQUIRED)



set(OPENVINO_INCLUDE /opt/intel/openvino_2023/include)
set(OPENVINO_LIB /opt/intel/openvino_2023/lib)
include_directories(${OPENVINO_INCLUDE})
link_directories(${OPENVINO_LIB})
link_libraries(libopenvino.so)

add_executable(main)
file(GLOB sources CONFIGURE_DEPENDS *.cpp *.h)
include_directories(main ${OpenCV_INCLUDE_DIRS})

target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main libopenvino.so)
target_sources(main PUBLIC ${sources})

add_library(detectMark ${sources})
target_link_libraries(detectMark libopenvino.so ${OpenCV_LIBS})

