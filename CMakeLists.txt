cmake_minimum_required(VERSION 3.23.0-rc4)
project(item_sorting_system)

set(CMAKE_CXX_STANDARD 11)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)#寻找OpenCV所需要的包
include_directories(${OpenCV_INCLUDE_DIRS})#引用OpenCV的include

add_executable(item_sorting_system main.cpp uart.cpp)
target_link_libraries(item_sorting_system ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES} Threads::Threads)#链接OpenCV库
