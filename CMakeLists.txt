cmake_minimum_required(VERSION 2.8)
project(zr300_data_reader)

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(realsense REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${Realsense_INCLUDE_DIRS}
)

add_executable(zr300_data_reader   
    src/main.cpp)

target_link_libraries(zr300_data_reader
    ${RealSense_LIBRARIES}
    ${OpenCV_LIBS}
)