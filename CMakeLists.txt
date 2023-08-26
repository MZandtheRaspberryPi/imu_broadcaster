cmake_minimum_required(VERSION 3.16)

project(imu_broadcaster VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(NOT_ARDUINO TRUE)

set(CMAKE_FIND_LIBRARY_SUFFIXES .a)

find_package (Protobuf REQUIRED)

add_library(${CMAKE_PROJECT_NAME}_lib STATIC
              src/Adafruit_BNO055.cpp
              src/Adafruit_Sensor.cpp
              src/pi4_i2c_bus.cpp
              proto_msg/imu_msgs.pb.cc
              )

target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC include proto_msg)
target_compile_definitions(${CMAKE_PROJECT_NAME}_lib
     PUBLIC NOT_ARDUINO
)

target_link_libraries(${CMAKE_PROJECT_NAME}_lib ${PROTOBUF_LIBRARIES})

add_executable(read_from_sensor read_from_sensor.cpp)

target_link_libraries(read_from_sensor PUBLIC ${CMAKE_PROJECT_NAME}_lib)
