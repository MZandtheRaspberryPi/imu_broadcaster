cmake_minimum_required(VERSION 3.16)

project(imu_broadcaster VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(NOT_ARDUINO TRUE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/imu_websockets)

add_library(${PROJECT_NAME}_lib STATIC
              src/Adafruit_BNO055.cpp
              src/Adafruit_Sensor.cpp
              src/pi4_i2c_bus.cpp
              )

target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
target_compile_definitions(${PROJECT_NAME}_lib
     PUBLIC NOT_ARDUINO
)

add_executable(read_from_sensor read_from_sensor.cpp)

target_link_libraries(read_from_sensor PUBLIC ${PROJECT_NAME}_lib imu_websockets_lib)
