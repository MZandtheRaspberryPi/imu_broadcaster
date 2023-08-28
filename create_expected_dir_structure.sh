#!/bin/bash

set -eux o pipefail

mkdir -p ./imu_broadcaster_expected_structure
mkdir -p ./imu_broadcaster_expected_structure/include
mkdir -p ./imu_broadcaster_expected_structure/src

cp ./CMakeLists.txt ./imu_broadcaster_expected_structure
cp ./Adafruit_BNO055/Adafruit_BNO055.h ./imu_broadcaster_expected_structure/include/Adafruit_BNO055.h
cp ./Adafruit_BNO055/Adafruit_BNO055.cpp ./imu_broadcaster_expected_structure/src/Adafruit_BNO055.cpp
cp ./read_from_sensor.cpp ./imu_broadcaster_expected_structure/read_from_sensor.cpp
cp -r ./Adafruit_BNO055/utility ./imu_broadcaster_expected_structure/include/utility

cp ./Adafruit_Sensor/Adafruit_Sensor.h ./imu_broadcaster_expected_structure/include/Adafruit_Sensor.h
cp ./Adafruit_Sensor/Adafruit_Sensor.cpp ./imu_broadcaster_expected_structure/src/Adafruit_Sensor.cpp

cp ./Adafruit_BNO055/pi4_i2c_bus.h ./imu_broadcaster_expected_structure/include/pi4_i2c_bus.h
cp ./Adafruit_BNO055/pi4_i2c_bus.cpp ./imu_broadcaster_expected_structure/src/pi4_i2c_bus.cpp


cp -r ./imu_websockets ./imu_broadcaster_expected_structure/imu_websockets