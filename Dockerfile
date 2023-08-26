FROM ubuntu:22.04

RUN apt-get update
RUN apt-get install i2c-tools nano git libi2c-dev libboost-all-dev cmake build-essential g++ protobuf-compiler -y

RUN mkdir /imu_broadcaster
COPY CMakeLists.txt /imu_broadcaster/CMakeLists.txt
COPY Adafruit_BNO055/Adafruit_BNO055.h /imu_broadcaster/include/Adafruit_BNO055.h
COPY Adafruit_BNO055/Adafruit_BNO055.cpp /imu_broadcaster/src/Adafruit_BNO055.cpp
COPY read_from_sensor.cpp /imu_broadcaster/read_from_sensor.cpp
COPY Adafruit_BNO055/utility /imu_broadcaster/include/utility

COPY Adafruit_Sensor/Adafruit_Sensor.h /imu_broadcaster/include/Adafruit_Sensor.h
COPY Adafruit_Sensor/Adafruit_Sensor.cpp /imu_broadcaster/src/Adafruit_Sensor.cpp

COPY Adafruit_BNO055/pi4_i2c_bus.h /imu_broadcaster/include/pi4_i2c_bus.h
COPY Adafruit_BNO055/pi4_i2c_bus.cpp /imu_broadcaster/src/pi4_i2c_bus.cpp

COPY websocketpp/websocketpp /imu_broadcaster/include/websocketpp
COPY websocket_broadcaster.h /imu_broadcaster/include/websocket_broadcaster.h


COPY imu_msgs.proto /imu_broadcaster/imu_msgs.proto
RUN mkdir /imu_broadcaster/proto_msg
RUN protoc -I=/imu_broadcaster/ --cpp_out=/imu_broadcaster/proto_msg /imu_broadcaster/imu_msgs.proto

RUN ls /imu_broadcaster/proto_msg

WORKDIR /imu_broadcaster
RUN mkdir build

WORKDIR /imu_broadcaster/build
RUN cmake ..
RUN make
RUN ls -ltrh

STOPSIGNAL SIGINT

ENTRYPOINT ["/imu_broadcaster/build/read_from_sensor", "/dev/i2c-3", "0x28", "9000"]
