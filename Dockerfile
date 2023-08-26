FROM ubuntu:22.04 as build

RUN apt-get update
RUN apt-get install i2c-tools nano git libi2c-dev libboost-all-dev cmake build-essential g++ protobuf-compiler -y

COPY . /tmp/repo

WORKDIR /tmp/repo
RUN chmod +x create_expected_dir_structure.sh
RUN ./create_expected_dir_structure.sh

RUN protoc -I=/tmp/repo/imu_broadcaster_expected_structure/ --cpp_out=/tmp/repo/imu_broadcaster_expected_structure/proto_msg /tmp/repo/imu_broadcaster_expected_structure/imu_msgs.proto

WORKDIR /tmp/repo/imu_broadcaster_expected_structure
RUN mkdir build

WORKDIR /tmp/repo/imu_broadcaster_expected_structure/build
RUN cmake ..
RUN make
RUN ls -ltrh

FROM ubuntu:22.04

COPY --from=build /tmp/repo/imu_broadcaster_expected_structure/build/read_from_sensor /entrypoint/read_from_sensor

STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint/read_from_sensor"]
