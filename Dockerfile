FROM mzandtheraspberrypi/imu_websocket_broadcaster:build-2023-09-10 as build

RUN apt-get update
RUN apt-get install i2c-tools nano git libi2c-dev libboost-all-dev cmake build-essential g++ -y

COPY . /tmp/repo

WORKDIR /tmp/repo
RUN chmod +x create_expected_dir_structure.sh
RUN ./create_expected_dir_structure.sh

WORKDIR /tmp/repo/imu_broadcaster_expected_structure
RUN mkdir build

WORKDIR /tmp/repo/imu_broadcaster_expected_structure/build
RUN cmake -Dabsl_DIR=/abseil/CMakeProject/install/lib/cmake/absl ..
RUN make
RUN ls -ltrh

FROM ubuntu:22.04

COPY --from=build /tmp/repo/imu_broadcaster_expected_structure/build/read_from_sensor /entrypoint/read_from_sensor
COPY --from=build /tmp/repo/imu_broadcaster_expected_structure/build/imu_websockets/libimu_websockets_lib.so /usr/local/lib/libimu_websockets_lib.so
COPY --from=build /usr/local/lib/libprotobuf.so.24.3.0 /usr/local/lib/


ENV LD_LIBRARY_PATH=/usr/local/lib

STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint/read_from_sensor"]
