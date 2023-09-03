#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "imu_msgs.pb.h"
#include "pi4_i2c_bus.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "websocket_broadcaster.h"


/* Set the delay between fresh samples */
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 2;
bool EXIT_FLAG = false;

struct sensor_broadcaster_args_t {
  std::string i2c_device_address;
  uint8_t i2c_chip_address;
  uint16_t websocket_port;
  int8_t err_flag;
};


void my_signal_handler(int s)
{
  printf("Caught signal %d\n", s);
  EXIT_FLAG = true;
}


void setup_bno(Adafruit_BNO055& bno)
{

  log_msg("Trying to Connect to the BNO055 Sensor");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    log_msg("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    exit(1);
  }

  delay(1000);
}

void setup_sigint_handler(struct sigaction& sig_int_handler)
{
  sig_int_handler.sa_handler = my_signal_handler;
  sigemptyset(&sig_int_handler.sa_mask);
  sig_int_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_int_handler, NULL);
}

sensor_broadcaster_args_t parse_args(const int& argc, char* argv[])
{
  if (argc != 4) {
    sensor_broadcaster_args_t parsed_args = {"", 0, 0, -1};
    std::cerr << "Usage: " << argv[0] << " /dev/i2c-3 0x28 9000" <<  std::endl;
    std::cerr << "  arg: " << "i2c_device_addres (ex: /dev/i2c-3)" << std::endl;
    std::cerr << "  arg: " << "i2c_chip_address (ex: 0x28)" << std::endl;
    std::cerr << "  arg: " << "websocket_port (ex: 9000)" << std::endl;
    return parsed_args;
  }

  std::string i2c_device_address = argv[1];
  std::string i2c_chip_address_str = argv[2];
  std::string websocket_port_str = argv[3];

  unsigned long int i2c_chip_address = std::strtoul(i2c_chip_address_str.c_str(), nullptr, 0);
  unsigned long int websocket_port = std::strtoul(websocket_port_str.c_str(), nullptr, 0);

  sensor_broadcaster_args_t parsed_args{i2c_device_address, static_cast<uint8_t>(i2c_chip_address), static_cast<uint16_t>(websocket_port), 0};

  std::cout << "args parsed:" << std::endl;
  std::cout << "i2c_device_address: " << parsed_args.i2c_device_address << std::endl;
  std::ios::fmtflags f(std::cout.flags());
  std::cout << "i2c_chip_address: " << "0x" << std::hex << static_cast<int>(parsed_args.i2c_chip_address) << std::endl;
  std::cout.flags(f);
  std::cout << "websocket_port: " << parsed_args.websocket_port << std::endl;
  delay(2000);
  return parsed_args;

}

void write_vect_to_triad(const imu::Vector<3>& vect, imu_msgs::Triad* triad)
{
  triad->set_x(vect[0]);
  triad->set_y(vect[1]);
  triad->set_z(vect[2]);
  return;
}

int main(int argc, char* argv[])
{

  sensor_broadcaster_args_t parsed_args = parse_args(argc, argv);
  if (parsed_args.err_flag == -1)
  {
    return -1;
  }

  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  struct sigaction sig_int_handler;
  setup_sigint_handler(sig_int_handler);

  // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
  //                                   id, address
  Adafruit_BNO055 bno = Adafruit_BNO055(55, parsed_args.i2c_chip_address, parsed_args.i2c_device_address);
  setup_bno(bno);

  BroadcastServer broadcast_server = BroadcastServer();
  broadcast_server.start_listening(parsed_args.websocket_port);

  while (!EXIT_FLAG)
  {

    uint64_t start_time = millis();

    imu_msgs::ImuMsg imu_msg;
    imu_msg.set_timestamp(start_time);

    // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    imu::Vector<3> sensor_data = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu_msgs::Triad* euler_angles = imu_msg.mutable_euler_angles();
    write_vect_to_triad(sensor_data, euler_angles);

    sensor_data = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu_msgs::Triad* linear_acceleration = imu_msg.mutable_linear_acceleration();
    write_vect_to_triad(sensor_data, linear_acceleration);

    sensor_data = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msgs::Triad* angular_acceleration = imu_msg.mutable_angular_acceleration();
    write_vect_to_triad(sensor_data, angular_acceleration);

    sensor_data = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu_msgs::Triad* magnetometer_vector = imu_msg.mutable_magnetometer_vector();
    write_vect_to_triad(sensor_data, magnetometer_vector);

    imu::Quaternion quat = bno.getQuat();
    imu_msgs::Quaternion* quaternion_ptr = imu_msg.mutable_ground_truth();
    quaternion_ptr->set_w(quat.w());
    imu_msgs::Triad* quat_xyz_ptr = quaternion_ptr->mutable_xyz();
    quat_xyz_ptr->set_x(quat.x());
    quat_xyz_ptr->set_y(quat.y());
    quat_xyz_ptr->set_z(quat.z());

    int8_t board_temp = bno.getTemp();
    imu_msg.set_board_temp(board_temp);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    imu_msg.set_system_calibration(system);
    imu_msg.set_gyro_calibration(gyro);
    imu_msg.set_accel_calibration(accel);
    imu_msg.set_mag_calibration(mag);

    // std::string debug_str = imu_msg.DebugString();
    // std::cout << debug_str << std::endl;

    size_t msg_size = imu_msg.ByteSizeLong();
    uint8_t* msg_arr = new uint8_t[msg_size];
    imu_msg.SerializeToArray(msg_arr, msg_size);
    broadcast_server.send_message(msg_arr, msg_size);
    delete[] msg_arr;

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }

  return 0;
}
