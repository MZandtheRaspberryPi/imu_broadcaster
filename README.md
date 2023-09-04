# imu_broadcaster
![example workflow](https://github.com/MZandtheRaspberryPi/imu_broadcaster/actions/workflows/pipeline.yaml/badge.svg)  
A repo that talks to a bno055 imu using adafruit libraries and broadcasts the data via websocket, serialized with protobuffer.

```
git clone git@github.com:MZandtheRaspberryPi/imu_broadcaster.git --recurse-submodules
```

## Running
The script takes as input the i2c device address, and the chip address (the adafrui bno055 has a default address of 0x28), and the websocket port to publish messages to.  
```
docker run -it --device /dev/i2c-3 mzandtheraspberrypi/imu_websocket_broadcaster:2023-08-26 /dev/i2c-3 0x28 9001
```
```
docker run -it --device /dev/i2c-3 mzandtheraspberrypi/imu_websocket_broadcaster:2023-08-26 /dev/i2c-3 0x28 9001
0x28
9001
40
9001
args parsed:
i2c_device_address: /dev/i2c-3
i2c_chip_address: 0x28
websocket_port: 9001
Trying to Connect to the BNO055 Sensor
Failed to write then read to 40
...
Failed to write then read to 40
timestamp: 1693085201730
ground_truth {
  w: 0.999267578125
  xyz {
    x: -0.03521728515625
    y: 0.0135498046875
    z: 0
  }
}
euler_angles {
  x: 0
  y: -1.5
  z: 4
}
linear_acceleration {
  x: -0.22
  y: -0.65
  z: 9.54
}
angular_acceleration {
  x: 0.0625
  y: 0
  z: -0.125
}
magnetometer_vector {
  x: -26.0625
  y: 14.375
  z: -15.0625
}
board_temp: 31
system_calibration: 0
gyro_calibration: 3
accel_calibration: 0
mag_calibration: 0

timestamp: 1693085201738
ground_truth {
  w: 0.999267578125
  xyz {
    x: -0.03521728515625
    y: 0.0135498046875
    z: 0
  }
}
euler_angles {
  x: 0
  y: -1.5
  z: 4
}
linear_acceleration {
  x: -0.25
  y: -0.67
  z: 9.54
}
angular_acceleration {
  x: 0.125
  y: -0.0625
  z: 0.0625
}
magnetometer_vector {
  x: -26.0625
  y: 14.375
  z: -15.0625
}
board_temp: 31
system_calibration: 0
gyro_calibration: 3
accel_calibration: 0
mag_calibration: 0
```

### Checking I2C Ports
Note here I am checking the 3 i2c device. I have setup software i2c to try and avoid some clock stretching issues on the Raspberry Pi 4 (see configuring ubuntu section below for more details).  

```
sudo apt-get install i2c-tools
i2cdetect -y 3
```
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --

```

### configuring ubuntu
https://hackaday.com/2022/02/01/did-you-know-that-the-raspberry-pi-4-has-more-spi-i2c-uart-ports/
https://forums.raspberrypi.com//viewtopic.php?f=29&t=302381
https://forums.raspberrypi.com/viewtopic.php?t=248439
https://github.com/raspberrypi/linux/issues/4884

if we use software i2c, add the second line to the file in the first line. configuring software i2c is detailed in the 4th link, the github issue link.

```
nano /boot/firmware/config.txt
dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2,bus=1
```


or to keep hardware i2c, pins 2,3, and add software one
```
dtparam=i2c_arm=on
dtoverlay=i2c-gpio,i2c_gpio_sda=4,i2c_gpio_scl=5,i2c_gpio_delay_us=2,bus=3
```

```
m@compass:~$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
m@compass:~$ i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --


```