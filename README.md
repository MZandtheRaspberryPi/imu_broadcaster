# imu_broadcaster
A repo that talks to a bno055 imu using adafruit libraries and broadcasts the data via websocket, serialized with protobuffer.


``
docker build -t i2c_test .
docker run -it --device /dev/i2c-1 i2c_test

```
```
mikey@compass:~/digital_compass$ i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- 1c -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

# configuring ubuntu
https://hackaday.com/2022/02/01/did-you-know-that-the-raspberry-pi-4-has-more-spi-i2c-uart-ports/
https://forums.raspberrypi.com//viewtopic.php?f=29&t=302381
https://forums.raspberrypi.com/viewtopic.php?t=248439
https://github.com/raspberrypi/linux/issues/4884

with default hardware i2c interface, can talk on pi 4. cpu usage 4-5%

if we use software i2c:
dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2,bus=3

 Enabling I2C3, with SDA on GPIO4 and SCL on GPIO5
nano /boot/firmware/config.txt
dtoverlay=i2c3,pins_4_5

