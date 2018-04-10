# avrlib_i2c
Yet Another I2C bus library for AVR devices

# API

### Add a device to the bus

	I2C my_i2c_device {I2C_ADDRESS};

### Remove the device from the bus

Just delete the I2C object.

### Send data to the device

	my_i2c_device.tx(byte, address_space, stop);

where:
 * byte - How many bytes to send
 * address_space - \*ptr to the data to be sent.
 * stop - send the stop bit (default=true, false)

### Receive data from the device

	my_i2c_device.rx(byte, address_space, stop);

where:
 * byte - bytes to receive
 * address_space - \*ptr to the data to be received.
 * stop - send the stop bit (default=true, false)

### Example compiling C++ code:
avr-g++ -Wall -pedantic -std=c++11 -mmcu=atmega1284p -O2 -D F_CPU=1000000UL -c i2c.cpp
