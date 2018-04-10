# avrlib_i2c
Yet Another I2C bus library for AVR devices.

This version does NOT support multi-master or slave yet.

# C API

### Initialize the bus

	i2c_init();

Use only once if more than one device is present.

### Shutdown the bus

	i2c_shut();

Use only once if more than one device is present.

### Send data to a device

	error = i2c_mtm(address, byte, address_space, stop);

where:
 * address - Device address (8 bit).
 * byte - How many bytes to send
 * address_space - \*ptr to the data to be sent.
 * stop - send the stop bit (0, 1)
 * return error if present.

### Receive data from a device

	error = i2c_mrm(address, byte, address_space, stop);

where:
 * address - Device address (8 bit).
 * byte - How many bytes to send
 * address_space - \*ptr to the data to be sent.
 * stop - send the stop bit (0, 1)
 * return error if present.

### Example compiling C++ code:
avr-gcc -Wall -Wstrict-prototypes -pedantic -std=c11 -mmcu=atmega1284p -O2 -D F_CPU=1000000UL -c i2c.c

# C++ API (std=C++11)

### Add a device to the bus

	I2C my_i2c_device {I2C_ADDRESS};

### Remove the device from the bus

Just delete the I2C object.

### Send data to the device

	my_i2c_device.mtm(byte, address_space, stop);

where:
 * byte - How many bytes to send
 * address_space - \*ptr to the data to be sent.
 * stop - send the stop bit (bool, default=true)

### Receive data from the device

	my_i2c_device.mrm(byte, address_space, stop);

where:
 * byte - bytes to receive
 * address_space - \*ptr to the data to be received.
 * stop - send the stop bit (bool, default=true)

### Example compiling C++ code:
avr-g++ -Wall -pedantic -std=c++11 -mmcu=atmega1284p -O2 -D F_CPU=1000000UL -c i2c.cpp
