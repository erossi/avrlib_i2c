/* Copyright (C) 2017, 2018 Enrico Rossi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <util/twi.h>
#include <avr/io.h>
#include "i2c.h"

/* defines */
#define START 1
#define STOP 2
#define SLA 3
#define DATA 4
#define ACK 5
#define NACK 6

// Static initializations
bool I2C::Initialized {false}; //! I2C bus not yet initialized.
uint8_t I2C::Bus_status {0}; //! Clear the bus status.

/*! Initialize the i2c bus.
 *
 * See the datasheet for SCL speed.
 * Prescaler value (1, 4, 16, 64)
 *
 * SCL freq = CPU FREQ / (16 + 2 * TWBR * Prescaler)
 * (16 + 2 * TWBR * Prescaler) = CPU FREQ / SCL freq
 * 2 * TWBR * Prescaler = (CPU FREQ / SCL freq) - 16
 * TWBR * Prescaler = ((CPU FREQ / SCL freq) - 16)/2
 *
 * SCLf(max) = CPUf/16
 *
 * 16Mhz CLK, 100Khz I2C bus, Prescaler = 4, TWBR = 18
 * 16Mhz CLK, 10Khz I2C bus, Prescaler = 4, TWBR = 198
 * 1Mhz CLK, 10Khz I2C bus, Prescaler = 1, TWBR = 42
 */
void I2C::Init()
{
#if (F_CPU == 1000000UL)
	/* Prescaler 1 */
	TWSR = 0;
	TWBR = 42;
#elif (F_CPU == 16000000UL)
	/* Prescaler 8 */
	TWSR |= _BV(TWPS0);
	TWBR = 99;
#else
#error I2C clock rate unsupported
#endif

	I2C::Initialized = true;
}

/*! Shutdown the i2c bus.
*/
void I2C::Shut()
{
	TWSR = 0;
	TWBR = 0;
	I2C::Initialized = false;
}

/*! Contructor
 *
 * Initialize the bus if not done already.
 *
 * \note C++11 set the const addr to address.
 */
I2C::I2C(uint8_t addr) : address{addr}
{
	if (!I2C::Initialized)
		I2C::Init();
}

/*! Perform an i2c operation.
 *
 * \return the i2c status register properly masked.
 */
void I2C::send(const uint8_t code, const uint8_t data)
{
	switch (code) {
		/* valid also as restart */
		case START:
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
			loop_until_bit_is_set(TWCR, TWINT);
			break;
		case STOP:
			TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
			break;
		case SLA:
		case DATA:
			TWDR = data;
			/* clear interrupt to start transmission */
			TWCR = _BV(TWINT) | _BV(TWEN); 
			loop_until_bit_is_set(TWCR, TWINT);
			break;
		case ACK:
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
			loop_until_bit_is_set(TWCR, TWINT);
			break;
		case NACK:
			TWCR = _BV(TWINT) | _BV(TWEN);
			loop_until_bit_is_set(TWCR, TWINT);
			break;
		default:
			break;
	}

	I2C::Bus_status = TW_STATUS;
}

/*! i2c Master Trasmit.
 *
 * \param lenght the number of byte to send or
 * the max lenght the number of byte to receive.
 *
 * \param *data the pointer to the block of byte.
 * \param stop send the stop at the end of the communication
 * default to TRUE.
 */
void I2C::tx(const uint16_t lenght, uint8_t *data, bool stop)
{
	error_ = 0; // Clear errors
	send(START, 0); // START

	// if start acknoledge
	if ((I2C::Bus_status == TW_START) || (I2C::Bus_status == TW_REP_START))
		send(SLA, (address | TW_WRITE)); // Send WRITE address
	else
		error_ |= (1 << I2C_ERR_BUS); // Something wrong with the bus.

	// if the address is ACK
	if (I2C::Bus_status == TW_MT_SLA_ACK)
		// send data
		for (uint16_t i=0; i<lenght; i++) {
			send(DATA, *(data+i));

			// if data is not ACK
			if (I2C::Bus_status != TW_MT_DATA_ACK) {
				error_ |= (1 << I2C_ERR_DATA);
				i = lenght; // exit
			}
		}
	else
		error_ = (1 << I2C_ERR_NOTFOUND); // Device not responding.

	// if client NACK on ADDR or DATA
	if ((I2C::Bus_status == TW_MT_SLA_NACK) ||
			(I2C::Bus_status == TW_MT_DATA_NACK))
		stop = true; // send the stop

	// if data is ACK
	if (I2C::Bus_status == TW_MT_DATA_ACK)
		I2C::Bus_status = 0; // Everything is ok

	/* send the STOP if required */
	if (stop)
		send(STOP, 0);
}

/*! i2c Master Receive.
 *
 * \param lenght the number of byte to send or
 * the max lenght the number of byte to receive.
 *
 * \param *data the pointer to the block of byte.
 * \param stop send the stop at the end of the communication
 * default to TRUE.
 *
 * \missing complete error handling.
 */
void I2C::rx(const uint16_t lenght, uint8_t *data, bool stop)
{
	// Clear errors
	error_ = 0;

	// START
	send(START, 0);

	// if start acknoledge
	if ((I2C::Bus_status == TW_START) || (I2C::Bus_status == TW_REP_START))
		send(SLA, (address | TW_READ)); // Send READ address
	else
		error_ |= (1 << I2C_ERR_BUS); // Something wrong with the bus.

	// if the address is ACK
	if (I2C::Bus_status == TW_MR_SLA_ACK)
		// Receive data
		for (uint16_t i=0; i<lenght; i++) {
			send(ACK, 0); // send ACK

			// if RX data is ACK
			if (I2C::Bus_status == TW_MR_DATA_ACK) {
				*(data+i) = TWDR; // fetch it
			} else {
				error_ |= (1 << I2C_ERR_DATA);
				i = lenght; // exit
			}
		}
	else
		error_ = (1 << I2C_ERR_NOTFOUND); // Device not responding.

	// Error NACK on ADDR-R or Last DATA
	if ((I2C::Bus_status == TW_MR_SLA_NACK) ||
			(I2C::Bus_status == TW_MR_DATA_NACK))
		stop = true; // send the stop

	if (I2C::Bus_status == TW_MR_DATA_ACK) {
		send(NACK, 0); // last byte, send NACK

		// if data is NACK
		if (I2C::Bus_status == TW_MR_DATA_NACK)
			I2C::Bus_status = 0; // Everything is ok
	}

	/* send the STOP if required */
	if (stop)
		send(STOP, 0);
}
