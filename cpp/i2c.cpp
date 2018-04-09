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

// local defines
#define START 1
#define STOP 2
#define SLA 3
#define DATA 4
#define ACK 5
#define NACK 6

// Static initializations
uint8_t I2C::Registered {0}; //! I2C bus not yet initialized.
uint8_t I2C::Bus_status {0}; //! Clear the bus status.

/*! Initialize the i2c bus.
 *
 * Setup the bus speed, depends on the hardware
 * implementation, mcu and clock speed.
 *
 * For info on theese setup check the datasheet.
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
	// Fscl = F_CPU/(16 + 2*TWBR*(4^Prescaler))
	// 2.840909091 Khz
	TWSR = 0; // Prescaler 1
	TWBR = 42;
#elif (F_CPU == 8000000UL)
	// .9746588693 Khz
	TWSR |= _BV(TWPS0); // Prescaler 4
	TWBR = 16;
#elif (F_CPU == 16000000UL)
	// 0.3155569580 Khz
	TWSR |= _BV(TWPS0); // Prescaler 4
	TWBR = 99;
#else
#error I2C clock rate unsupported
#endif
}

/*! Shutdown the i2c bus.
*/
void I2C::Shut()
{
	TWSR = 0;
	TWBR = 0;
}

/*! Add a device to the bus.
 *
 * Check if the bus has not been initialized yet
 * and do it.
 *
 * \note C++11 set the const addr to address.
 */
I2C::I2C(uint8_t addr) : address{addr}
{
	if (!I2C::Registered)
		I2C::Init();

	I2C::Registered++; // Add the device
}

/*! Remove a device from the bus.
 *
 * If the device is the last registered on the bus, shut
 * down the I2C bus.
 */
I2C::~I2C()
{
	// Remove the device
	I2C::Registered--;

	// shut down the bus if not in use.
	if (!I2C::Registered)
		I2C::Shut();
}

/*! Perform an i2c operation.
 *
 * Send on the I2C bus the required operation like Start,
 * Stop etc. Set the I2C bus error status.
 *
 * \param code I2C operation do be performed.
 * \param data data to be sent.
 */
void I2C::send(const uint8_t code, const uint8_t data)
{
	switch (code) {
		/* valid also as RESTART */
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

/*! i2c Master Trasmitter Mode.
 *
 * Send the data to I2C client, following the diagram in
 * the datasheet.
 * To send a write command (ex. to load a register), then a read
 * to get the result use:
 *
 * i2c_mtm(a, 1, d, FALSE);
 * i2c_mrm(a, 1, d, TRUE);
 *
 * on the MRM the start will be considered a REP-START and should
 * generate a TW_REP_START as a result.
 *
 * Case not managed:
 *   - multi-master
 *   - TW_MT_SLA_NACK with sending data anyway.
 *
 * Successful result is considered only those which results with
 * SLA_ACK or DATA_ACK.
 *
 * \note: Re-start is equal to start, but with a different
 * TW_results.
 *
 * \warning in case of NACK, if stop was not forseen,
 * the NO stop is generated.
 *
 * \param addr the i2c slave address.
 * \param lenght the number of byte to send.
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

/*! i2c Master Receiver Mode.
 *
 * \see i2c_mtm
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
