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
#include <util/delay.h>
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
 * \param addr the address of the device
 * \param timeout msec non-block timeout, default 0 = block
 * until the operation is completed.
 */
I2C::I2C(uint8_t addr, uint16_t t) : address{addr}, timeout{t}
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
	uint16_t i { timeout };

	switch (code) {
		/* valid also as RESTART */
		case START:
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

			if (i)
				while (i-- && bit_is_clear(TWCR, TWINT))
					_delay_ms(1);
			else
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

			if (i)
				while (i-- && bit_is_clear(TWCR, TWINT))
					_delay_ms(1);
			else
				loop_until_bit_is_set(TWCR, TWINT);

			break;
		case ACK:
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);

			if (i)
				while (i-- && bit_is_clear(TWCR, TWINT))
					_delay_ms(1);
			else
				loop_until_bit_is_set(TWCR, TWINT);

			break;
		case NACK:
			TWCR = _BV(TWINT) | _BV(TWEN);

			if (i)
				while (i-- && bit_is_clear(TWCR, TWINT))
					_delay_ms(1);
			else
				loop_until_bit_is_set(TWCR, TWINT);

			break;
		default:
			break;
	}

	I2C::Bus_status = TW_STATUS;
}

/*! i2c Master Mode.
 *
 * Send the data to I2C client, following the diagram in
 * the datasheet for both the Trasmitter and Receiver mode.
 *
 * To send a write command (ex. to load a register), then a read
 * to get the result use:
 *
 * i2c_mtm(a, 1, d, false);
 * i2c_mrm(a, 1, d, true);
 *
 * on the MRM the start will be considered a REP-START and should
 * generate a TW_REP_START as a result.
 *
 * Case not managed:
 *   - multi-master
 *   - TW_MT_SLA_NACK with sending data anyway.
 *
 * Successful result is considered:
 * SLA_ACK or DATA_ACK.
 *
 * \note: Re-start is equal to start, but with a different
 * TW_results.
 *
 * \warning in case of NACK, if stop was not forseen,
 * the NO stop is generated.
 *
 * \param lenght the number of byte to send.
 * \param *data the pointer to the block of byte.
 * \param stop send the stop at the end of the communication.
 * \param tw_read I2C read/write, true = read.
 */
void I2C::mXm(const uint16_t lenght, uint8_t *data, bool stop, bool tw_read)
{
	uint16_t i {0};
	bool run {true};

	/* Send the start.
	 * Results:
	 * TW_START
	 * TW_REP_START in case this was a restart.
	 */
	send(START); // START

	do {
		switch (I2C::Bus_status) {
			case TW_START:
			case TW_REP_START:
				/* Send address + W.
				 * Results:
				 * TW_MT_SLA_ACK,
				 * TW_MT_SLA_NACK,
				 * TW_MR_ARB_LOST,
				 * TW_SR_ARB_LOST_SLA_ACK,
				 * TW_SR_ARB_LOST_GCALL_ACK,
				 * TW_ST_ARB_LOST_SLA_ACK
				 *
				 * Send address + R.
				 * Results:
				 * TW_MR_SLA_ACK
				 * TW_MR_SLA_NACK
				 * TW_MT_ARB_LOST
				 * TW_SR_ARB_LOST_SLA_ACK
				 * TW_SR_ARB_LOST_GCALL_ACK
				 * TW_ST_ARB_LOST_SLA_ACK
				 */
				send(SLA, (address | tw_read));
				break;

				/* note that the implementation
				 * of the TW_MT_SLA_ACK is the
				 * same of TW_MT_DATA_ACK.
				 * Removed duplication of code.
				 */
			case TW_MT_SLA_ACK:
			case TW_MT_DATA_ACK:
				/* send data if present (should be).
				 * Results:
				 * TW_MT_DATA_ACK
				 * TW_MT_DATA_NACK
				 */
				if (i < lenght) {
					send(DATA, *(data+i));
					i++;
				} else {
					run = false;
				}

				break;

			case TW_MR_SLA_ACK:
			case TW_MR_DATA_ACK:
				/* Receive data.
				 *
				 * data reception is triggered by sending
				 * an ACK or NACK, depending if this is the
				 * last byte to be received or not.
				 *
				 * Results generated by us:
				 * TW_MR_DATA_ACK
				 * TW_MR_DATA_NACK
				 */
				if (i < lenght) {
					if (i < (lenght - 1))
						/* status should remain
						 * TW_MR_DATA_ACK
						 */
						send(ACK);
					else
						/* status should become
						 * TW_MR_DATA_NACK.
						 * Instead it becomes
						 * TW_NO_INFO !!!
						 */
						send(NACK);

					// Read the byte
					*(data+i) = TWDR;
					i++;
				} else {
					/* this should not be reached
					 * if the last NACK set the
					 * status to TW_MR_DATA_NACK
					 * TW_NO_INFO.
					 */
					run = false;
				}

				break;

			case TW_MR_DATA_NACK:
			case TW_NO_INFO:
				/* exit */
				run = false;
				break;

			case TW_MT_SLA_NACK:
			case TW_MT_DATA_NACK:
				/* exit.
				 * by forcing a STOP, a RE-START
				 * from error conditions cannot be
				 * performed. If a device generate
				 * a NACK to indicate wait and it
				 * need a restart sequence this part of
				 * the code need to be changed.
				 */
				/* code de-duplication */

			case TW_MR_SLA_NACK:
				/* force exit without managing
				 * a re-start condition.
				 * see MT_SLA_NACK.
				 */
				/* code de-duplication */

			default:
				/* unmanaged situation,
				 * send STOP and exit:
				 * Apply to:
				 * TW_BUS_ERROR (0x00)
				 */
				stop = true;
				run = false;
		}
	} while (run);

	/* send the STOP if required */
	if (stop)
		send(STOP);

	/* if everything is ok, assuming the only two
	 * conditions are the ACK results.
	 */
	if ((I2C::Bus_status == TW_MT_SLA_ACK) ||
			(I2C::Bus_status == TW_MT_DATA_ACK) ||
			(I2C::Bus_status == TW_MR_DATA_NACK) ||
			(I2C::Bus_status == TW_NO_INFO))
		error_ = false;
	else
		error_ = true;
}

/*! i2c Master Transmitter Mode.
 *
 * \see I2C::mXm
 *
 * \param lenght the max lenght the number of byte to receive.
 * \param *data the pointer to the block of byte.
 * \param stop the stop at the end of the communication.
 */
void I2C::mtm(const uint16_t lenght, uint8_t *data, bool stop)
{
	mXm(lenght, data, stop, false);
}

/*! i2c Master Receiver Mode.
 *
 * \see I2C::mXm
 *
 * \param the max lenght the number of byte to receive.
 * \param *data the pointer to the block of byte.
 * \param stop the stop at the end of the communication.
 */
void I2C::mrm(const uint16_t lenght, uint8_t *data, bool stop)
{
	mXm(lenght, data, stop, true);
}
