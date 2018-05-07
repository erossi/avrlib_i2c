/* Copyright (C) 2011-2018 Enrico Rossi
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
void i2c_init(void)
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
void i2c_shut(void)
{
	TWSR = 0;
	TWBR = 0;
}

/*! Perform an i2c operation.
 *
 * Send on the I2C bus the required operation like Start,
 * Stop etc. Set the I2C bus error status.
 *
 * \param code I2C operation do be performed.
 * \param data data to be sent.
 */
void send(const uint8_t code, const uint8_t data)
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

	i2c_Bus_status = TW_STATUS;
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
 * \param address the i2c slave address.
 * \param lenght the number of byte to send.
 * \param *data the pointer to the block of byte.
 * \param stop the stop at the end of the communication.
 * \return TRUE: error, FALSE: ok.
 */
uint8_t i2c_mXm(const uint8_t address, const uint16_t lenght,
		uint8_t *data, uint8_t stop)
{
	uint16_t i;
	uint8_t run; // boolean

	i = 0;
	run = TRUE;

	/* Send the start.
	 * Results:
	 * TW_START
	 * TW_REP_START in case this was a restart.
	 */
	send(START, 0);

	do {
		switch (i2c_Bus_status) {
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
				send(SLA, address);
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
					run = FALSE;
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
						send(ACK, 0);
					else
						/* status should become
						 * TW_MR_DATA_NACK.
						 * Instead it becomes
						 * TW_NO_INFO !!!
						 */
						send(NACK, 0);

					// Read the byte
					*(data+i) = TWDR;
					i++;
				} else {
					/* this should not be reached
					 * if the last NACK set the
					 * status to TW_MR_DATA_NACK
					 * TW_NO_INFO.
					 */
					run = FALSE;
				}

				break;

			case TW_MR_DATA_NACK:
			case TW_NO_INFO:
				/* exit */
				run = FALSE;
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
				stop = TRUE;
				run = FALSE;
		}
	} while (run);

	/* send the STOP if required */
	if (stop)
		send(STOP, 0);

	/* if everything is ok, assuming the only two
	 * conditions are the ACK results.
	 */
	if ((i2c_Bus_status == TW_MT_SLA_ACK) ||
			(i2c_Bus_status == TW_MT_DATA_ACK) ||
			(i2c_Bus_status == TW_MR_DATA_NACK) ||
			(i2c_Bus_status == TW_NO_INFO))
		return(FALSE); // ok
	else
		return(TRUE);
}

/*! i2c Master Transmitter Mode.
 *
 * \see i2c_mXm
 *
 * \param address the i2c slave address.
 * \param the max lenght the number of byte to receive.
 * \param *data the pointer to the block of byte.
 * \param stop the stop at the end of the communication.
 * \return 1 error, 0 ok
 */
uint8_t i2c_mtm(const uint8_t address, const uint16_t lenght,
		uint8_t *data, uint8_t stop)
{
	return(i2c_mXm(address, lenght, data, stop));
}

/*! i2c Master Receiver Mode.
 *
 * \see i2c_mXm
 *
 * \param address the i2c slave address.
 * \param the max lenght the number of byte to receive.
 * \param *data the pointer to the block of byte.
 * \param stop the stop at the end of the communication.
 * \return 1 error, 0 ok
 */
uint8_t i2c_mrm(const uint8_t address, const uint16_t lenght,
		uint8_t *data, uint8_t stop)
{
	return(i2c_mXm((address | TW_READ), lenght, data, stop));
}

#ifdef I2C_LEGACY_MODE
/*! Legacy master Send a byte.
 *
 * \param addr address of the slave.
 * \param data byte to send.
 * \param stop if true send the stop condition at the end.
 * \return 0 = OK or error.
 * \note Atmel does not show the skip of stop in transitions.
 */
uint8_t i2c_master_send_b(const uint8_t addr, const uint8_t data,
		uint8_t stop)
{
	return(i2c_mtm(addr, 1, (uint8_t *) &data, stop));
}

/*! legacy i2c master send a word.
 * Send a word to the i2c slave with stop at the end.
 * \param addr address of the slave.
 * \param msb byte to send first.
 * \param lsb byte to send last.
 * \return 0 - OK, 1 - Error
 */
uint8_t i2c_master_send_w(const uint8_t addr, const uint8_t msb,
		const uint8_t lsb)
{
	uint16_t data;

	data = (msb << 8) | lsb;

	return(i2c_mtm(addr, 2, (uint8_t *) &data, TRUE));
}

/*! Legacy Master Read a byte.
 *
 * \param addr address of the slave.
 * \param data pointer to the byte to receive.
 * \param stop if true send the stop condition at the end.
 * \return 0 = OK or error.
 * \note Atmel does not show the skip of stop in transitions.
 */
uint8_t i2c_master_read_b(const uint8_t addr, uint8_t *data,
		uint8_t stop)
{
	return(i2c_mrm(addr, 1, data, stop));
}

/*! Read a word (2 byte) from the slave.
 * \param addr address of the slave.
 * \param data pre-allocated word.
 * \return 1 - value OK, 0 - Error.
 */
uint8_t i2c_master_read_w(const uint8_t addr, uint16_t *data)
{
	uint8_t err;
	uint8_t swap;

	err = i2c_mrm(addr, 2, (uint8_t *) data, TRUE);
	/* swap lsb <-> msb */
	swap = *data & 0xff;
	*data = (*data << 8) | swap;

	return(err);
}
#endif
