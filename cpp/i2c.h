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

/*! \file i2c.h
 *
 */

#ifndef _I2C_DEF
#define _I2C_DEF

// uint* defs
#include <stdint.h>

// common defs
#define I2C_GC_RESET 0
#define I2C_TIMEOUT 0xff

class I2C {
	private:
		// I2C bus should be initialized only once
		static uint8_t Registered; //! \brief Devices registered on the bus
		static uint8_t Bus_status; //! \biref Global Bus error common to all devices.
		bool error_; //! Local error to a single device.
		const uint8_t address; // device's address
		static void send(const uint8_t, const uint8_t = 0);
		static void Init(); // Initialize bus
		static void Shut(); // De-initialize bus
	public:
		I2C(uint8_t); //! \brief Register a device with i2c address.
		~I2C(); //! \brief de-register a device.
		static uint8_t BusError() { return(Bus_status); }; //! \deprecated
		static uint8_t BusStatus() { return(Bus_status); }; //! Global I2C bus
		uint8_t error() { return(error_); };
		void mtm(const uint16_t, uint8_t*, bool = true);
		void mrm(const uint16_t, uint8_t*, bool = true);
};

#endif
