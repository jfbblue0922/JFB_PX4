/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ST_ASM330_registers.hpp
 *
 * ST ASM330 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_ASM330
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHO_AM_I_ID = 0x6B;    // Who I am ID

enum class Register : uint8_t {
	FIFO_CTRL1          = 0x07,
	FIFO_CTRL2          = 0x08,
	FIFO_CTRL3          = 0x09,
	FIFO_CTRL4          = 0x0A,

	WHO_AM_I            = 0x0F,

	CTRL1_XL            = 0x10, // Control register 1 (Linear acceleration sensor).
	CTRL2_G             = 0x11, // Control register 2 (Angular rate sensor).
	CTRL3_C             = 0x12, // Control register 3 (Common).
	CTRL4_C             = 0x13, // Control register 4 (Common).
	CTRL5_C             = 0x14, // Control register 5 (Common).
	CTRL6_C             = 0x15, // Control register 6 (Common).
	CTRL7_G             = 0x16, // Control register 7 (Angular rate sensor).
	CTRL8_XL            = 0x17, // Control register 8 (Linear acceleration sensor).
	CTRL9_XL            = 0x18, // Control register 9 (Linear acceleration sensor).
	CTRL10_C            = 0x19, // Control register 10 (Common).

	OUT_TEMP_L          = 0x20, // Temperature data output register(L)
	OUT_TEMP_H          = 0x21, // Temperature data output register(H)

	FIFO_STATUS1        = 0x3A, // FIFO status register 1.
	FIFO_STATUS2        = 0x3B, // FIFO status register 2.

	FIFO_DATA_OUT_TAG   = 0x78, // FIFO tag register.
	FIFO_DATA_OUT_X_L   = 0x79, // FIFO data output X(L).
	FIFO_DATA_OUT_X_H   = 0x7A, // FIFO data output X(H).
	FIFO_DATA_OUT_Y_L   = 0x7B, // FIFO data output Y(L).
	FIFO_DATA_OUT_Y_H   = 0x7C, // FIFO data output Y(H).
	FIFO_DATA_OUT_Z_L   = 0x7D, // FIFO data output Z(L).
	FIFO_DATA_OUT_Z_H   = 0x7E, // FIFO data output Z(H).
};

// FIFO_CTRL3
enum FIFO_CTRL3_BIT : uint8_t {
	// BDR_GY[3:0]
	BRD_GY_3333HZ   = Bit7 | Bit4,  // Batch Data Rate Gyroscope Data : 3333Hz
	// BDR_GY[3:0]
	BRD_XL_3333HZ   = Bit3 | Bit0,  // Batch Data Rate Accelerometer Data : 3333Hz
};

// FIFO_CTRL4
enum FIFO_CTRL4_BIT : uint8_t {
	// FIFO_MODE[2:0]
	FIFO_MODE_CONTINUOUS_MODE   = Bit2 | Bit1,  // FIFO Mode : Continuous Mode
};

// CTRL1_XL
enum CTRL1_XL_BIT : uint8_t {
	// ODR_G[3:0]
	ODR_XL_3333HZ   = Bit7 | Bit4,  // Accelerometer ODR : 3333Hz
	// FS[3:0]
	FS_XL_16G       = Bit2,  // Accelerometer Full-Scale : 16g
};

// CTRL2_G
enum CTRL2_G_BIT : uint8_t {
	// ODR_G[3:0]
	ODR_G_3333HZ    = Bit7 | Bit4,  // Gyroscope ODR : 3333Hz
	// FS_G[3:0]
	FS_G_2000DPS    = Bit3 | Bit2,  // Gyroscope Chain Full-Scale : 2000dps
};

// CTRL3_C
enum CTRL3_C_BIT : uint8_t {
	// BDU
	BDU_EN          = Bit6, // Block Data Update Enable
	// IF_INC
	IF_INC_EN       = Bit2, // Register Address Automatically Incremented Enable
	// SW_RESET
	SW_RESET_EN     = Bit0, // Software Reset Enbale
};

// CTRL4_C
enum CTRL4_C_BIT : uint8_t {
	// I2C_disable
	I2C_DISABLE_EN  = Bit2, // I2C Interfaces Disbled
};

// CTRL9_XL
enum CTRL9_XL_BIT : uint8_t {
	// DEN_X
	DEN_X_STORED    = Bit7, // DEN stored in X-axis LSB
	// DEN_Y
	DEN_Y_STORED    = Bit6, // DEN stored in Y-axis LSB
	// DEN_Z
	DEN_Z_STORED    = Bit5, // DEN stored in Z-axis LSB
};

// FIFO_STATUS2
enum FIFO_STATUS2_BIT : uint8_t {
	FIFO_OVR_IA     = Bit6, // FIFO overrun status.
};

namespace FIFO
{
static constexpr size_t SIZE = 256 * 12; // 256 samples max
}

} // namespace ST_LSM9DS1
