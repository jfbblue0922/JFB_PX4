/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

namespace Murata::SCHA63T::Accelerometer
{


// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface

enum class Register : uint8_t {
 S_SUM  = 0x0E,      // INIT
 R_S1   = 0x10,
 A_S1   = 0x12,
 C_S1   = 0x14,
 C_S2   = 0x15,
 G_FILT_DYN = 0x16,  // INIT
 RESCTRL = 0x18,     // INIT
 MODE    = 0x19,     // INIT
 A_FILT_DYN = 0x1A,  // INIT
 SEL_BANK   = 0x1F,  // INIT
 SET_EOI = 0x20,     // INIT-END
};

// INT_STATUS_1
enum INT_STATUS_1_BIT : uint8_t {
	data_int      = Bit7,
	fifo_wm_int   = Bit6,
	fifo_full_int = Bit5,
};

// FIFO_STATUS
enum FIFO_STATUS_BIT : uint8_t {
	fifo_overrun       = Bit7,
	fifo_frame_counter = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // fifo_frame_counter<6:0>
};

// ACCD_HBW
enum ACCD_HBW_BIT : uint8_t {
	data_high_bw = Bit7, // 1 -> unfiltered
};

// PMU_RANGE
enum PMU_RANGE_BIT : uint8_t {
	// range<3:0>
	range_2g  = Bit1 | Bit0, //  ́E011b ́E-> ±2g range
	range_4g  = Bit2 | Bit0, //  ́E101b ́E-> ±4g range
	range_8g  = Bit3,        //  ́E000b ́E-> ±8g range
	range_16g = Bit3 | Bit2, //  ́E100b ́E-> ±16g range
};

// INT_EN_1
enum INT_EN_1_BIT : uint8_t {
	int_fwm_en   = Bit6,
	int_ffull_en = Bit5,
	data_en      = Bit4,
};

// INT_MAP_1
enum INT_MAP_1_BIT : uint8_t {
	int2_data  = Bit7,
	int2_fwm   = Bit6,
	int2_ffull = Bit5,

	int1_ffull = Bit2,
	int1_fwm   = Bit1,
	int1_data  = Bit0,
};

// INT_OUT_CTRL
enum INT_OUT_CTRL_BIT : uint8_t {
	int1_od  = Bit1,
	int1_lvl = Bit0,
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	fifo_mode = Bit6,
};

namespace FIFO
{
struct DATA {
	uint8_t ACCD_X_LSB;
	uint8_t ACCD_X_MSB;
	uint8_t ACCD_Y_LSB;
	uint8_t ACCD_Y_MSB;
	uint8_t ACCD_Z_LSB;
	uint8_t ACCD_Z_MSB;
};
static_assert(sizeof(DATA) == 6);

static constexpr size_t SIZE = sizeof(DATA) * 32; // up to 32 frames of accelerometer data

} // namespace FIFO
} // namespace Murata::SCHA63T::Accelerometer
