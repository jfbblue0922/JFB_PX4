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
 * @file ASM330LHH.hpp
 *
 * Driver for the ST ASM330LHH connected via SPI.
 *
 */

#pragma once

#include "ST_ASM330LHH_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_ASM330LHH;

class ASM330LHH : public device::SPI, public I2CSPIDriver<ASM330LHH>
{
public:
	ASM330LHH(const I2CSPIDriverConfig &config);
	~ASM330LHH() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	static constexpr uint32_t LA_ODR = 3333;    // Linear acceleration output data rate
	static constexpr uint32_t G_ODR  = 3333;    // Angular rate output data rate

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / ASM330LHH::G_ODR};
	static constexpr float GYRO_RATE{ASM330LHH::G_ODR};   // 3333 Hz gyro
	static constexpr float ACCEL_RATE{ASM330LHH::LA_ODR}; // 3333 Hz accel

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(math::min(FIFO::SIZE / 12, sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadStatus();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void UpdateTemperature();

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{14};
	register_config_t _register_cfg[size_register_cfg] {
		// Register               | Set bits, Clear bits
		{ Register::CTRL3_C,      CTRL3_C_BIT::BDU_EN | CTRL3_C_BIT::IF_INC_EN, CTRL3_C_BIT::SW_RESET_EN },
		{ Register::CTRL4_C,      CTRL4_C_BIT::I2C_DISABLE_EN, 0x00 },
		{ Register::CTRL5_C,      0x00, 0x00 },
		{ Register::CTRL6_C,      0x00, 0x00 },
		{ Register::CTRL10_C,     0x00, 0x00 },
		{ Register::FIFO_CTRL1,   0x00, 0x00 },
		{ Register::FIFO_CTRL2,   0x00, 0x00 },
		{ Register::FIFO_CTRL3,   FIFO_CTRL3_BIT::BRD_GY_3333HZ | FIFO_CTRL3_BIT::BRD_XL_3333HZ, 0x00 },
		{ Register::FIFO_CTRL4,   FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS_MODE, 0x00 },
		{ Register::CTRL2_G,      CTRL2_G_BIT::ODR_G_3333HZ | CTRL2_G_BIT::FS_G_2000DPS, 0x00 },
		{ Register::CTRL7_G,      0x00, 0x00 },
		{ Register::CTRL1_XL,     CTRL1_XL_BIT::ODR_XL_3333HZ | CTRL1_XL_BIT::FS_XL_16G, 0x00 },
		{ Register::CTRL8_XL,     0x00, 0x00 },
		{ Register::CTRL9_XL,     CTRL9_XL_BIT::DEN_X_STORED | CTRL9_XL_BIT::DEN_Y_STORED | CTRL9_XL_BIT::DEN_Z_STORED, 0x00 },
	};
};
