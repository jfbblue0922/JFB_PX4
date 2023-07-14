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

#include "SCHA63T_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Murata::SCHA63T::Accelerometer
{

SCHA63T_Accelerometer::SCHA63T_Accelerometer(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
		int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SCHA63T(DRV_ACC_DEVTYPE_SCHA63T, "SCHA63T_Accelerometer", bus_option, bus, device, spi_mode, bus_frequency, drdy_gpio),
	_px4_accel(get_device_id(), rotation)
{
	if (drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: DRDY missed");
	}
}

SCHA63T_Accelerometer::~SCHA63T_Accelerometer()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

void SCHA63T_Accelerometer::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void SCHA63T_Accelerometer::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int SCHA63T_Accelerometer::probe()
{
   ScheduleDelayed(50_ms);
	return PX4_OK;
}

void SCHA63T_Accelerometer::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state)
	{
	case STATE::RESET:
		switch (_inits)
		{
		case INITS::INITS_0:
			_reset_timestamp = now;
			_inits = INITS::INITS_1;
			break;
		case INITS::INITS_1:
			if (hrt_elapsed_time(&_reset_timestamp) >= 50_ms) {
				RegisterWrite(Register::SEL_BANK, 0);
				RegisterWrite(Register::RESCTRL, 0);
				_reset_timestamp = now;
				_inits = INITS::INITS_2;
			}
			break;
		case INITS::INITS_2:
			if (hrt_elapsed_time(&_reset_timestamp) >= 30_ms) {
				RegisterWrite(Register::MODE, 0);
				ScheduleDelayed(1_ms);
				RegisterWrite(Register::G_FILT_DYN, 0);
				ScheduleDelayed(1_ms);
				RegisterWrite(Register::A_FILT_DYN, 0);
				_reset_timestamp = now;
				_inits = INITS::INITS_3;
			}
			break;
		case INITS::INITS_3:
			if (hrt_elapsed_time(&_reset_timestamp) >= 500_ms) {
    				RegisterWrite(Register::SET_EOI, 0);
				_reset_timestamp = now;
				_inits = INITS::INITS_4;
			}
			break;
		case INITS::INITS_4:
			if (hrt_elapsed_time(&_reset_timestamp) >= 30_ms) {
    				RegisterRead(Register::S_SUM);
				_reset_timestamp = now;
				_inits = INITS::INITS_5;
			}
			break;
		case INITS::INITS_5:
			if (hrt_elapsed_time(&_reset_timestamp) >= 30_ms) {
    				RegisterRead(Register::S_SUM);
				_reset_timestamp = now;
				_inits = INITS::INITS_6;
			}
			break;
		case INITS::INITS_6:
			if (hrt_elapsed_time(&_reset_timestamp) >= 30_ms) {
    				RegisterRead(Register::S_SUM);
				_inits = INITS::INITS_0;
				_state = STATE::WAIT_FOR_RESET;
			}
			break;

		}

		ScheduleDelayed(50_ms);

		break;

	case STATE::WAIT_FOR_RESET:
		_state = STATE::CONFIGURE;
   		if( RegisterRead(Register::S_SUM) != 0) {
			_state = STATE::RESET;
			ScheduleDelayed(5_s);
		}
		ScheduleDelayed(25_ms);
		break;
	case STATE::CONFIGURE:
		// start reading from FIFO
		_state = STATE::FIFO_READ;
		_data_ready_interrupt_enabled = false;
		
		// ACCL Configure Set
		Configure();
		
		// Interval = 2000usec = 2msec
		ScheduleOnInterval(2000, 0);
		_reset_timestamp = now;
		ScheduleDelayed(25_ms);
		break;

	case STATE::FIFO_READ:
		RunCount += 1;
		ACCELRead(now);

		// periodically update temperature (~1 Hz)
		if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
			UpdateTemperature();
			_temperature_update_timestamp = now;
		}

		ScheduleDelayed(1_ms);   /* This is Very Important For ReCall STATE::FIFO_READ */
		break;
	}
}

void SCHA63T_Accelerometer::ConfigureAccel()
{
	_px4_accel.set_scale(CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB
	_px4_accel.set_range(6.f * CONSTANTS_ONE_G);
}

bool SCHA63T_Accelerometer::Configure()
{
	ConfigureAccel();
	return true;
}

int SCHA63T_Accelerometer::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SCHA63T_Accelerometer *>(arg)->DataReady();
	return 0;
}

void SCHA63T_Accelerometer::DataReady()
{
}

bool SCHA63T_Accelerometer::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool SCHA63T_Accelerometer::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool SCHA63T_Accelerometer::RegisterCheck(const register_config_t &reg_cfg)
{
   return true;
}

bool SCHA63T_Accelerometer::ACCELRead(const hrt_abstime &timestamp_sample)
{
    // const hrt_abstime now = hrt_absolute_time();
   static uint8_t cmd_accl_x[4] = { 0x10, 0x00, 0x00, 0xE9 }; // ACCL_X
   static uint8_t cmd_accl_y[4] = { 0x14, 0x00, 0x00, 0xEF }; // ACCL_Y
   static uint8_t cmd_accl_z[4] = { 0x18, 0x00, 0x00, 0xE5 }; // ACCL_Z
   static uint8_t cmd_rate_x[4] = { 0x04, 0x00, 0x00, 0xF7 }; // RATE_X
   static uint8_t cmd_temper[4] = { 0x1C, 0x00, 0x00, 0xE3 }; // TEMPER

   uint8_t rsp_accl_x[4];
   uint8_t rsp_accl_y[4];
   uint8_t rsp_accl_z[4];
   uint8_t rsp_rate_x[4];
   uint8_t rsp_temper[4];

   int16_t accel_x;
   int16_t accel_y;
   int16_t accel_z;

	// #### ACCL_X Cmd Send (This Response rsp_accl_x is Dust!!)
	transfer( cmd_accl_x, rsp_accl_x, 4);

	// #### ACCL_Y Cmd Send + ACCL_X Response Receive
	transfer( cmd_accl_y, rsp_accl_x, 4);

	// #### ACCL_Z Cmd Send + ACCL_Y Response Receive
	transfer( cmd_accl_z, rsp_accl_y, 4);

	// ##### RATE_X Cmd Send + ACCL_Z Response Receive
	transfer( cmd_rate_x, rsp_accl_z, 4);

	// ##### TEMPER Cmd Send + RATE_X Response Receive
	transfer( cmd_temper, rsp_rate_x, 4);

	// ##### TEMPER Cmd Send + TEMPRE Response Receive
	transfer( cmd_temper, rsp_temper, 4);

	accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);
	// accel_x = accel_x + ((32767 * 65) / (980 * 6));
	accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);
	// accel_y = accel_y + ((32767 * 135) / (980 * 6));
	accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);
	accl_temper  = combine(rsp_temper[1], rsp_temper[2]);

	// gyro_x save
	gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);

	// sensor's frame is +x forward, +y left, +z up
	// flip y & z to publish right handed with z down (x forward, y right, z down)
	accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	uint8_t ret = RegisterRead(Register::S_SUM);
	if( ret != 0) {	/* crc ng data load */
		accel_x = accel_x_sv;
		accel_y = accel_y_sv;
		accel_z = accel_z_sv;
	} else {			/* crc ok data save */
		accel_x_sv = accel_x;
		accel_y_sv = accel_y;
		accel_z_sv = accel_z;
	}

	if(RunCount > 2000 ) {
		RunCount = 0;
	}

	// PX4 Controller ACCL-DATA update
	_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);

	if( ret != 0) return false;
	return true;
}

void SCHA63T_Accelerometer::UpdateTemperature()
{
	float temperature;
	temperature = 25.0f + ( accl_temper / 30 );
	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);

	} else {
		perf_count(_bad_transfer_perf);
	}
}

uint8_t SCHA63T_Accelerometer::RegisterRead(Register reg)
{
    uint8_t *cmd;
    uint8_t acc0E[4] = { 0x38, 0x00, 0x00, 0xD5 };
    uint8_t acc10[4] = { 0x40, 0x00, 0x00, 0x91 };
    uint8_t acc12[4] = { 0x48, 0x00, 0x00, 0x9D };
    uint8_t acc14[4] = { 0x50, 0x00, 0x00, 0x89 };
    uint8_t acc15[4] = { 0x54, 0x00, 0x00, 0x8F };
    uint8_t rrsp[4];
    cmd = acc15;
 
    switch( reg )
	{
	case Register::S_SUM:  /* 0x0E */
		cmd = acc0E;
		break;
	case Register::R_S1:   /* 0x10 */
		cmd = acc10;
		break;
	case Register::A_S1:   /* 0x12 */
		cmd = acc12;
		break;
	case Register::C_S1:   /* 0x14 */
		cmd = acc14;
		break;
	case Register::C_S2:   /* 0x15 */
		cmd = acc15;
		break;
	default:
		break;
	}

	transfer(cmd, rrsp, 4);

	if( reg == Register::S_SUM ) {
		unsigned char	bCrc = CalcTblCrc( rrsp, 3);
		if( bCrc != rrsp[3] ) {
			return 1;
		}
	}
	return 0;
}

void SCHA63T_Accelerometer::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t *cmd = nullptr;

//----------< SELECT 13,20,46,200,300Hz >----------
//	uint8_t acc16_13Hz[4] = { 0xD8, 0x00, 0x00, 0x45 };
//	uint8_t acc16_20Hz[4] = { 0xD8, 0x09, 0x09, 0xA6 };
//	uint8_t acc16_46Hz[4] = { 0xD8, 0x12, 0x12, 0x9E };
//	uint8_t acc16_200Hz[4] = { 0xD8, 0x1B, 0x1B, 0x7D };
	uint8_t acc16_300Hz[4] = { 0xD8, 0x24, 0x24, 0xEE };

	uint8_t acc18[4] = { 0xE0, 0x00, 0x01, 0x7C };
	uint8_t acc19[4] = { 0xE4, 0x00, 0x00, 0x67 };

//----------< SELECT 13,20,46,200,300Hz >----------
//	uint8_t acc1A_13Hz[4] = { 0xE8, 0x00, 0x00, 0x6D };
//	uint8_t acc1A_20Hz[4] = { 0xE8, 0x01, 0x11, 0xF1 };
//	uint8_t acc1A_46Hz[4] = { 0xE8, 0x02, 0x22, 0x48 };
//	uint8_t acc1A_200Hz[4] = { 0xE8, 0x03, 0x33, 0xD4 };
	uint8_t acc1A_300Hz[4] = { 0xE8, 0x04, 0x44, 0x27 };

	uint8_t acc1F[4] = { 0xFC, 0x00, 0x00, 0x73 };
	uint8_t acc20[4] = { 0xE0, 0x00, 0x02, 0x5B };

	uint8_t rsp[6];

	switch( reg )
	{
	case Register::G_FILT_DYN: /* 0x16 */
		cmd = acc16_300Hz;
		break;
	case Register::RESCTRL: 	/* 0x18 */
		cmd = acc18;
		break;
	case Register::MODE: 		/* 0x19 */
		cmd = acc19;
		break;
	case Register::A_FILT_DYN: /* 0x1A */
		cmd = acc1A_300Hz;
		break;
	case Register::SEL_BANK: 	/* 0x1F */
		cmd = acc1F;
		break;
	case Register::SET_EOI: 	/* 0x20 */
		cmd = acc20;
		break;
	default:
		break;
	}

	transfer(cmd, rsp, 4);
}

void SCHA63T_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
} // namespace Murata::SCHA63T::Accelerometer
