/****************************************************************************
 *
 *   Copyright (c) 2021-2025 PX4 Development Team. All rights reserved.
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sen0473.cpp
 *
 * I2C driver for DFRobot Gravity SEN0473 hydrogen sensor.
 *
 * Author:  Nick Truttmann <ntruttmann@ethz.ch>
 */

 #include "sen0473.h"

 using namespace time_literals;


 uint8_t gravity_checksum(const uint8_t *buf)
{
    uint16_t sum = 0;
    for (int i = 1; i < 7; ++i) sum += buf[i];
    return uint8_t(0x100 - (sum & 0xFF));     // identical result
}


 SEN0473::SEN0473(const I2CSPIDriverConfig &config)
     : I2C(config), ModuleParams(nullptr), I2CSPIDriver(config)
 {
     measurement_time   = hrt_absolute_time();
     measurement_index  = 0;
     measured_hydrogen  = 0;
     measured_temperature = 0.f;
 }


 bool SEN0473::write_frame(const uint8_t frame[9])
 {
     uint8_t buf[10];
     buf[0] = 0x00;                  // register pointer
     memcpy(&buf[1], frame, 9);      // full Gravity frame
     return transfer(buf, 10, nullptr, 0) == PX4_OK;
 }

 bool SEN0473::read_frame(uint8_t frame[9])
 {
     uint8_t reg = 0x00;             // reset internal pointer
     if (transfer(&reg, 1, frame, 9) != PX4_OK) {
	 return false;
     }
     return (frame[0] == 0xFF &&
	frame[8] == gravity_checksum(frame));
 }



 bool SEN0473::query_all(float &ppm, float &tempC)
 {
     /* —— 1 / Send passive “get-all” request (command 0x88) —— */
     uint8_t tx[9] = {0xFF, 0x01, CMD_GET_ALL, 0, 0, 0, 0, 0, 0};
     tx[8] = gravity_checksum(tx);

     if (!write_frame(tx)) {
	 return false;
     }

     px4_usleep(10_ms);

     /* —— 2 / Receive 9-byte reply —— */
     uint8_t rx[9] {};
     if (!read_frame(rx)) {
	 return false;
     }

     /* —— 3 / Parse raw values —— */
     const uint16_t raw_ppm  = uint16_t(rx[2] << 8) | rx[3];
     const uint8_t  decimals = rx[5];
     const uint16_t raw_temp = uint16_t(rx[6] << 8) | rx[7];

     static const float kDecLut[3] = {1.f, 0.1f, 0.01f};
     float con_ppm = raw_ppm * kDecLut[decimals <= 2 ? decimals : 0];      // ppm before T-comp

     /* —— 4 / Convert NTC reading to °C (DFRobot formula) —— */
     float vpd3 = 3.f * raw_temp / 1024.f;
     float rth  = vpd3 * 10000.f / (3.f - vpd3);
     tempC      = 1.f / (1.f / (273.15f + 25.f) + 1.f / 3380.13f * logf(rth / 10000.f)) - 273.15f;   // °C

     /* —— 5 / Apply hydrogen temperature-compensation curve —— */
     if (tempC > -20.f && tempC <= 20.f) {
	 con_ppm = (con_ppm / (0.74f  * tempC + 0.007f)) - 5.f;
     } else if (tempC > 20.f && tempC <= 40.f) {
	 con_ppm = (con_ppm / (0.025f * tempC + 0.30f)) - 5.f;
     } else if (tempC > 40.f && tempC <= 60.f) {
	 con_ppm = (con_ppm / (0.001f * tempC + 0.90f)) - (0.75f * tempC - 25.f);
     } else {
     con_ppm = NAN;                       // outside characterized range
     PX4_WARN("Temperature %.1f °C out of range", (double)tempC);
     }

     if (con_ppm < 0) { // clamp negatives
	con_ppm = NAN;
	PX4_WARN("Temperature %.1f °C out of range", (double)tempC);

}

     /* —— 6 / Return compensated reading —— */
     ppm = con_ppm;
     return true;
 }

 bool SEN0473::set_passive_mode()
 {
     uint8_t tx[9] = {0xFF, 0x01, CMD_SET_ACQ_MODE, ACQUIRE_MODE_PASSIVE, 0, 0, 0, 0, 0};
     tx[8] = gravity_checksum(tx);
     return write_frame(tx);
 }

 int SEN0473::init()
 {
     if (I2C::init() != PX4_OK) {
	 return PX4_ERROR;
     }

     _sensor_hydrogen_pub.advertise();
     ScheduleOnInterval(1_s);   // 1 Hz
     return PX4_OK;
 }

 int SEN0473::init_sensor()
 {
     px4_usleep(2000_ms);

     if (!set_passive_mode()) {
	 PX4_ERR("failed to switch to passive mode");
	 return PX4_ERROR;
     }

     px4_usleep(50_ms);
     PX4_INFO("Gravity H2 sensor initialised");
     return PX4_OK;
 }

 void SEN0473::sensor_compose_msg(bool publish)
 {
     float ppm = 0;
     float temp = 0.f;

     if (!query_all(ppm, temp)) {
	 return;   // silently drop – RunImpl() will detect timeout
     }

     measurement_time  = hrt_absolute_time();
     measurement_index++;
     measured_hydrogen     = ppm;
     measured_temperature  = temp;

     if (publish) {
	 sensor_hydrogen_s msg{};
	 msg.timestamp         = hrt_absolute_time();
	 msg.timestamp_sample  = measurement_time;
	 msg.hydrogen          = measured_hydrogen;
	 msg.temperature       = measured_temperature;
	 _sensor_hydrogen_pub.publish(msg);
     }
 }

 void SEN0473::RunImpl()
 {
     switch (_state) {
     case sen0473_state::INIT:
	 if (init_sensor() == PX4_OK) {
	     _state = sen0473_state::MEASUREMENT;
	 }
	 break;

     case sen0473_state::MEASUREMENT:
	 if (hrt_elapsed_time(&measurement_time) > 200_ms) {
	     sensor_compose_msg(true);
	 }

	 if (hrt_elapsed_time(&measurement_time) > 3_s) {
	     _state = sen0473_state::ERROR_READOUT;
	 }
	 break;

     case sen0473_state::ERROR_READOUT:
     case sen0473_state::ERROR_GENERAL:
	 if (_last_state != _state) {
	    // PX4_WARN("sensor readout error, retrying …");
	 }

	_state = sen0473_state::INIT;

	 break;
     }

     if (_last_state != _state) {
	 _time_in_state = hrt_absolute_time();
	 _last_state    = _state;
     }
 }

 // ------------------------------------------------------------------------
 //  CLI helpers
 // ------------------------------------------------------------------------

 void SEN0473::custom_method(const BusCLIArguments &cli)
 {
     switch (cli.custom1) {
     case 1: // values
	 PX4_INFO("H2 %d ppm (%.3fs ago), T %.2f C",
		  measured_hydrogen,
		  (double)hrt_elapsed_time(&measurement_time) / 1e6,
		  (double)measured_temperature);
	 break;

     case 2: // reset
	 _state = sen0473_state::INIT;
	 break;
     }
 }

 void SEN0473::print_status()
 {
     PX4_INFO("Gravity H2 sensor @0x%02x", get_device_address());
     I2CSPIDriverBase::print_status();
     PX4_INFO("State: %s", sen0473_state_names[_state]);
 }

 void SEN0473::print_usage()
 {
     PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
 ### Description
 Driver for the DFRobot Gravity SEN0473 hydrogen sensor (I²C).
 Make sure to preheat the sensor before usage, at least 5min or 24h after extended storage.
 )DESCR_STR");

     PRINT_MODULE_USAGE_NAME("sen0473", "driver");
     PRINT_MODULE_USAGE_COMMAND("start");
     PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true /*I2C*/, false /*SPI*/);
     PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(H2_I2C_ADDR_DEFAULT);
     PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
     PRINT_MODULE_USAGE_COMMAND_DESCR("values", "Print latest reading");
     PRINT_MODULE_USAGE_COMMAND_DESCR("reset",  "Re‑initialise sensor");
 }

 extern "C" __EXPORT int sen0473_main(int argc, char *argv[])
 {
     using ThisDriver = SEN0473;

     BusCLIArguments cli{true, false};
     cli.default_i2c_frequency = 100000;        // spec recommends ≤100 kHz
     cli.i2c_address           = H2_I2C_ADDR_DEFAULT;

     const char *verb = cli.parseDefaultArguments(argc, argv);
     if (!verb) {
	 ThisDriver::print_usage();
	 return -1;
     }

     BusInstanceIterator iterator(MODULE_NAME, cli, DRV_HYDROGEN_DEVTYPE_SEN0473);

     if (!strcmp(verb, "start"))  { return ThisDriver::module_start(cli, iterator); }
     if (!strcmp(verb, "stop"))   { return ThisDriver::module_stop(iterator);        }
     if (!strcmp(verb, "status")) { return ThisDriver::module_status(iterator);      }

     if (!strcmp(verb, "values")) {
	 cli.custom1 = 1;
	 return ThisDriver::module_custom_method(cli, iterator, false);
     }

     if (!strcmp(verb, "reset"))  {
	 cli.custom1 = 2;
	 return ThisDriver::module_custom_method(cli, iterator, false);
     }

     ThisDriver::print_usage();
     return -1;
 }
