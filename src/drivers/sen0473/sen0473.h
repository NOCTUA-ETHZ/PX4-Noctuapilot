/****************************************************************************
 *
 *   Copyright (c) 2021‑2025 PX4 Development Team. All rights reserved.
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
 * @file sen0473.h
 *
 * Header for the DFRobot Gravity SEN0473 hydrogen‑gas sensor driver.
 *
 * The sensor communicates with fixed‑length 9‑byte frames over I²C.  CRC‑8
 * is **not** used; frames are protected with a two‑complement checksum as
 * specified in the DFRobot reference library.
 *
 * Author: Nick Truttmann <nick.truttmann@px4.io>
 */

 #pragma once

 #include <drivers/device/i2c.h>
 #include <drivers/drv_hrt.h>
 #include <mathlib/mathlib.h>
 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>
 #include <px4_platform_common/i2c_spi_buses.h>
 #include <uORB/PublicationMulti.hpp>
 #include <uORB/topics/sensor_hydrogen.h>

 using namespace time_literals;

 // ------------------------------------------------------------------------
 //  Sensor defaults & command set
 // ------------------------------------------------------------------------

 // I²C addresses (SEL=0, A1/A0 DIP switches)
 #define H2_I2C_ADDR_DEFAULT  0x74
 #define H2_I2C_ADDR_ALT_1    0x75
 #define H2_I2C_ADDR_ALT_2    0x76
 #define H2_I2C_ADDR_ALT_3    0x77

 // Command bytes (see DFRobot_MultiGasSensor.h)
 #define CMD_SET_ACQ_MODE     0x78   ///< set active/passive upload mode
 #define CMD_GET_ALL          0x88   ///< read hydrogen + temperature + decimals

 // Acquisition‑mode arguments for CMD_SET_ACQ_MODE
 static constexpr uint8_t ACQUIRE_MODE_ACTIVE  = 0x03;   ///< sensor pushes every second
 static constexpr uint8_t ACQUIRE_MODE_PASSIVE = 0x04;   ///< sensor replies only when queried

 // ------------------------------------------------------------------------
 //  Driver state machine
 // ------------------------------------------------------------------------

 enum sen0473_state {
     ERROR_GENERAL,
     ERROR_READOUT,
     INIT,
     MEASUREMENT
 };

 static constexpr const char *sen0473_state_names[] = {
     "General error",
     "Readout error",
     "Initialization",
     "Measurement"
 };

 // Sensor information stored for debug / uORB metadata
 struct sen_info {
     uint32_t serial_number{0};
 };

 // ------------------------------------------------------------------------
 //  Driver class declaration
 // ------------------------------------------------------------------------

 class SEN0473 : public device::I2C,
		 public ModuleParams,
		 public I2CSPIDriver<SEN0473>
 {
 public:
     explicit SEN0473(const I2CSPIDriverConfig &config);
     ~SEN0473() override = default;

     // PX4 module hooks ------------------------------------------------------
     static void  print_usage();
     void         RunImpl();

     int          init() override;
     int          probe() override;
     int          init_sensor();
     void         print_status() override;

     void         custom_method(const BusCLIArguments &cli);

 private:
     // Low‑level I²C helpers --------------------------------------------------
     bool         write_frame(const uint8_t frame[9]);     ///< write 9‑byte command
     bool         read_frame(uint8_t frame[9]);            ///< read 9‑byte reply & validate checksum
     bool         query_all(float &ppm, float &tempC);     ///< request combined gas & temperature
     bool         set_passive_mode();

     // High‑level processing --------------------------------------------------
     void         sensor_compose_msg(bool publish);

     // Measurement data -------------------------------------------------------
     int          measured_hydrogen{0};      ///< ppm
     float        measured_temperature{0.f}; ///< deg C
     uint64_t     measurement_time{0};
     uint32_t     measurement_index{0};

     // State machine ----------------------------------------------------------
     sen_info     _sen0473_info{};
     int          _state{INIT};
     int          _last_state{INIT};
     uint64_t     _time_in_state{hrt_absolute_time()};

     // uORB -------------------------------------------------------------------
     uORB::PublicationMulti<sensor_hydrogen_s> _sensor_hydrogen_pub{ORB_ID(sensor_hydrogen)};

     // Parameters -------------------------------------------------------------
     DEFINE_PARAMETERS(
	 (ParamInt<px4::params::SENS_EN_SEN0473>) _param_sens_en_sen0473
     )
 };

 extern "C" __EXPORT int sen0473_main(int argc, char *argv[]);
