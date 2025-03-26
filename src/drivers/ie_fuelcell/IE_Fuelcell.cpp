/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file Roboclaw.cpp
 *
 * Roboclaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

 #include "IE_Fuelcell.hpp"
 #include <termios.h>

 IE_Fuelcell::IE_Fuelcell(const char *device_name, const char *bad_rate_parameter) :
	 OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
 {
	 strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
	 _stored_device_name[sizeof(_stored_device_name) - 1] = '\0'; // Ensure null-termination

	 strncpy(_stored_baud_rate_parameter, bad_rate_parameter, sizeof(_stored_baud_rate_parameter) - 1);
	 _stored_baud_rate_parameter[sizeof(_stored_baud_rate_parameter) - 1] = '\0'; // Ensure null-termination
 }

 IE_Fuelcell::~IE_Fuelcell()
 {
	 close(_uart_fd);
 }

 int IE_Fuelcell::initializeUART()
 {
	 // The Roboclaw has a serial communication timeout of 10ms
	 // Add a little extra to account for timing inaccuracy
	 static constexpr int TIMEOUT_US = 11_ms;
	 _uart_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };

	 int32_t baud_rate_parameter_value{0};
	 int32_t baud_rate_posix{0};
	 param_get(param_find(_stored_baud_rate_parameter), &baud_rate_parameter_value);

	 switch (baud_rate_parameter_value) {
	 case 0: // Auto
	 default:
		 PX4_ERR("Please configure the port's baud_rate_parameter_value");
		 break;

	 case 2400:
		 baud_rate_posix = B2400;
		 break;

	 case 9600:
		 baud_rate_posix = B9600;
		 break;

	 case 19200:
		 baud_rate_posix = B19200;
		 break;

	 case 38400:
		 baud_rate_posix = B38400;
		 break;

	 case 57600:
		 baud_rate_posix = B57600;
		 break;

	 case 115200:
		 baud_rate_posix = B115200;
		 break;

	 case 230400:
		 baud_rate_posix = B230400;
		 break;

	 case 460800:
		 baud_rate_posix = B460800;
		 break;
	 }

	 // start serial port
	 _uart_fd = open(_stored_device_name, O_RDWR | O_NOCTTY);

	 if (_uart_fd < 0) { err(1, "could not open %s", _stored_device_name); }

	 int ret = 0;
	 struct termios uart_config {};
	 ret = tcgetattr(_uart_fd, &uart_config);

	 if (ret < 0) { err(1, "failed to get attr"); }

	 uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	 uart_config.c_cflag &= ~CRTSCTS;

	 // Set baud rate
	 ret = cfsetispeed(&uart_config, baud_rate_posix);

	 if (ret < 0) { err(1, "failed to set input speed"); }

	 ret = cfsetospeed(&uart_config, baud_rate_posix);

	 if (ret < 0) { err(1, "failed to set output speed"); }

	 ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	 if (ret < 0) { err(1, "failed to set attr"); }

	 FD_ZERO(&_uart_fd_set);
	 FD_SET(_uart_fd, &_uart_fd_set);

	//  // Make sure the device does respond
	//  static constexpr int READ_STATUS_RESPONSE_SIZE = 6;
	//  uint8_t response_buffer[READ_STATUS_RESPONSE_SIZE];

	//  if (receiveTransaction(Command::ReadStatus, response_buffer, READ_STATUS_RESPONSE_SIZE) < READ_STATUS_RESPONSE_SIZE) {
	// 	 PX4_ERR("No valid response, stopping driver");
	// 	 request_stop();
	// 	 return ERROR;

	//  } else {
	// 	 PX4_INFO("Successfully connected");
	// 	 return OK;
	//  }
	 PX4_INFO("Successfully connected");
	 return OK;
 }


 void IE_Fuelcell::Run()
 {
	 if (should_exit()) {
		 ScheduleClear();
		 exit_and_cleanup();
		 return;
	 }


	 if (!_uart_initialized) {
		 initializeUART();
		 _uart_initialized = true;
	 }

	 // check for parameter updates
	 if (_parameter_update_sub.updated()) {
		 // Read from topic to clear updated flag
		 parameter_update_s parameter_update;
		 _parameter_update_sub.copy(&parameter_update);

		 updateParams();
	 }


	//  if (readEncoder() != OK) {
	// 	 PX4_ERR("Error reading encoders");
	//  }
 }

 int IE_Fuelcell::task_spawn(int argc, char *argv[])
 {
	 const char *device_name = argv[1];
	 const char *baud_rate_parameter_value = argv[2];

	 IE_Fuelcell *instance = new IE_Fuelcell(device_name, baud_rate_parameter_value);

	 if (instance) {
		 _object.store(instance);
		 _task_id = task_id_is_work_queue;
		 instance->ScheduleNow();
		 return OK;

	 } else {
		 PX4_ERR("alloc failed");
	 }

	 delete instance;
	 _object.store(nullptr);
	 _task_id = -1;

	 printf("Ending task_spawn");

	 return ERROR;
 }

 int IE_Fuelcell::custom_command(int argc, char *argv[])
 {
	 return print_usage("unknown command");
 }

 int IE_Fuelcell::print_usage(const char *reason)
 {
	 if (reason) {
		 PX4_WARN("%s\n", reason);
	 }

	 PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
 ### Description

 This driver communicates over UART with the [Intelligent Energy FuelCell driver](https://www.intelligent-energy.com).

 TEMPLATE TEMPLATE

 In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and
 your flight controller's UART port should be connected to the Roboclaw as shown in the documentation.
 The driver needs to be enabled using the parameter `RBCLW_SER_CFG`, the baudrate needs to be set correctly and
 the address `RBCLW_ADDRESS` needs to match the ESC configuration.

 The command to start this driver is: `$ roboclaw start <UART device> <baud rate>`
 )DESCR_STR");

	 PRINT_MODULE_USAGE_NAME("ie_fuelcell", "driver");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	 return 0;
 }

 int IE_Fuelcell::print_status()
 {
	 return 0;
 }


 bool IE_Fuelcell::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
	unsigned num_outputs, unsigned num_control_groups_updated)
{
	return false;
}

void IE_Fuelcell::updateParams()
{
	PX4_INFO("Fuel cell parameters updated");
}





 extern "C" __EXPORT int ie_fuelcell_main(int argc, char *argv[])
 {
	 return IE_Fuelcell::main(argc, argv);
 }
