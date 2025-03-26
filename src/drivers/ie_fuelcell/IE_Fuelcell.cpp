/**
 * @file IE_Fuelcell.cpp
 *
 * IE Fuel Cell Driver
 *
 * references:
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
	 PX4_INFO("IE_Fuelcell created");
 }

 IE_Fuelcell::~IE_Fuelcell()
 {
	PX4_WARN("Shuting down driver");
	 close(_uart_fd);
 }

 int IE_Fuelcell::initializeUART()
 {
	PX4_INFO("Initializing UART");
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
	PX4_INFO("Running");
	 if (should_exit()) {
		 ScheduleClear();
		 exit_and_cleanup();
		 return;
	 }


	 if (!_uart_initialized) {
		 initializeUART();
		 _uart_initialized = true;
	 }

	// Publish dummy fuel cell data
	fuel_cell_s fuel_cell_msg{};

	// Use hrt_absolute_time() for timestamp
	fuel_cell_msg.timestamp = hrt_absolute_time();

	// Dummy data generation
	fuel_cell_msg.tankpressure = 50.5f;     // example pressure in bar
	fuel_cell_msg.regpressure = 2.3f;       // regulator pressure
	fuel_cell_msg.voltage = 12.6f;          // system voltage
	fuel_cell_msg.outputpower = 100.0f;     // output power in watts
	fuel_cell_msg.spmpower = 75.5f;         // some power metric
	fuel_cell_msg.battpower = 25.5f;        // battery power
	fuel_cell_msg.psustate = 1;             // PSU state (example)

	// Example info bytes
	fuel_cell_msg.info[0] = 0xAA;
	fuel_cell_msg.info[1] = 0x55;

	// Publish the message
	_fuel_cell_pub.publish(fuel_cell_msg);

	ScheduleDelayed(1_s);
 }

 int IE_Fuelcell::task_spawn(int argc, char *argv[])
 {
	PX4_INFO("Starting task_spawn");
	 const char *device_name = argv[1];
	 const char *baud_rate_parameter_value = argv[2];

	 IE_Fuelcell *instance = new IE_Fuelcell(device_name, baud_rate_parameter_value);

	 if (instance) {
		 _object.store(instance);
		 _task_id = task_id_is_work_queue;
		 //instance->ScheduleNow();
		 instance->ScheduleOnInterval(1_s, 0);
		 PX4_INFO("Task spawned");
		 return OK;

	 } else {
		 PX4_ERR("alloc failed");
	 }

	 delete instance;
	 _object.store(nullptr);
	 _task_id = -1;


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


 The driver needs to be enabled using the parameter `IEFC_SER_CFG` and the baudrate needs to be set correctly.

 The command to start this driver is: `$ ie_fuelcell start <UART device> <baud rate>`
 )DESCR_STR");

	 PRINT_MODULE_USAGE_NAME("ie_fuelcell", "driver");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	 return 0;
 }

 int IE_Fuelcell::print_status()
 {
	 return 1;
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
