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

 IE_Fuelcell::IE_Fuelcell(const char *device_name, const char *baud_rate_parameter) :
	 OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
 {
	 strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
	 _stored_device_name[sizeof(_stored_device_name) - 1] = '\0'; // Ensure null-termination
	 PX4_INFO("IE_Fuelcell created");
 }

 IE_Fuelcell::~IE_Fuelcell()
 {
	PX4_WARN("Shutting down driver");
	 close(_uart_fd);
 }

 int IE_Fuelcell::initializeUART()
 {
	PX4_INFO("Initializing UART");
	mavlink_log_info(_mavlink_log_pub, "Succesfully try connecting to fc");

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
	 case 9600:
		 baud_rate_posix = B9600;
		 break;

	 case 19200:
		 baud_rate_posix = B19200;
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

	 case 921600:
	 		baud_rate_posix = B921600;
	 	break;
	}

	 // start serial port
	 _uart_fd = open(_stored_device_name, O_RDWR | O_NOCTTY);

	 if (_uart_fd < 0) { err(1, "could not open %s", _stored_device_name); }

	 int ret = 0;
	 struct termios uart_config {};
	 ret = tcgetattr(_uart_fd, &uart_config);

	 if (ret < 0) { err(1, "failed to get attr"); }

	// Set non-canonical mode to read raw data
	uart_config.c_lflag &= ~(ICANON | ECHO);
	uart_config.c_cc[VMIN]  = 0;
	uart_config.c_cc[VTIME] = 1; // 0.1 sec read timeout

	uart_config.c_oflag &= ~ONLCR;  // Do not translate NL to CR
	uart_config.c_cflag &= ~CRTSCTS; // Disable hardware flow control

	 // Set baud rate
	 ret = cfsetispeed(&uart_config, baud_rate_posix);

	 if (ret < 0) { err(1, "failed to set input speed"); }

	 ret = cfsetospeed(&uart_config, baud_rate_posix);

	 if (ret < 0) { err(1, "failed to set output speed"); }

	 ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	 if (ret < 0) { err(1, "failed to set attr"); }

	 FD_ZERO(&_uart_fd_set);
	 FD_SET(_uart_fd, &_uart_fd_set);

	tcflush(_uart_fd, TCIOFLUSH);


	 PX4_INFO("Successfully connected");
	 mavlink_log_info(_mavlink_log_pub, "Succesfully connected to Fuel Cell");

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

	 fuel_cell_s _data;
	 readData(_data);

	 // Run at 5 Hz
	ScheduleDelayed(200_ms);
 }

 int IE_Fuelcell::readData(fuel_cell_s &data)
 {
	mavlink_log_info(_mavlink_log_pub, "Reading FC DATA");
     char buf[256];
     memset(buf, 0, sizeof(buf));

     // Wait for data to be available
     fd_set readfds = _uart_fd_set;
     struct timeval timeout = {0, 100000}; // 100ms timeout
     int ret = select(_uart_fd + 1, &readfds, NULL, NULL, &timeout);
     if (ret <= 0) {
	 PX4_ERR("UART read timeout");
	 return -1;
     }

     // Read data from the UART
     ssize_t n = read(_uart_fd, buf, sizeof(buf) - 1);
     if (n <= 0) {
	 PX4_ERR("Failed to read from UART");
	 return -1;
     }
     buf[n] = '\0';

     // Look for the live data message marked by '<' and '>'
     char *start = strchr(buf, '<');
     char *end   = strchr(buf, '>');
     if (!start || !end || start >= end) {
	 PX4_ERR("Invalid data format received");
	 return -1;
     }

     // Copy the data between '<' and '>' into a temporary buffer
     size_t data_len = end - start - 1;
     if (data_len >= sizeof(buf)) {
	 PX4_ERR("Data string too long");
	 return -1;
     }
     char dataStr[256];
     memcpy(dataStr, start + 1, data_len);
     dataStr[data_len] = '\0';

     // Tokenize the string using comma as a delimiter.
     // We'll store pointers to tokens in an array.
     char *tokens[20];
     int token_count = 0;
     char *token = strtok(dataStr, ",");
     while (token != NULL && token_count < 20) {
	 tokens[token_count++] = token;
	 token = strtok(NULL, ",");
     }

     // Ensure we have the expected number of tokens (at least 12).
     if (token_count < 12) {
	 PX4_ERR("Incomplete data received");
	 return -1;
     }

     // Parse tokens based on the new UART format:
     // Assuming tokens layout (indexes):
     // [0] Tank pressure, [1] Regulated pressure, [2] Battery voltage,
     // [3] Output power, [4] SPM power draw, [5] (unused or reserved),
     // [6] Battery power, [7] PSU state, [8] Main error code,
     // [9] Sub error code, [10] Info string (ignored), [11] Checksum (ignored)
     data.tankpressure = (float)atof(tokens[0]);
     data.regpressure  = (float)atof(tokens[1]);
     data.voltage      = (float)atof(tokens[2]);
     data.outputpower  = (float)atof(tokens[3]);
     data.spmpower     = (float)atof(tokens[4]);
     // Skip tokens[5] as it is not used in the FuelCell.msg definition
     data.battpower    = (float)atof(tokens[6]);
     data.psustate     = atoi(tokens[7]);
     data.mainerror    = atoi(tokens[8]);
     data.suberror     = atoi(tokens[9]);
     // tokens[10] (info string) and tokens[11] (checksum) are not used here

     publishData(data);
     return PX4_OK;
 }


int IE_Fuelcell::publishData(const fuel_cell_s &data)
{
	mavlink_log_info(_mavlink_log_pub, "Publishing FC DATA");
	// Publish dummy fuel cell data
	fuel_cell_s fuel_cell_msg{};
	fuel_cell_msg.timestamp = hrt_absolute_time();
	fuel_cell_msg.tankpressure = data.tankpressure;
	fuel_cell_msg.regpressure = data.regpressure;
	fuel_cell_msg.voltage = data.voltage;
	fuel_cell_msg.outputpower = data.outputpower;
	fuel_cell_msg.spmpower = data.spmpower;
	fuel_cell_msg.battpower = data.battpower;
	fuel_cell_msg.psustate = data.psustate;
	fuel_cell_msg.mainerror = data.mainerror;
	fuel_cell_msg.suberror = data.suberror;

	// Publish the message
	_fuel_cell_pub.publish(fuel_cell_msg);

	return PX4_OK;
}


 int IE_Fuelcell::task_spawn(int argc, char *argv[])
 {
	// Include checks for correctness
	 if (argc < 3) {
		 return print_usage("missing arguments");
	 }
	 if (argc > 3) {
		 return print_usage("too many arguments");
	 }

	PX4_INFO("Starting task_spawn");
	 const char *device_name = argv[1];
	 const char *baud_rate_parameter_value = argv[2];

	 IE_Fuelcell *instance = new IE_Fuelcell(device_name, baud_rate_parameter_value);

	 if (instance) {
		 _object.store(instance);
		 _task_id = task_id_is_work_queue;
		 instance->ScheduleNow();
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


 The driver needs to be enabled using the parameter `IEFC_SER_CFG` and the baudrate needs to be set using IEFC_BAUD.

 The command to start this driver is: `$ ie_fuelcell start <UART device> <baud rate>`
 )DESCR_STR");

	 PRINT_MODULE_USAGE_NAME("ie_fuelcell", "driver");
	 PRINT_MODULE_USAGE_COMMAND("start");
	 PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	 return 0;
 }

 int IE_Fuelcell::print_status()
 {
	 if (_uart_initialized) {
		 PX4_INFO("Driver is running");
		 return PX4_OK;
	 } else {
		 PX4_INFO("Driver is not running");
		 return PX4_ERROR;
	 }
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
